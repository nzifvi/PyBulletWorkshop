import mujoco
import mujoco.viewer
import numpy as np
import time
import json
import math


def loadConfig(path: str) -> dict:
    try:
        with open(path, "r") as file:
            return json.load(file)
    except FileNotFoundError:
        print(f"! {path} not found")
        exit(1)


class Environment:
    def __init__(self, timeStep):
        self._timeStep      = timeStep
        self._objectiveGeom = -1                 # geom id of the objective (was _objectiveId)
        self._agentUrdf     = "assets/agent.urdf"
        self._agentBodyName = None

        self._spec     = None
        self.model     = None
        self.data      = None
        self.viewer    = None
        self._renderer = None

    # --- model construction -------------------------------------------------
    def loadEnvironment(self, level: str) -> None:
        self._spec = mujoco.MjSpec.from_file(self._agentUrdf)
        world = self._spec.worldbody
        world.add_geom(
            type=mujoco.mjtGeom.mjGEOM_PLANE,
            size=[0, 0, 0.1],
            rgba=[0.6, 0.6, 0.6, 1.0],
        )
        world.add_light(pos=[0, 0, 10], dir=[0, 0, -1])
        agentRoot = world.bodies[0]
        agentRoot.add_freejoint()
        self._agentBodyName = agentRoot.name
        camQuat = np.zeros(4)
        mujoco.mju_mat2Quat(
            camQuat,
            np.array([0, 0, -1,
                      -1, 0,  0,
                       0, 1,  0], dtype=float),
        )
        agentRoot.add_camera(name="agentcam", pos=[0.3, 0.0, 0.3], quat=camQuat, fovy=90)

        env = loadConfig("configs/env.json")
        if level not in env:
            print(f"! {level} not found in configs/env.json")
            exit(2)
        self._spawnBarriers(world, env[level]["arenaWalls"])

        levels = loadConfig("configs/levels.json")
        if level not in levels:
            print(f"! {level} not found in configs/levels.json")
            exit(3)
        levelData = levels[level]
        self._spawnBarriers(world, levelData["barriers"])
        if "objective" in levelData:
            objGeom = self._spawnBarriers(
                world, [levelData["objective"]], colour=[0.0, 1.0, 0.0, 1.0]
            )[0]
            objGeom.name = "objective"

        self.model = self._spec.compile()
        self.data  = mujoco.MjData(self.model)
        self.model.opt.timestep = self._timeStep
        self._objectiveGeom = mujoco.mj_name2id(
            self.model, mujoco.mjtObj.mjOBJ_GEOM, "objective"
        )
        mujoco.mj_forward(self.model, self.data)

    def openViewer(self):
        self.viewer = mujoco.viewer.launch_passive(
            self.model, self.data,
            show_left_ui=False,
            show_right_ui=False,
        )
        return self.viewer

    def step(self) -> None:
        mujoco.mj_step(self.model, self.data)

    def _spawnBarriers(self, world, barriers, colour=[1.0, 1.0, 1.0, 1.0]) -> list:
        geoms = []
        for barrier in barriers:
            geoms.append(world.add_geom(
                type=mujoco.mjtGeom.mjGEOM_BOX,
                size=[dim / 2 for dim in barrier["size"]],
                pos=barrier["pos"],
                rgba=colour,
            ))
        return geoms

    def updateCameraPosition(self, targetPos, dist=10.0, yaw=0.0, pitch=-89.9) -> None:
        if self.viewer is None:
            return
        cam = self.viewer.cam
        cam.lookat[:] = targetPos
        cam.distance  = dist
        cam.azimuth   = yaw
        cam.elevation = pitch
        self.viewer.sync()


class Agent:
    def __init__(self, env):
        self._env         = env
        self._bodyId      = None
        self._freeQAdr    = None
        self._freeVAdr    = None
        self._sensorLinks = {}

    def load(self):
        model, data = self._env.model, self._env.data
        agent = loadConfig("configs/agent.json")

        self._bodyId = mujoco.mj_name2id(
            model, mujoco.mjtObj.mjOBJ_BODY, self._env._agentBodyName
        )
        freeJoint      = model.body_jntadr[self._bodyId]
        self._freeQAdr = model.jnt_qposadr[freeJoint]
        self._freeVAdr = model.jnt_dofadr[freeJoint]

        q = self._freeQAdr
        data.qpos[q : q + 3]     = agent["pos"]
        data.qpos[q + 3 : q + 7] = [1.0, 0.0, 0.0, 0.0]
        mujoco.mj_forward(model, data)

        for name in ("leftIR", "rightIR"):
            self._sensorLinks[name] = mujoco.mj_name2id(
                model, mujoco.mjtObj.mjOBJ_BODY, name
            )

    def getPose(self) -> tuple:
        data = self._env.data
        return data.xpos[self._bodyId], data.xmat[self._bodyId]

    def setVelocity(self, bearing: float, mag: float) -> None:
        data = self._env.data
        yaw  = math.radians(90 - bearing)

        q = self._freeQAdr
        data.qpos[q + 3] = math.cos(yaw / 2)
        data.qpos[q + 4] = 0.0
        data.qpos[q + 5] = 0.0
        data.qpos[q + 6] = math.sin(yaw / 2)

        v = self._freeVAdr
        data.qvel[v : v + 3] = [mag * math.cos(yaw), mag * math.sin(yaw), 0.0]
        data.qvel[v + 3 : v + 6] = 0.0

    def stop(self) -> None:
        v = self._freeVAdr
        self._env.data.qvel[v : v + 6] = 0.0

    def readSensors(self, maxRange=12.0) -> tuple:
        model, data = self._env.model, self._env.data
        readings = {}
        geomid   = np.array([-1], dtype=np.int32)

        for name in ("leftIR", "rightIR"):
            bid = self._sensorLinks[name]
            src = np.array(data.xpos[bid], dtype=np.float64)
            mat = data.xmat[bid]
            fwd = np.array([mat[0], mat[3], mat[6]], dtype=np.float64)
            dist = mujoco.mj_ray(
                model, data, src, fwd,
                None,
                1,
                self._bodyId,
                geomid,
            )
            readings[name] = maxRange if (dist < 0.0 or dist > maxRange) else dist
        return readings["leftIR"], readings["rightIR"]

    def renderCamera(self, width=160, height=160):
        if self._env._renderer is None:
            self._env._renderer = mujoco.Renderer(self._env.model, height, width)
        r = self._env._renderer
        r.update_scene(self._env.data, camera="agentcam")
        return r.render()

    def getBearing(self) -> float:
        _, mat = self.getPose()
        yaw = math.atan2(mat[3], mat[0])
        return (90 - math.degrees(yaw)) % 360

    def castRay(self, bearing, maxRange=12.0, startOffset=0.35):
        model, data = self._env.model, self._env.data
        pos, _ = self.getPose()
        yaw = math.radians(90.0 - bearing)
        d   = np.array([math.cos(yaw), math.sin(yaw), 0.0], dtype=np.float64)
        src = np.array([pos[0] + d[0] * startOffset,
                        pos[1] + d[1] * startOffset,
                        pos[2]], dtype=np.float64)
        geomid = np.array([-1], dtype=np.int32)
        dist = mujoco.mj_ray(model, data, src, d, None, 1, self._bodyId, geomid)
        if dist < 0.0 or dist > maxRange:
            return maxRange - startOffset, -1
        return dist, int(geomid[0])

    def seeObjective(self, radHalf=35.0, rays=9, maxRange=12.0, startOffset=0.35):
        model, data = self._env.model, self._env.data
        objGeom = self._env._objectiveGeom
        if objGeom < 0:
            return False, 0.0, maxRange

        pos, _ = self.getPose()
        base   = self.getBearing()
        geomid = np.array([-1], dtype=np.int32)

        hitOffsets, hitDist = [], []
        for k in range(rays):
            off = -radHalf + (2.0 * radHalf) * (k / (rays - 1))
            yaw = math.radians(90 - (base + off))
            d   = np.array([math.cos(yaw), math.sin(yaw), 0.0], dtype=np.float64)
            src = np.array([pos[0] + d[0] * startOffset,
                            pos[1] + d[1] * startOffset,
                            pos[2]], dtype=np.float64)
            dist = mujoco.mj_ray(model, data, src, d, None, 1, self._bodyId, geomid)
            if dist >= 0.0 and dist <= maxRange and int(geomid[0]) == objGeom:
                hitOffsets.append(off)
                hitDist.append(dist)

        if not hitOffsets:
            return False, 0.0, maxRange
        return True, sum(hitOffsets) / len(hitOffsets), min(hitDist)

    def atObjective(self) -> bool:
        objGeom = self._env._objectiveGeom
        if objGeom < 0:
            return False
        data = self._env.data
        for i in range(data.ncon):
            c = data.contact[i]
            if objGeom in (c.geom1, c.geom2):
                return True
        return False


if __name__ == "__main__":
    renderRate = 5
    frame      = 0
    timeStep   = 1 / 240
    env   = Environment(timeStep=timeStep)
    agent = Agent(env)

    env.loadEnvironment(level="level1")
    agent.load()

    speed       = 5.0
    seekSpeed   = 2.5
    turnStep    = 15.0
    triggerDist = 2.5
    stopDist    = 0.6
    maxRange    = 12.0

    with env.openViewer() as viewer:
        while viewer.is_running():
            # put your code here!

            # ----------------------| do not delete any of the below code if you want the simulator to work! |----------------------
            env.step()
            env.updateCameraPosition(targetPos=agent.getPose()[0])
            time.sleep(timeStep)
            frame += 1
            if frame % renderRate == 0:
                agent.renderCamera()