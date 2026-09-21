import pybullet
import pybullet_data
import time
import json
import math

class Environment:
    def __init__(self, timeStep):
        self._physicsClient = pybullet.connect(pybullet.GUI)
        self._timeStep      = timeStep
        self._objectiveId   = None

        pybullet.setAdditionalSearchPath(pybullet_data.getDataPath())
        pybullet.setTimeStep(timeStep)

    def loadEnvironment(self, level:str) -> None:
        pybullet.loadURDF("plane.urdf")
        env = loadConfig("configs/env.json")


        levels = loadConfig("configs/levels.json")
        if level not in levels:
            print(f"! {level} not found in configs/levels.json")
            exit(2)
        else:
            levelData = levels[level]
            self._spawnBarriers(levelData["barriers"])
            if "objective" in levelData:
                self._objectiveId = self._spawnBarriers(
                    [levelData["objective"]],
                    colour = [0.0, 1.0, 0.0, 1.0]
                )

    def step(self) -> None:
        pybullet.stepSimulation()

    def _spawnBarriers(self, barriers: list, mass=0, colour=[1.0, 1.0, 1.0, 1.0]) -> list:
        ids = []
        for barrier in barriers:
            halfExtents = [dim / 2 for dim in barrier["size"]]
            collisionShape = pybullet.createCollisionShape(
                pybullet.GEOM_BOX, halfExtents=halfExtents
            )
            visualShape = pybullet.createVisualShape(
                pybullet.GEOM_BOX, halfExtents=halfExtents, rgbaColor=colour
            )
            ids.append(pybullet.createMultiBody(
                baseMass=mass,
                baseCollisionShapeIndex=collisionShape,
                baseVisualShapeIndex=visualShape,
                basePosition=barrier["pos"],
            ))
        return ids

    def updateCameraPosition(self, targetPos:tuple, dist = 10.0, yaw = 0.0, pitch = -89.9) -> None:
        pybullet.resetDebugVisualizerCamera(
            cameraDistance       = dist,
            cameraYaw            = yaw,
            cameraPitch          = pitch,
            cameraTargetPosition = targetPos,
        )

class Agent:
    def __init__(self, env):
        self._bodyId      = None
        self._env         = env
        self._sensorLinks = {}

    def load(self):
        agent = loadConfig("configs/agent.json")
        self._bodyId = pybullet.loadURDF(
            "assets/agent.urdf",
            basePosition = agent["pos"]
        )
        for i in range(pybullet.getNumJoints(self._bodyId)):
            name = pybullet.getJointInfo(self._bodyId, i)[12].decode()
            self._sensorLinks[name] = i

    def getPose(self) -> tuple:
        return pybullet.getBasePositionAndOrientation(self._bodyId)

    def setVelocity(self, bearing:float, mag:float) -> None:
        yaw = math.radians(90 - bearing)
        orientation = pybullet.getQuaternionFromEuler([0, 0, yaw])
        position, _ = self.getPose()
        pybullet.resetBasePositionAndOrientation(self._bodyId, position, orientation)
        velocity = [mag * math.cos(yaw), mag * math.sin(yaw), 0.0]
        pybullet.resetBaseVelocity(
            self._bodyId,
            linearVelocity = velocity
        )

    def stop(self) -> None:
        pybullet.resetBaseVelocity(
            self._bodyId,
            linearVelocity = [0.0, 0.0, 0.0]
        )

    def readSensors(self, maxRange = 12.0) -> tuple:
        readings   = {}
        raySources = []
        raySinks   = []
        names      = []

        for name in ("leftIR", "rightIR"):
            state = pybullet.getLinkState(
                self._bodyId,
                self._sensorLinks[name]
            )
            linkPos         = state[0]
            linkOrientation = state[1]

            rotationMatrix = pybullet.getMatrixFromQuaternion(linkOrientation)
            forward = [rotationMatrix[0], rotationMatrix[3], rotationMatrix[6]]

            raySources.append(linkPos)
            raySinks.append(
                [linkPos[i] + forward[i] * maxRange for i in range(3)]
            )
            names.append(name)

        results = pybullet.rayTestBatch(
            raySources,
            raySinks
        )

        for name, result in zip(names, results):
            hitBodyId, _, hitFrac, _, _ = result
            if hitBodyId < 0:
                readings[name] = maxRange
            else:
                readings[name] = hitFrac * maxRange
        return readings["leftIR"], readings["rightIR"]

    def renderCamera(self, width = 160, height = 160, fov = 90.0, near = 0.02, far = 12.0, eyeHeight = 0.3, forwardOffset = 0.3):
        pos, orientation = self.getPose()
        rotationMatrix = pybullet.getMatrixFromQuaternion(orientation)
        forward = [
            rotationMatrix[0],
            rotationMatrix[3],
            rotationMatrix[6]
        ]
        eye = [
            pos[0] + forward[0] * forwardOffset,
            pos[1] + forward[1] * forwardOffset,
            pos[2] + eyeHeight
        ]
        target = [eye[i] + forward[i] for i in range(3)]

        viewMatrix = pybullet.computeViewMatrix(
            cameraEyePosition    = eye,
            cameraTargetPosition = target,
            cameraUpVector       = [0.0, 0.0, 1.0]
        )
        projectionMatrix = pybullet.computeProjectionMatrixFOV(
            fov     = fov,
            aspect  = width / height,
            nearVal = near,
            farVal  = far
        )
        return pybullet.getCameraImage(
            width,
            height,
            viewMatrix       = viewMatrix,
            projectionMatrix = projectionMatrix,
            renderer         = pybullet.ER_BULLET_HARDWARE_OPENGL
        )

def loadConfig(path:str) -> dict:
    try:
        with open(path, "r") as file:
            return json.load(file)
    except FileNotFoundError:
        print(f"! {path} not found")
        exit(1)

if __name__ == "__main__":
    timeStep = 1 / 240
    env = Environment(timeStep=timeStep)
    agent = Agent(env)

    env.loadEnvironment(level="level2")
    agent.load()

    bearing      = 0.0
    speed        = 5.0
    turnStep     = 15.0
    triggerDist  = 1.25
    maxRange     = 12.0

    renderRate = 5
    frame      = 0
    while True:
        left, right = agent.readSensors(maxRange=maxRange)


        if left < triggerDist and right < triggerDist:
            bearing += turnStep
            mag = 0.0
        elif left < triggerDist:
            bearing += turnStep
            mag = speed
        elif right < triggerDist:
            bearing -= turnStep
            mag = speed
        else:
            mag = speed
        bearing %= 360.0

        agent.setVelocity(bearing=bearing, mag=mag)


        agent.setVelocity(bearing=bearing, mag=speed)
# ----------------------| do not delete any of the below code if you want the simulator to work! |----------------------
        env.step()
        env.updateCameraPosition(targetPos=agent.getPose()[0])
        time.sleep(timeStep)
        frame += 1
        if frame % renderRate == 0:
            agent.renderCamera()