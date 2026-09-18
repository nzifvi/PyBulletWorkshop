import pybullet
import pybullet_data
import time
import json
import math

class Environment:
    def __init__(self, timeStep):
        self._physicsClient = pybullet.connect(pybullet.GUI)
        self._timeStep = timeStep
        pybullet.setAdditionalSearchPath(pybullet_data.getDataPath())
        pybullet.setTimeStep(timeStep)

    def loadEnvironment(self, level:str, size = 1.0, mass = 0, colour = [1.0, 1.0, 1.0, 1.0]) -> None:
        pybullet.loadURDF("plane.urdf")
        env = loadConfig("configs/env.json")
        self._spawnBarriers(env["arenaWalls"]["barriers"])

        levels = loadConfig("configs/levels.json")
        if level not in levels:
            print(f"! {level} not found in configs/levels.json")
            exit(2)
        else:
            self._spawnBarriers(levels[level]["barriers"])

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

    def updateCameraPosition(self, targetPos:tuple, dist = 5.0, yaw = 0.0, pitch = -89.9) -> None:
        pybullet.resetDebugVisualizerCamera(
            cameraDistance       = dist,
            cameraYaw            = yaw,
            cameraPitch          = pitch,
            cameraTargetPosition = targetPos,
        )

class Agent:
    def __init__(self, env):
        self._bodyId = None
        self._env = env

    def load(self):
        agent = loadConfig("configs/agent.json")
        halfExtents = [dim / 2 for dim in agent["size"]]
        collisionShape = pybullet.createCollisionShape(
            pybullet.GEOM_BOX,
            halfExtents=halfExtents
        )
        visualShape = pybullet.createVisualShape(
            pybullet.GEOM_BOX,
            halfExtents=halfExtents,
            rgbaColor = agent["colour"]
        )
        self._bodyId = pybullet.createMultiBody(
            baseMass=agent["mass"],
            baseCollisionShapeIndex=collisionShape,
            baseVisualShapeIndex=visualShape,
            basePosition=agent["pos"],
        )

    def getPose(self) -> tuple:
        return pybullet.getBasePositionAndOrientation(self._bodyId)

    def move(self, speed = 2.0, steps = 120) -> None:
        _, orientation = self.getPose()
        _, _, yaw = pybullet.getEulerFromQuaternion(orientation)   # current heading
        velocity = [speed * math.cos(yaw), speed * math.sin(yaw), 0.0]
        for _ in range(steps):
            pybullet.resetBaseVelocity(self._bodyId, linearVelocity=velocity, angularVelocity=[0,0,0])
            pybullet.stepSimulation()
            self._env.updateCameraPosition(targetPos=self.getPose()[0])
            time.sleep(1 / 240)
        pybullet.resetBaseVelocity(self._bodyId, linearVelocity=[0,0,0], angularVelocity=[0,0,0])

    def setAngle(self, angle:float) -> None:
        yaw = math.radians(90 - angle)
        orientation = pybullet.getQuaternionFromEuler([0, 0, yaw])
        position, _ = self.getPose()
        pybullet.resetBasePositionAndOrientation(self._bodyId, position, orientation)

    def runSequence(self, moves:list) -> None:
        for move in moves:
            if isinstance(move, tuple):
                method, *args = move
                method(*args)
            else:
                move()

def loadConfig(path:str) -> dict:
    try:
        with open(path, "r") as file:
            return json.load(file)
    except FileNotFoundError:
        print(f"! {path} not found")
        exit(1)

if __name__ == "__main__":
    timeStep = 1 / 240
    env = Environment(
        timeStep = timeStep
    )
    agent = Agent(env)

    env.loadEnvironment(level = "level1")
    agent.load()

    # define your sequence of movements here!
    agent.runSequence([
        (agent.setAngle, 0.0),
        agent.move,
        (agent.setAngle, 45.0),
        agent.move,
    ])

    time.sleep(2)