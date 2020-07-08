from enum import Enum, auto
from typing import List

import pybullet as p
import pybullet_data

from lobster_simulator.simulation_time import SimulationTime
from lobster_simulator.tools.Constants import GRAVITY


class Frame(Enum):
    LINK_FRAME = 1
    WORLD_FRAME = 2


class PybulletAPI:



    __instance = None

    def __init__(self, time_step: SimulationTime, gui=False):
        self.gui = gui

        self.physics_client_id = -1
        if gui:
            self.physics_client_id = p.connect(p.GUI)
        else:
            self.physics_client_id = p.connect(p.DIRECT)

        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setTimeStep(time_step.seconds)
        p.setGravity(0, 0, -GRAVITY)
        p.loadURDF("plane.urdf", [0, 0, -100])
        p.loadURDF("plane.urdf", [0, 0, 0])

    @staticmethod
    def initialize(time_step: SimulationTime, gui: bool):
        PybulletAPI.__instance = PybulletAPI(time_step, gui)

    @staticmethod
    def get_pybullet_id():
        return PybulletAPI.__instance.physics_client_id

    @staticmethod
    def gui():
        return PybulletAPI.__instance.gui

    @staticmethod
    def setTimeStep(time_step: SimulationTime):
        p.setTimeStep(time_step.seconds)

    @staticmethod
    def stepSimulation():
        p.stepSimulation()

    @staticmethod
    def addUserDebugParameter(name, rangeMin, rangeMax, startValue):
        if PybulletAPI.__instance.gui:
            return p.addUserDebugParameter(name, rangeMin, rangeMax, startValue)

    @staticmethod
    def readUserDebugParameter(itemUniqueId: int) -> float:
        print("gui", PybulletAPI.__instance.gui)
        if PybulletAPI.__instance.gui:
            return p.readUserDebugParameter(itemUniqueId)

    @staticmethod
    def moveCameraToPosition(position: List[float]):
        if PybulletAPI.__instance.gui:
            camera_info = p.getDebugVisualizerCamera()
            p.resetDebugVisualizerCamera(
                cameraDistance=camera_info[10],
                cameraYaw=camera_info[8],
                cameraPitch=camera_info[9],
                cameraTargetPosition=position
            )

    @staticmethod
    def resetBasePositionAndOrientation(objectUniqueId: int, posObj: List[float], ornObj: List[float]):
        p.resetBasePositionAndOrientation(objectUniqueId, posObj, ornObj)

    @staticmethod
    def resetBaseVelocity(objectUniqueId: int, linearVelocity: List[float], angularVelocity: List[float]):
        p.resetBaseVelocity(objectUniqueId, linearVelocity, angularVelocity)

    @staticmethod
    def applyExternalForce(objectUniqueId: int, forceObj: List[float], posObj: List[float], frame: Frame):

        p.applyExternalForce(objectUniqueId, -1, forceObj, posObj, frame.value)
