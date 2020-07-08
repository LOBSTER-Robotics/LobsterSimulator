import math
from enum import Enum
from typing import List

import pybullet as p
import pybullet_data

from lobster_simulator.simulation_time import SimulationTime
from lobster_simulator.tools.Constants import GRAVITY


class Frame(Enum):
    LINK_FRAME = 1
    WORLD_FRAME = 2


class PybulletAPI:
    """
    This class is meant as a wrapper for the Pybullet API, but adds support for typing and using custom types.
    """

    KEY_WAS_TRIGGERED = p.KEY_WAS_TRIGGERED

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
        p.loadURDF("plane.urdf", [0, 0, 0], self.getQuaternionFromEuler([math.pi, 0, 0]))

    @staticmethod
    def initialize(time_step: SimulationTime, gui: bool):
        PybulletAPI.__instance = PybulletAPI(time_step, gui)

    @staticmethod
    def changeDynamics(bodyUniqueId: int, linearDamping: float, angularDamping: float):

        p.changeDynamics(bodyUniqueId=bodyUniqueId,
                         linkIndex=-1,
                         linearDamping=linearDamping,
                         angularDamping=angularDamping)

    @staticmethod
    def get_pybullet_id():
        return PybulletAPI.__instance.physics_client_id

    @staticmethod
    def loadURDF(file_name: str, base_position: List[float], base_orientation: List[float]):
        return p.loadURDF(fileName=file_name, basePosition=base_position, baseOrientation=base_orientation)

    @staticmethod
    def getQuaternionFromEuler(euler_angle: List[float]):
        return p.getQuaternionFromEuler(euler_angle)

    @staticmethod
    def getEulerFromQuaternion(quaternion: List[float]):
        return p.getEulerFromQuaternion(quaternion)

    @staticmethod
    def getMatrixFromQuaternion(quaternion: List[float]):
        return p.getMatrixFromQuaternion(quaternion)

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
        if PybulletAPI.__instance.gui:
            return p.readUserDebugParameter(itemUniqueId)

    @staticmethod
    def addUserDebugLine(lineFromXYZ: List[float], lineToXYZ: List[float], lineWidth: float, lineColorRGB: List[float],
                         replaceItemUniqueId: int = -1):

        return p.addUserDebugLine(lineFromXYZ=lineFromXYZ,
                                  lineToXYZ=lineToXYZ,
                                  lineWidth=lineWidth,
                                  lineColorRGB=lineColorRGB,
                                  replaceItemUniqueId=replaceItemUniqueId)

    @staticmethod
    def getKeyboardEvents():
        return p.getKeyboardEvents()

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
    def getBasePositionAndOrientation(objectUniqueId: int):
        return p.getBasePositionAndOrientation(objectUniqueId)

    @staticmethod
    def resetBasePositionAndOrientation(objectUniqueId: int, posObj: List[float], ornObj: List[float]):
        p.resetBasePositionAndOrientation(objectUniqueId, posObj, ornObj)

    @staticmethod
    def getBaseVelocity(objectUniqueId: int):
        return p.getBaseVelocity(objectUniqueId)

    @staticmethod
    def resetBaseVelocity(objectUniqueId: int, linearVelocity: List[float], angularVelocity: List[float]):
        p.resetBaseVelocity(objectUniqueId, linearVelocity, angularVelocity)

    @staticmethod
    def applyExternalForce(objectUniqueId: int, forceObj: List[float], posObj: List[float], frame: Frame):

        p.applyExternalForce(objectUniqueId, -1, forceObj, posObj, frame.value)

    @staticmethod
    def disconnect():
        p.disconnect()