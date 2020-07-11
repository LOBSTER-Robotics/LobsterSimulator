# This is needed to resolve the Lobster class type, since it can't be imported due to a cyclic dependency
from __future__ import annotations

import math
from enum import Enum
from typing import List, Tuple, TYPE_CHECKING


import pybullet as p
import pybullet_data

from lobster_simulator.common.Quaternion import Quaternion
from lobster_simulator.common.Vec3 import Vec3

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
        self._gui = gui

        self._physics_client_id = -1
        if gui:
            self._physics_client_id = p.connect(p.GUI)
        else:
            self._physics_client_id = p.connect(p.DIRECT)

        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setTimeStep(time_step.seconds)
        p.setGravity(0, 0, -GRAVITY)
        self.loadURDF("plane.urdf", Vec3([0, 0, -100]))
        self.loadURDF("plane.urdf", Vec3([0, 0, 0]), self.getQuaternionFromEuler([math.pi, 0, 0]))

    def is_gui_enabled(self):
        return self._gui

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
        return PybulletAPI.__instance._physics_client_id

    @staticmethod
    def loadURDF(file_name: str, base_position: Vec3, base_orientation: Quaternion = None):
        if base_orientation:
            return p.loadURDF(fileName=file_name, basePosition=base_position.array, baseOrientation=base_orientation.data)
        else:
            return p.loadURDF(fileName=file_name, basePosition=base_position.array)

    @staticmethod
    def getQuaternionFromEuler(euler_angle: List[float]):
        return Quaternion(p.getQuaternionFromEuler(euler_angle))

    @staticmethod
    def getEulerFromQuaternion(quaternion: List[float]):
        return p.getEulerFromQuaternion(quaternion)

    @staticmethod
    def getMatrixFromQuaternion(quaternion: Quaternion):

        return p.getMatrixFromQuaternion(quaternion.data)

    @staticmethod
    def gui():
        return PybulletAPI.__instance.is_gui_enabled()

    @staticmethod
    def setTimeStep(time_step: SimulationTime):
        p.setTimeStep(time_step.seconds)

    @staticmethod
    def stepSimulation():
        p.stepSimulation()

    @staticmethod
    def addUserDebugParameter(name, rangeMin, rangeMax, startValue):
        if PybulletAPI.gui():
            return p.addUserDebugParameter(name, rangeMin, rangeMax, startValue)

    @staticmethod
    def readUserDebugParameter(itemUniqueId: int) -> float:
        if PybulletAPI.gui():
            return p.readUserDebugParameter(itemUniqueId)

    @staticmethod
    def addUserDebugLine(lineFromXYZ: Vec3, lineToXYZ: Vec3, lineWidth: float, lineColorRGB: List[float],
                         replaceItemUniqueId: int = -1):

        if PybulletAPI.gui():
            return p.addUserDebugLine(lineFromXYZ=lineFromXYZ.array,
                                      lineToXYZ=lineToXYZ.array,
                                      lineWidth=lineWidth,
                                      lineColorRGB=lineColorRGB,
                                      replaceItemUniqueId=replaceItemUniqueId)

    @staticmethod
    def getKeyboardEvents():
        return p.getKeyboardEvents()

    @staticmethod
    def moveCameraToPosition(position: Vec3):
        if PybulletAPI.gui():
            camera_info = p.getDebugVisualizerCamera()
            p.resetDebugVisualizerCamera(
                cameraDistance=camera_info[10],
                cameraYaw=camera_info[8],
                cameraPitch=camera_info[9],
                cameraTargetPosition=position.array
            )

    @staticmethod
    def getBasePositionAndOrientation(objectUniqueId: int) -> Tuple[Vec3, Quaternion]:

        position, orientation = p.getBasePositionAndOrientation(objectUniqueId)

        return Vec3(position), Quaternion(orientation)

    @staticmethod
    def resetBasePositionAndOrientation(objectUniqueId: int, posObj: List[float], ornObj: List[float]):
        p.resetBasePositionAndOrientation(objectUniqueId, posObj, ornObj)

    @staticmethod
    def getBaseVelocity(objectUniqueId: int) -> Tuple[Vec3, Vec3]:
        linearVelocity, angularVelocity = p.getBaseVelocity(objectUniqueId)
        return Vec3(linearVelocity), Vec3(angularVelocity)

    @staticmethod
    def resetBaseVelocity(objectUniqueId: int, linearVelocity: List[float], angularVelocity: List[float]):
        p.resetBaseVelocity(objectUniqueId, linearVelocity, angularVelocity)

    @staticmethod
    def applyExternalForce(objectUniqueId: int, forceObj: Vec3, posObj: Vec3, frame: Frame):
        assert isinstance(forceObj, Vec3) and isinstance(posObj, Vec3)

        p.applyExternalForce(objectUniqueId, -1, forceObj.array, posObj.array, frame.value)

    @staticmethod
    def disconnect():
        p.disconnect()
