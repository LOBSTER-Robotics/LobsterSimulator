# This is needed to resolve the Lobster class type, since it can't be imported due to a cyclic dependency
from __future__ import annotations

import math
from enum import Enum
from typing import List, Tuple

import pybullet as p
import pybullet_data
from pkg_resources import resource_filename

import numpy as np

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
            p.configureDebugVisualizer(p.COV_ENABLE_RGB_BUFFER_PREVIEW, 0)
            p.configureDebugVisualizer(p.COV_ENABLE_DEPTH_BUFFER_PREVIEW, 0)
            p.configureDebugVisualizer(p.COV_ENABLE_SEGMENTATION_MARK_PREVIEW, 0)

            p.configureDebugVisualizer(p.COV_ENABLE_SHADOWS, 0)

            p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)


        else:
            self._physics_client_id = p.connect(p.DIRECT)

        # p.configureDebugVisualizer(p.COV_ENABLE_Y_AXIS_UP)

        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setTimeStep(time_step.seconds)
        p.setGravity(0, 0, -GRAVITY)

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
            return p.loadURDF(fileName=file_name, basePosition=base_position.asENU(),
                              baseOrientation=base_orientation.asENU())
        else:
            return p.loadURDF(fileName=file_name, basePosition=base_position.asENU())

    @staticmethod
    def getQuaternionFromEuler(euler_angle: Vec3):
        return Quaternion.fromENU(p.getQuaternionFromEuler(euler_angle.asENU()))

    @staticmethod
    def getEulerFromQuaternion(quaternion: Quaternion) -> Quaternion:
        return Vec3.fromENU(p.getEulerFromQuaternion(quaternion.asENU()))

    @staticmethod
    def getMatrixFromQuaternion(quaternion: Quaternion):

        return p.getMatrixFromQuaternion(quaternion.array)


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
                         parentObjectUniqueId: int = -1, replaceItemUniqueId: int = -1):

        if PybulletAPI.gui():
            return p.addUserDebugLine(lineFromXYZ=lineFromXYZ.asENU(),
                                      lineToXYZ=lineToXYZ.asENU(),
                                      lineWidth=lineWidth,
                                      lineColorRGB=lineColorRGB,
                                      parentObjectUniqueId=parentObjectUniqueId,
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
                cameraTargetPosition=position.asENU()
            )

    @staticmethod
    def getBasePositionAndOrientation(objectUniqueId: int) -> Tuple[Vec3, Quaternion]:

        position, orientation = p.getBasePositionAndOrientation(objectUniqueId)

        return Vec3.fromENU(position), Quaternion.fromENU(orientation)

    @staticmethod
    def resetBasePositionAndOrientation(objectUniqueId: int, posObj: Vec3 = None, ornObj: Quaternion = None):
        if posObj is None:
            posObj = PybulletAPI.getBasePositionAndOrientation(objectUniqueId=objectUniqueId)[0]
        if ornObj is None:
            ornObj = PybulletAPI.getBasePositionAndOrientation(objectUniqueId=objectUniqueId)[1]

        p.resetBasePositionAndOrientation(objectUniqueId, posObj.asENU(), ornObj.asENU())

    @staticmethod
    def getBaseVelocity(objectUniqueId: int) -> Tuple[Vec3, Vec3]:
        """
        Gets the velocity and angular velocity of an object.
        :param objectUniqueId: Id of the object.
        :return: Tuple with velocity and angular velocity.
        """
        linearVelocity, angularVelocity = p.getBaseVelocity(objectUniqueId)
        return Vec3.fromENU(linearVelocity), Vec3.fromENU(angularVelocity)

    @staticmethod
    def resetBaseVelocity(objectUniqueId: int, linearVelocity: Vec3, angularVelocity: Vec3):
        p.resetBaseVelocity(objectUniqueId, linearVelocity.asENU(), angularVelocity.asENU())

    @staticmethod
    def applyExternalForce(objectUniqueId: int, forceObj: Vec3, posObj: Vec3, frame: Frame):
        assert isinstance(forceObj, Vec3) and isinstance(posObj, Vec3)

        p.applyExternalForce(objectUniqueId, -1, forceObj.asENU(), posObj.asENU(), frame.value)

    @staticmethod
    def createVisualSphere(radius, rgbaColor):
        sphereShape = p.createVisualShape(p.GEOM_SPHERE, radius=radius, rgbaColor=rgbaColor)
        return p.createMultiBody(0, -1, sphereShape, [0, 0, 0])

    @staticmethod
    def rayTest(rayFromPosition: Vec3, rayToPosition: Vec3) -> Tuple[float, Vec3, Vec3]:
        _, _, hit_fraction, hit_position, hit_normal = p.rayTest(rayFromPosition.asENU(), rayToPosition.asENU())[0]

        return hit_fraction, Vec3.fromENU(hit_position), Vec3.fromENU(hit_normal)

    # @staticmethod
    # def createMultiBody()

    @staticmethod
    def disconnect():
        p.disconnect()
