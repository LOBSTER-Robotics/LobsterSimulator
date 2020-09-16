# This is needed to resolve the Lobster class type, since it can't be imported due to a cyclic dependency
from __future__ import annotations

import math
from enum import Enum
from typing import List, Tuple, Dict, Optional

import numpy as np
import pybullet as p
import pybullet_data
from lobster_common.constants import *
from lobster_common.quaternion import Quaternion

from lobster_common.vec3 import Vec3
from lobster_simulator.common.simulation_time import SimulationTime
from lobster_simulator.common import translation


class Frame(Enum):
    LINK_FRAME = 1
    WORLD_FRAME = 2


class PybulletAPI:
    """
    This class is meant as a wrapper for the Pybullet API, but adds support for typing and using custom types.
    """

    KEY_WAS_TRIGGERED = p.KEY_WAS_TRIGGERED
    KEY_IS_DOWN = p.KEY_IS_DOWN
    DELETE_KEY = p.B3G_DELETE

    _INSTANCE: Optional[PybulletAPI] = None

    def __init__(self, time_step: SimulationTime, gui: bool = False):
        self._gui = gui

        if self._INSTANCE is not None and p.isConnected(self._INSTANCE._physics_client_id):
            # Instance could already exist when running test, just to be sure resetting it here.
            p.resetSimulation(self._INSTANCE._physics_client_id)

        self._physics_client_id = -1
        if gui:
            self._physics_client_id = p.connect(p.GUI)

            p.configureDebugVisualizer(p.COV_ENABLE_KEYBOARD_SHORTCUTS, 0)
            p.configureDebugVisualizer(p.COV_ENABLE_RGB_BUFFER_PREVIEW, 0)
            p.configureDebugVisualizer(p.COV_ENABLE_DEPTH_BUFFER_PREVIEW, 0)
            p.configureDebugVisualizer(p.COV_ENABLE_SEGMENTATION_MARK_PREVIEW, 0)
            p.configureDebugVisualizer(p.COV_ENABLE_SHADOWS, 1)
            p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)

        else:
            self._physics_client_id = p.connect(p.DIRECT)

        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setTimeStep(time_step.seconds)
        p.setGravity(0, 0, -GRAVITY)

    def is_gui_enabled(self) -> bool:
        return self._gui

    @staticmethod
    def initialize(time_step: SimulationTime, gui: bool) -> None:
        PybulletAPI._INSTANCE = PybulletAPI(time_step, gui)

    @staticmethod
    def changeDynamics(bodyUniqueId: int, linearDamping: float, angularDamping: float) -> None:

        p.changeDynamics(bodyUniqueId=bodyUniqueId,
                         linkIndex=-1,
                         linearDamping=linearDamping,
                         angularDamping=angularDamping)

    @staticmethod
    def get_pybullet_id() -> int:
        return PybulletAPI._INSTANCE._physics_client_id

    @staticmethod
    def loadURDF(file_name: str, base_position: Vec3, base_orientation: Quaternion = None) -> int:
        if base_orientation:
            return p.loadURDF(fileName=file_name, basePosition=base_position.asENU(),
                              baseOrientation=base_orientation.asENU())
        else:
            return p.loadURDF(fileName=file_name, basePosition=base_position.asENU())

    @staticmethod
    def getQuaternionFromEuler(euler_angle: Vec3) -> Quaternion:
        return Quaternion.fromENU(p.getQuaternionFromEuler(euler_angle.asENU()))

    @staticmethod
    def getEulerFromQuaternion(quaternion: Quaternion) -> Vec3:
        return Vec3.fromENU(p.getEulerFromQuaternion(quaternion.asENU()))

    @staticmethod
    def getMatrixFromQuaternion(quaternion: Quaternion) -> np.ndarray:
        # This could be wrong since the quaternion is not converted from NED to ENU
        return p.getMatrixFromQuaternion(quaternion._data)

    @staticmethod
    def gui() -> bool:
        return PybulletAPI._INSTANCE.is_gui_enabled()

    @staticmethod
    def setTimeStep(time_step: SimulationTime) -> None:
        p.setTimeStep(time_step.seconds)

    @staticmethod
    def stepSimulation() -> None:
        p.stepSimulation()

    @staticmethod
    def addUserDebugParameter(name: str, rangeMin: float, rangeMax: float, startValue: float) -> int:
        if PybulletAPI.gui():
            return p.addUserDebugParameter(name, rangeMin, rangeMax, startValue)

    @staticmethod
    def readUserDebugParameter(itemUniqueId: int) -> float:
        if PybulletAPI.gui():
            return p.readUserDebugParameter(itemUniqueId)

    @staticmethod
    def addUserDebugLine(lineFromXYZ: Vec3, lineToXYZ: Vec3, lineWidth: float, lineColorRGB: List[float],
                         parentObjectUniqueId: int = -1, replaceItemUniqueId: int = -1) -> int:

        if PybulletAPI.gui():
            return p.addUserDebugLine(lineFromXYZ=lineFromXYZ.asENU(),
                                      lineToXYZ=lineToXYZ.asENU(),
                                      lineWidth=lineWidth,
                                      lineColorRGB=lineColorRGB,
                                      parentObjectUniqueId=parentObjectUniqueId,
                                      replaceItemUniqueId=replaceItemUniqueId)

    @staticmethod
    def getKeyboardEvents() -> Dict:
        return p.getKeyboardEvents()

    @staticmethod
    def moveCameraToPosition(position: Vec3, orientation: Optional[Quaternion] = None) -> None:
        if PybulletAPI.gui():
            camera_info = p.getDebugVisualizerCamera()

            if orientation:
                orn = PybulletAPI.getEulerFromQuaternion(orientation)
                yaw = - orn[YAW] * 360 / (2 * math.pi) - 90
                pitch = orn[ROLL] * 360 / (2 * math.pi) - 20
            else:
                yaw = camera_info[8]
                pitch = camera_info[9]

            p.resetDebugVisualizerCamera(
                cameraDistance=camera_info[10],
                cameraYaw=yaw,
                cameraPitch=pitch,
                cameraTargetPosition=position.asENU()
            )

    @staticmethod
    def getBasePositionAndOrientation(objectUniqueId: int) -> Tuple[Vec3, Quaternion]:

        position, orientation = p.getBasePositionAndOrientation(objectUniqueId)

        return Vec3.fromENU(position), Quaternion.fromENU(orientation)

    @staticmethod
    def resetBasePositionAndOrientation(objectUniqueId: int, posObj: Vec3 = None, ornObj: Quaternion = None) -> None:
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
    def resetBaseVelocity(objectUniqueId: int, linearVelocity: Vec3, angularVelocity: Vec3) -> None:
        p.resetBaseVelocity(objectUniqueId, linearVelocity.asENU(), angularVelocity.asENU())

    @staticmethod
    def applyExternalForce(objectUniqueId: int, forceObj: Vec3, posObj: Vec3, frame: Frame) -> None:
        assert isinstance(forceObj, Vec3) and isinstance(posObj, Vec3)

        p.applyExternalForce(objectUniqueId, -1, forceObj.asENU(), posObj.asENU(), frame.value)

    @staticmethod
    def changeVisualShapeColor(objectUniqueId: int, color: List[float]):
        p.changeVisualShape(objectUniqueId=objectUniqueId, linkIndex=-1, rgbaColor=color)

    @staticmethod
    def createVisualSphere(radius, rgbaColor) -> int:
        sphereShape = p.createVisualShape(p.GEOM_SPHERE, radius=radius, rgbaColor=rgbaColor)
        return p.createMultiBody(0, -1, sphereShape, [0, 0, 0])

    @staticmethod
    def createVisualPlane(radius, rgbaColor) -> int:
        shape = p.createVisualShape(p.GEOM_PLANE, radius=radius, rgbaColor=rgbaColor)
        return p.createMultiBody(0, -1, shape, [0, 0, 0])

    @staticmethod
    def createVisualCylinder(radius: float, height: float, color: List[float], orientation: Quaternion):
        shape = p.createVisualShape(p.GEOM_CYLINDER, radius=radius, length=height, rgbaColor=color)
        return p.createMultiBody(0, -1, shape, basePosition=[0, 0, 0], baseOrientation=orientation.asENU())

    @staticmethod
    def loadTexture(file_name: str) -> int:
        return p.loadTexture(file_name)

    @staticmethod
    def changeVisualShape(object_id: int, textureUniqueId, rgbaColor):
        p.changeVisualShape(object_id, -1, textureUniqueId=textureUniqueId, rgbaColor=rgbaColor)

    @staticmethod
    def rayTest(rayFromPosition: Vec3, rayToPosition: Vec3, object_id=-1) -> Tuple[float, Vec3, Vec3]:
        if object_id != -1:
            rayFromPosition = translation.vec3_local_to_world_id(object_id, rayFromPosition)
            rayToPosition = translation.vec3_local_to_world_id(object_id, rayToPosition)

        _, _, hit_fraction, hit_position, hit_normal = p.rayTest(rayFromPosition.asENU(), rayToPosition.asENU())[0]

        return hit_fraction, Vec3.fromENU(hit_position), Vec3.fromENU(hit_normal)

    @staticmethod
    def removeBody(objectUniqueId: int):
        p.removeBody(objectUniqueId)

    @staticmethod
    def removeUserDebugItem(itemUniqueId: int):
        p.removeUserDebugItem(itemUniqueId=itemUniqueId)

    @staticmethod
    def applyExternalTorque(objectUniqueId: int, torqueObj: Vec3, frame: Frame):
        assert isinstance(torqueObj, Vec3)

        # There is a bug in Pybullet that the Link Frame and World frame are swapped when applying a torque to the
        # base link of a robot (https://github.com/bulletphysics/bullet3/issues/1949)
        if frame == Frame.WORLD_FRAME:
            frame = Frame.LINK_FRAME
        else:
            frame = Frame.WORLD_FRAME

        p.applyExternalTorque(objectUniqueId, -1, torqueObj.asENU(), flags=frame.value)

    @staticmethod
    def disconnect():
        p.disconnect()
