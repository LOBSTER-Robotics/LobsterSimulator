from abc import ABC

from lobster_common.quaternion import Quaternion
from lobster_common.vec3 import Vec3

import time

from pkg_resources import resource_filename

from lobster_simulator.common.pybullet_api import PybulletAPI


class DebugObject(ABC):

    def __init__(self, object_id):
        self._object_id = object_id

    def remove(self) -> None:
        """
        Removes current object from GUI
        """
        PybulletAPI.removeUserDebugItem(self._object_id)
        self._object_id = None

    @property
    def object_id(self):
        if self._object_id is None:
            print("Trying to get object id but object has been removed or not instantiated")
        return self._object_id


class DebugLine(DebugObject):
    """
    Class used to create a debug line in the GUI.
    """
    _MIN_UPDATE_INTERVAL = 0.03

    def __init__(self, from_location: Vec3 = None, to_location: Vec3 = None, width=5, color=None, parentIndex=-1):
        if color is None:
            color = [1, 1, 1]

        self.parentIndex = parentIndex

        if from_location is None:
            from_location = Vec3([0, 0, 0])

        if to_location is None:
            to_location = Vec3([0, 0, 0])

        self.from_location = from_location
        self.to_location = to_location

        self._width = width
        self._color = color

        self._latest_update_time = 0

        object_id = self._update_debug_line()
        super().__init__(object_id)

    def update(self, from_location: Vec3 = None, to_location: Vec3 = None, frame_id: int = None, color=None) -> None:
        """
        Update the pose of the debug line.
        """
        if from_location:
            self.from_location = from_location
        if to_location:
            self.to_location = to_location

        if not self.can_update():
            return
        if frame_id:
            self.parentIndex = frame_id

        if color:
            self._color = color

        self._object_id = self._update_debug_line()
        self._latest_update_time = time.time()

    def can_update(self) -> bool:
        """
        Checks if time has passed to allow a new debug line to be created.
        """
        return time.time() - self._latest_update_time > self._MIN_UPDATE_INTERVAL

    def _update_debug_line(self) -> int:
        return PybulletAPI.addUserDebugLine(lineFromXYZ=self.from_location,
                                            lineToXYZ=self.to_location,
                                            lineWidth=self._width,
                                            lineColorRGB=self._color,
                                            parentObjectUniqueId=self.parentIndex,
                                            replaceItemUniqueId=self._object_id)


class DebugSphere(DebugObject):

    def __init__(self, radius, rgba_color):
        object_id = PybulletAPI.createVisualSphere(radius, rgba_color)
        super().__init__(object_id)

    def update_position(self, position: Vec3):
        PybulletAPI.resetBasePositionAndOrientation(self._object_id, posObj=position)


class DebugScout(DebugObject):

    def __init__(self, pos: Vec3 = None):
        if not pos:
            pos = Vec3([0, 0, 0])

        object_id = PybulletAPI.loadURDF(resource_filename("lobster_simulator",
                                                           "data/scout-alpha-visual.urdf"), pos)
        super().__init__(object_id)

    def set_position_and_orientation(self, position: Vec3, orientation: Quaternion):
        PybulletAPI.resetBasePositionAndOrientation(self._object_id, posObj=position, ornObj=orientation)
