from lobster_simulator.common.Vec3 import Vec3
from typing import List

import time

from lobster_simulator.tools import Translation
from lobster_simulator.tools.PybulletAPI import PybulletAPI


class DebugLine:
    """
    Class used to create a debug line in the GUI.
    """
    _MAX_UPDATE_FREQUENCY = 0.03

    def __init__(self, from_location: Vec3 = None, to_location: Vec3 = None, width=5, color=None):
        if color is None:
            color = [1, 0, 0]

        if from_location is None:
            from_location = Vec3([0, 0, 0])

        if to_location is None:
            to_location = Vec3([0, 0, 0])

        self._width = width
        self._color = color

        self._id = -1
        self._id = self._add_debug_line(from_location, to_location)
        self._latest_update_time = time.time()

    def update(self, from_location: Vec3, to_location: Vec3, frame_id: int = None, color=None) -> None:
        """
        Update the pose of the debug line.
        """
        if not self.can_update():
            return
        if frame_id:
            from_location = Translation.vec3_local_to_world_id(frame_id, from_location)
            to_location = Translation.vec3_local_to_world_id(frame_id, to_location)

        if color:
            self._color = color

        self._id = self._add_debug_line(from_location, to_location)
        self._latest_update_time = time.time()

    def can_update(self) -> bool:
        """
        Checks if time has passed to allow a new debug line to be created.
        """
        return time.time() - self._latest_update_time > self._MAX_UPDATE_FREQUENCY

    def _add_debug_line(self, from_location, to_location) -> int:
        return PybulletAPI.addUserDebugLine(lineFromXYZ=from_location,
                                            lineToXYZ=to_location,
                                            lineWidth=self._width,
                                            lineColorRGB=self._color,
                                            replaceItemUniqueId=self._id)


class DebugSphere:

    def __init__(self, radius, rgba_color):
        self.sphereId = PybulletAPI.createVisualSphere(radius, rgba_color)

    def update_position(self, position: Vec3):
        PybulletAPI.resetBasePositionAndOrientation(self.sphereId, posObj=position)
