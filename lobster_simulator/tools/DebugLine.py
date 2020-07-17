from typing import List

import pybullet as p
import time

from lobster_simulator.tools import Translation


class DebugLine:
    """
    Class used to create a debug line in the GUI.
    """
    _MAX_UPDATE_FREQUENCY = 0.05

    def __init__(self, from_location: List[float], to_location: List[float], width: float = 5, color: List[int] = None):
        if color is None:
            color = [1, 0, 0]

        self._width = width
        self._color = color
        self._id = -1
        self._id = self._add_debug_line(from_location, to_location)
        self._latest_update_time = time.time()

    def update(self, from_location: List[float], to_location: List[float], frame_id: int = None) -> None:
        """
        Update the pose of the debug line.
        """
        if not self.can_update():
            return
        if frame_id:
            from_location = Translation.vec3_local_to_world_id(frame_id, from_location)
            to_location = Translation.vec3_local_to_world_id(frame_id, to_location)

        self._id = self._add_debug_line(from_location, to_location)
        self._latest_update_time = time.time()

    def can_update(self) -> bool:
        """
        Checks if time has passed to allow a new debug line to be created.
        """
        return time.time() - self._latest_update_time > self._MAX_UPDATE_FREQUENCY

    def _add_debug_line(self, from_location, to_location) -> int:
        return p.addUserDebugLine(lineFromXYZ=from_location,
                                  lineToXYZ=to_location,
                                  lineWidth=self._width,
                                  lineColorRGB=self._color,
                                  replaceItemUniqueId=self._id)
