import pybullet as p
import time

from lobster_simulator.tools import Translation


class DebugLine:
    MAX_UPDATE_FREQUENCY = 0.05

    def __init__(self, from_location, to_location, width=5, color=None):
        if color is None:
            color = [1, 0, 0]

        self.width = width
        self.color = color
        self.id = p.addUserDebugLine(lineFromXYZ=from_location,
                                     lineToXYZ=to_location,
                                     lineWidth=width,
                                     lineColorRGB=color)
        self._latest_update_time = time.time()

    def update(self, from_location, to_location, frame_id=None) -> None:
        if not self.can_update():
            return
        if frame_id:
            from_location = Translation.vec3_local_to_world_id(frame_id, from_location)
            to_location = Translation.vec3_local_to_world_id(frame_id, to_location)

        self.id = p.addUserDebugLine(lineFromXYZ=from_location,
                                     lineToXYZ=to_location,
                                     lineWidth=self.width,
                                     lineColorRGB=self.color,
                                     replaceItemUniqueId=self.id)
        self._latest_update_time = time.time()

    def can_update(self) -> bool:
        return time.time() - self._latest_update_time > MAX_UPDATE_FREQUENCY
