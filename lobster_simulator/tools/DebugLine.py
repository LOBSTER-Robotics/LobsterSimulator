from lobster_simulator.tools import Translation
from lobster_simulator.tools.PybulletAPI import PybulletAPI


class DebugLine:

    def __init__(self, from_location, to_location, width=5, color=None):
        if color is None:
            color = [1, 0, 0]

        self.width = width
        self.color = color
        self.id = PybulletAPI.addUserDebugLine(lineFromXYZ=from_location,
                                               lineToXYZ=to_location,
                                               lineWidth=width,
                                               lineColorRGB=color)

    def update(self, from_location, to_location, frame_id=None):
        if frame_id:
            from_location = Translation.vec3_local_to_world_id(frame_id, from_location)
            to_location = Translation.vec3_local_to_world_id(frame_id, to_location)

        self.id = PybulletAPI.addUserDebugLine(lineFromXYZ=from_location,
                                               lineToXYZ=to_location,
                                               lineWidth=self.width,
                                               lineColorRGB=self.color,
                                               replaceItemUniqueId=self.id)
