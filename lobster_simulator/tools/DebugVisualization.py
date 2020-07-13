from lobster_simulator.common import Vec3
from lobster_simulator.tools import Translation
from lobster_simulator.tools.PybulletAPI import PybulletAPI


class DebugLine:

    def __init__(self, from_location: Vec3, to_location: Vec3, width=5, color=None):
        if color is None:
            color = [1, 0, 0]

        self._width = width
        self._color = color
        self._id = PybulletAPI.addUserDebugLine(lineFromXYZ=from_location,
                                                lineToXYZ=to_location,
                                                lineWidth=width,
                                                lineColorRGB=color)

    def update(self, from_location: Vec3, to_location: Vec3, frame_id=None):
        if frame_id:
            from_location = Translation.vec3_local_to_world_id(frame_id, from_location)
            to_location = Translation.vec3_local_to_world_id(frame_id, to_location)

        self._id = PybulletAPI.addUserDebugLine(lineFromXYZ=from_location,
                                                lineToXYZ=to_location,
                                                lineWidth=self._width,
                                                lineColorRGB=self._color,
                                                replaceItemUniqueId=self._id)


class DebugSphere:

    def __init__(self, radius, rgba_color):
        self.sphereId = PybulletAPI.createVisualSphere(radius, rgba_color)

    def update_position(self, position: Vec3):
        PybulletAPI.resetBasePositionAndOrientation(self.sphereId, posObj=position)
