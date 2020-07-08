
class Link:

    def __init__(self,
                 mass=0,
                 collision_shape=-1,
                 visual_shape=-1,
                 position=None,
                 orientation=None,
                 inertial_frame_position=None,
                 inertial_frame_orientation=None,
                 parent_index=0,
                 joint_type=0,
                 joint_axis=None):

        if joint_axis is None:
            joint_axis = [0, 0, 1]
        if inertial_frame_orientation is None:
            inertial_frame_orientation = [0, 0, 0, 1]
        if inertial_frame_position is None:
            inertial_frame_position = [0, 0, 0]
        if orientation is None:
            orientation = [0, 0, 0, 1]
        if position is None:
            position = [0, 0, 0]

        self._mass = mass
        self._joint_axis = joint_axis
        self._joint_type = joint_type
        self._parent_index = parent_index
        self._inertial_frame_orientation = inertial_frame_orientation
        self._inertial_frame_position = inertial_frame_position
        self._orientation = orientation
        self._visualShape = visual_shape
        self._collisionShape = collision_shape
        self._position = position
