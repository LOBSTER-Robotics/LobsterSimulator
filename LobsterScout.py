import pybullet as p
import numpy as np
import math
from Link import Link


class LobsterScout:

    thrustSliders = list()
    thrusts = list()

    def __init__(self, length, diameter, arm_length, arm_position_from_center, center_of_mass=0):
        self.center_of_mass = center_of_mass

        body_id = p.createCollisionShape(p.GEOM_CYLINDER, radius=diameter, height=length)
        head_id = p.createCollisionShape(p.GEOM_SPHERE, radius=diameter)
        arm_id = p.createCollisionShape(p.GEOM_CYLINDER, radius=0.05, height=arm_length*2)
        motor_id = p.createCollisionShape(p.GEOM_CYLINDER, radius=0.1, height=0.15)

        self.motorPositions = [
            [arm_length, 0, arm_position_from_center],
            [- arm_length, 0, arm_position_from_center],
            [0, arm_length, arm_position_from_center],
            [0, -arm_length, arm_position_from_center],
            [0.3, 0, arm_position_from_center],
            [-0.3, 0, arm_position_from_center]
        ]

        links = [
            Link(collision_shape=head_id,
                 position=[0, 0, 1]),  # Head Link

            Link(collision_shape=arm_id,
                 position=[0, 0, arm_position_from_center],
                 orientation=p.getQuaternionFromEuler([math.pi / 2, 0, 0])),  # Arm Link 1

            Link(collision_shape=arm_id, position=[0, 0, arm_position_from_center],
                 orientation=p.getQuaternionFromEuler([0, math.pi / 2, 0])),  # Arm Link 2
        ]

        for i in range(4):
            links.append(Link(collision_shape=motor_id, position=self.motorPositions[i]))  # Motor Link

        for i in range(4, 6):
            links.append(Link(collision_shape=motor_id, position=self.motorPositions[i],
                              orientation=p.getQuaternionFromEuler([math.pi / 2, 0, 0])))  # Motor Link

        self.id = p.createMultiBody(
            baseMass                        = 10,
            baseCollisionShapeIndex         = body_id,
            basePosition                    = [2, 2, 2],
            baseInertialFramePosition       = [0, 0, center_of_mass],
            linkMasses                      = [link.mass for link in links],
            linkVisualShapeIndices          = [link.visualShape for link in links],
            linkPositions                   = [link.position for link in links],
            linkCollisionShapeIndices       = [link.collisionShape for link in links],
            linkOrientations                = [link.orientation for link in links],
            linkInertialFramePositions      = [link.inertial_frame_position for link in links],
            linkInertialFrameOrientations   = [link.inertial_frame_orientation for link in links],
            linkParentIndices               = [link.parent_index for link in links],
            linkJointTypes                  = [link.joint_type for link in links],
            linkJointAxis                   = [link.joint_axis for link in links])

        p.changeDynamics(self.id, -1, linearDamping=0.9, angularDamping=0.9)
        for i in range(6):
            self.thrusts.append(0)

        self.buoyancyForceSlider = p.addUserDebugParameter("buoyancyForce", 0, 1000, 90)
        self.totalThrustSlider = p.addUserDebugParameter("total thrust", 0, 1000, 0)

        self.buoyancySphereShape = p.createVisualShape(p.GEOM_SPHERE, radius=0.05, rgbaColor=[1, 0, 0, 0.4])
        self.buoyancyPointIndicator = p.createMultiBody(0, -1, self.buoyancySphereShape, [0, 0, 0],
                                                        useMaximalCoordinates=0)

    def update_motors(self, thrust_values):
        self.thrusts = thrust_values

    def update(self):
        lobster_pos, lobster_orn = self.get_position_and_orientation()

        buoyancy     = p.readUserDebugParameter(self.buoyancyForceSlider)
        total_thrust = p.readUserDebugParameter(self.totalThrustSlider)

        # Apply forces for forward facing motors
        for i in range(4):
            p.applyExternalForce(objectUniqueId=self.id, linkIndex=-1,
                                 forceObj=[0, 0, self.thrusts[i] * total_thrust], posObj=self.motorPositions[i],
                                 flags=p.LINK_FRAME)

        # Apply forces for upward facing motors
        for i in range(4, 6):
            p.applyExternalForce(objectUniqueId=self.id, linkIndex=-1,
                                 forceObj=[0, self.thrusts[i] * 1000, 0], posObj=self.motorPositions[i],
                                 flags=p.LINK_FRAME)

        # Determine the point where the buoyance force acts on the robot
        buoyancy_force_pos = np.reshape(np.array(p.getMatrixFromQuaternion(lobster_orn)), (3, 3)).dot(
            np.array([0, 0, -self.center_of_mass])) \
            + lobster_pos

        # Move the sphere that points to the position of the buoyance force
        p.resetBasePositionAndOrientation(self.buoyancyPointIndicator, np.array(buoyancy_force_pos), lobster_orn)

        # Apply the bouyance force
        p.applyExternalForce(objectUniqueId=self.id, linkIndex=-1,
                             forceObj=[0, 0, buoyancy], posObj=np.array(buoyancy_force_pos), flags=p.WORLD_FRAME)

    def get_position_and_orientation(self):
        return p.getBasePositionAndOrientation(self.id)

    def get_position(self):
        position, _ = self.get_position_and_orientation()
        return position