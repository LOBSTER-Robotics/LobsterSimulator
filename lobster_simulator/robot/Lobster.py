import pybullet as p
import numpy as np
import math
from lobster_simulator.robot.Link import Link


def rpm_to_thrust(rpm):
    return rpm


class Lobster:

    def __init__(self, config):

        self.max_rpm_change_per_second = config['max_rpm_change_per_second']
        self.center_of_volume = config['center_of_volume']

        # body_id = p.createCollisionShape(p.GEOM_CYLINDER, radius=diameter, height=length)
        # head_id = p.createCollisionShape(p.GEOM_SPHERE, radius=diameter)
        # arm_id = p.createCollisionShape(p.GEOM_CYLINDER, radius=0.05, height=arm_length * 2)
        # motor_id = p.createCollisionShape(p.GEOM_CYLINDER, radius=0.1, height=0.15)

        front_facing_motor_x = config['front_facing_motor_x']
        front_facing_motor_y = config['front_facing_motor_y']
        side_facing_motor_x = config['side_facing_motor_x']
        side_facing_motor_y = config['side_facing_motor_y']

        self.motorPositions = [
            [front_facing_motor_x, 0,  front_facing_motor_y],
            [front_facing_motor_x, 0, -front_facing_motor_y],
            [front_facing_motor_x,  front_facing_motor_y, 0],
            [front_facing_motor_x, -front_facing_motor_y, 0],
            [side_facing_motor_x, 0,  side_facing_motor_y],
            [side_facing_motor_x, 0, -side_facing_motor_y],
            [side_facing_motor_x,  side_facing_motor_y, 0],
            [side_facing_motor_x, -side_facing_motor_y, 0]
        ]
        self.id = p.loadURDF("lobster_simulator\\Model_URDF.SLDASM.urdf",
                             [0, 0, 2],
                             p.getQuaternionFromEuler([math.pi / 2, 0, 0]))
        # links = [
        #     Link(collision_shape=head_id,
        #          position=[0, 0, length / 2]),  # Head Link
        #
        #     Link(collision_shape=arm_id,
        #          position=[0, 0, arm_position_from_center],
        #          orientation=p.getQuaternionFromEuler([math.pi / 2, 0, 0])),  # Arm Link 1
        #
        #     Link(collision_shape=arm_id, position=[0, 0, arm_position_from_center],
        #          orientation=p.getQuaternionFromEuler([0, math.pi / 2, 0])),  # Arm Link 2
        # ]
        #
        # for i in range(4):
        #     links.append(Link(collision_shape=motor_id, position=self.motorPositions[i]))  # Forward Motor Links
        #
        # for i in range(4, 6):
        #     links.append(Link(collision_shape=motor_id, position=self.motorPositions[i],
        #                       orientation=p.getQuaternionFromEuler([math.pi / 2, 0, 0])))  # Upwards Motor Links
        #
        # for i in range(6, 8):
        #     links.append(Link(collision_shape=motor_id, position=self.motorPositions[i],
        #                       orientation=p.getQuaternionFromEuler([0, math.pi / 2, 0])))  # Sidewards Motor Links
        #
        # self.id = p.createMultiBody(
        #     baseMass=10,
        #     baseOrientation=p.getQuaternionFromEuler([math.pi / 2, 0, 0]),
        #     baseCollisionShapeIndex=body_id,
        #     basePosition=[2, 2, 2],
        #     baseInertialFramePosition=[0, 0, center_of_mass],
        #     linkMasses=[link.mass for link in links],
        #     linkVisualShapeIndices=[link.visualShape for link in links],
        #     linkPositions=[link.position for link in links],
        #     linkCollisionShapeIndices=[link.collisionShape for link in links],
        #     linkOrientations=[link.orientation for link in links],
        #     linkInertialFramePositions=[link.inertial_frame_position for link in links],
        #     linkInertialFrameOrientations=[link.inertial_frame_orientation for link in links],
        #     linkParentIndices=[link.parent_index for link in links],
        #     linkJointTypes=[link.joint_type for link in links],
        #     linkJointAxis=[link.joint_axis for link in links])

        p.changeDynamics(self.id, -1, linearDamping=0.9, angularDamping=0.9)

        self.rpm_motors = list()
        self.desired_thrust_motors = list()
        for i in range(8):
            self.rpm_motors.append(0)
            self.desired_thrust_motors.append(0)

        self.buoyancySphereShape = p.createVisualShape(p.GEOM_SPHERE, radius=0.2, rgbaColor=[1, 0, 0, 1])
        self.buoyancyPointIndicator = p.createMultiBody(0, -1, self.buoyancySphereShape, [0, 0, 0],
                                                        useMaximalCoordinates=0)

        self.buoyancy = 100
        self.max_thrust = 100

    def set_buoyancy(self, value):
        self.buoyancy = value

    def set_desired_rpm_motors(self, desired_rpm_motors):
        for i in range(len(desired_rpm_motors)):
            rpm = desired_rpm_motors[i]

        self.desired_thrust_motors = desired_rpm_motors

    def set_desired_thrust_motors(self, desired_thrusts):
        self.desired_thrust_motors = desired_thrusts

    def update_motors(self, dt):
        for i in range(8):
            diff = self.desired_thrust_motors[i] - self.rpm_motors[i]
            sign = int(diff > 0) - int(diff < 0)
            if math.fabs(diff) <= self.max_rpm_change_per_second * dt:
                self.rpm_motors[i] = self.desired_thrust_motors[i]
            else:
                self.rpm_motors[i] += sign * self.max_rpm_change_per_second * dt
                print(self.max_rpm_change_per_second, dt)

    def update(self, dt):
        lobster_pos, lobster_orn = self.get_position_and_orientation()

        self.update_motors(dt)

        # Apply forces for forward facing motors
        for i in range(4):
            p.applyExternalForce(objectUniqueId=self.id, linkIndex=-1,
                                 forceObj=[rpm_to_thrust(self.rpm_motors[i]), 0, 0], posObj=self.motorPositions[i],
                                 flags=p.LINK_FRAME)

        # Apply forces for upward facing motors
        for i in range(4, 6):
            p.applyExternalForce(objectUniqueId=self.id, linkIndex=-1,
                                 forceObj=[0, rpm_to_thrust(self.rpm_motors[i]), 0], posObj=self.motorPositions[i],
                                 flags=p.LINK_FRAME)

        # Determine the point where the buoyancy force acts on the robot
        buoyancy_force_pos = np.reshape(np.array(p.getMatrixFromQuaternion(lobster_orn)), (3, 3)).dot(
            np.array(self.center_of_volume)) \
                             + lobster_pos

        # Apply the buoyancy force
        p.applyExternalForce(objectUniqueId=self.id, linkIndex=-1,
                             forceObj=[0, 0, self.buoyancy], posObj=np.array(buoyancy_force_pos), flags=p.WORLD_FRAME)

    def get_position_and_orientation(self):
        return p.getBasePositionAndOrientation(self.id)

    def get_position(self):
        position, _ = self.get_position_and_orientation()
        return position

    def get_orientation(self):
        _, orientation = self.get_position_and_orientation()
        return orientation
