import pybullet as p
import numpy as np
import math
from lobster_simulator.robot.Link import Link
from lobster_simulator.tools import Translation
from lobster_simulator.tools.DebugLine import DebugLine


def rpm_to_thrust(rpm):
    return rpm


class Lobster:

    def __init__(self, config):

        self.max_rpm_change_per_second = config['max_rpm_change_per_second']
        self.center_of_volume = config['center_of_volume']

        front_facing_motor_x = config['front_facing_motor_x']
        front_facing_motor_y = config['front_facing_motor_y']
        side_facing_motor_x = config['side_facing_motor_x']
        side_facing_motor_y = config['side_facing_motor_y']

        self.motorPositions = [
            np.array([front_facing_motor_x, 0,  front_facing_motor_y]),
            np.array([front_facing_motor_x, 0, -front_facing_motor_y]),
            np.array([front_facing_motor_x,  front_facing_motor_y, 0]),
            np.array([front_facing_motor_x, -front_facing_motor_y, 0]),
            np.array([side_facing_motor_x, 0,  side_facing_motor_y]),
            np.array([side_facing_motor_x, 0, -side_facing_motor_y]),
            np.array([side_facing_motor_x,  side_facing_motor_y, 0]),
            np.array([side_facing_motor_x, -side_facing_motor_y, 0])
        ]

        self.motor_directions = [
            np.array([1, 0, 0]),
            np.array([1, 0, 0]),
            np.array([1, 0, 0]),
            np.array([1, 0, 0]),
            np.array([0, 1, 0]),
            np.array([0, 1, 0]),
            np.array([0, 0, 1]),
            np.array([0, 0, 1]),
        ]

        self.id = p.loadURDF("lobster_simulator\\Model_URDF.SLDASM.urdf",
                             [0, 0, 2],
                             p.getQuaternionFromEuler([math.pi / 2, 0, 0]))

        p.changeDynamics(self.id, -1, linearDamping=0.9, angularDamping=0.9)

        self.motor_debug_lines = list()

        self.rpm_motors = list()
        self.desired_thrust_motors = list()
        for i in range(8):
            self.rpm_motors.append(0)
            self.desired_thrust_motors.append(0)
            self.motor_debug_lines.append(DebugLine(self.motorPositions[i], self.motorPositions[i]))

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

    def update(self, dt):
        lobster_pos, lobster_orn = self.get_position_and_orientation()

        self.update_motors(dt)

        # Apply forces for forward facing motors
        for i in range(8):
            p.applyExternalForce(objectUniqueId=self.id, linkIndex=-1,
                                 forceObj=self.motor_directions[i] * rpm_to_thrust(self.rpm_motors[i]),
                                 posObj=self.motorPositions[i],
                                 flags=p.LINK_FRAME)
            self.motor_debug_lines[i].update(self.motorPositions[i],
                                             self.motorPositions[i] + self.motor_directions[i] * self.rpm_motors[i] / 1000,
                                             self.id)

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
