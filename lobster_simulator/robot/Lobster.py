import pybullet as p
import numpy as np
import math

from pkg_resources import resource_filename

from lobster_simulator.robot.Link import Link
from lobster_simulator.tools import Translation
from lobster_simulator.tools.DebugLine import DebugLine


def rpm_to_thrust(rpm):
    return 3.92/1000 * rpm + 3.9/10000000 * rpm * rpm + 7.55/10000000000 * rpm * rpm * rpm


def thrust_to_rpm(x):
    return -172.185 - 5.05393 * pow(261.54 * math.sqrt(3.84767*math.pow(10, 8) * math.pow(x, 2) + 5.13478*math.pow(10, 8) * x + 4.48941 * math.pow(10, 9)) - 5.13023*math.pow(10, 6) * x - 3.42319*math.pow(10, 6), (1 / 3)) + 336577. / math.pow(261.54 * math.sqrt(3.84767*pow(10, 8) * math.pow(x, 2) + 5.13478*math.pow(10, 8) * x + 4.48941*math.pow(10, 9)) - 5.13023*math.pow(10, 6) * x - 3.42319*math.pow(10, 6), (1 / 3))
    # return 516*math.pow(x, 0.489)


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

        self.id = p.loadURDF(resource_filename("lobster_simulator", "data/Model_URDF.SLDASM.urdf"),
                             [0, 0, 2],
                             p.getQuaternionFromEuler([math.pi / 2, 0, 0]))

        p.changeDynamics(self.id, -1, linearDamping=0.9, angularDamping=0.9)

        self.motor_debug_lines = list()

        self.rpm_motors = list()
        self.desired_rpm_motors = list()
        for i in range(8):
            self.rpm_motors.append(0)
            self.desired_rpm_motors.append(0)
            self.motor_debug_lines.append(DebugLine(self.motorPositions[i], self.motorPositions[i]))

        self.buoyancySphereShape = p.createVisualShape(p.GEOM_SPHERE, radius=0.2, rgbaColor=[1, 0, 0, 1])
        self.buoyancyPointIndicator = p.createMultiBody(0, -1, self.buoyancySphereShape, [0, 0, 0],
                                                        useMaximalCoordinates=0)

        self.buoyancy = 100
        self.max_thrust = 100

    def set_buoyancy(self, value):
        self.buoyancy = value

    def set_desired_rpm_motors(self, desired_rpm_motors):
        self.desired_rpm_motors = desired_rpm_motors

    def set_desired_rpm_motor(self, index, desired_rpm):
        self.desired_rpm_motors[index] = desired_rpm
        print(desired_rpm)

    def set_desired_thrust_motors(self, desired_thrusts):
        for i in range(len(desired_thrusts)):
            self.desired_rpm_motors[i] = rpm_to_thrust(desired_thrusts[i])

    def set_desired_thrust_motor(self, index, desired_thrust):
        self.desired_rpm_motors[index] = thrust_to_rpm(desired_thrust)

    def update_motors(self, dt):
        for i in range(8):
            diff = self.desired_rpm_motors[i] - self.rpm_motors[i]
            sign = int(diff > 0) - int(diff < 0)
            if math.fabs(diff) <= self.max_rpm_change_per_second * dt:
                self.rpm_motors[i] = self.desired_rpm_motors[i]
            else:
                self.rpm_motors[i] += sign * self.max_rpm_change_per_second * dt

    def update(self, dt):
        lobster_pos, lobster_orn = self.get_position_and_orientation()

        self.update_motors(dt)

        # Apply forces for the  facing motors
        for i in range(8):
            p.applyExternalForce(objectUniqueId=self.id, linkIndex=-1,
                                 forceObj=self.motor_directions[i] * rpm_to_thrust(self.rpm_motors[i]),
                                 posObj=self.motorPositions[i],
                                 flags=p.LINK_FRAME)
            self.motor_debug_lines[i].update(self.motorPositions[i],
                                             self.motorPositions[i] + self.motor_directions[i] * rpm_to_thrust(self.rpm_motors[i]) / 100,
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
