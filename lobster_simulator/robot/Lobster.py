import pybullet as p
import numpy as np
import math

from typing import List

from pkg_resources import resource_filename

from lobster_simulator.robot.Motor import Motor
from lobster_simulator.robot.T200 import T200
from lobster_simulator.sensors.DepthSensor import DepthSensor
from lobster_simulator.tools.DebugLine import DebugLine


class Lobster:

    def __init__(self, config):

        max_rpm_change_per_second = config['max_rpm_change_per_second']
        self.center_of_volume = config['center_of_volume']

        front_facing_motor_x = config['front_facing_motor_x']
        front_facing_motor_y = config['front_facing_motor_y']
        side_facing_motor_x = config['side_facing_motor_x']
        side_facing_motor_y = config['side_facing_motor_y']

        motor_positions = [
            np.array([front_facing_motor_x, 0,  front_facing_motor_y]),
            np.array([front_facing_motor_x, 0, -front_facing_motor_y]),
            np.array([front_facing_motor_x,  front_facing_motor_y, 0]),
            np.array([front_facing_motor_x, -front_facing_motor_y, 0]),
            np.array([side_facing_motor_x, 0,  side_facing_motor_y]),
            np.array([side_facing_motor_x, 0, -side_facing_motor_y]),
            np.array([side_facing_motor_x,  side_facing_motor_y, 0]),
            np.array([side_facing_motor_x, -side_facing_motor_y, 0])
        ]

        motor_directions = [
            np.array([1, 0, 0]),
            np.array([1, 0, 0]),
            np.array([1, 0, 0]),
            np.array([1, 0, 0]),
            np.array([0, 1, 0]),
            np.array([0, 1, 0]),
            np.array([0, 0, 1]),
            np.array([0, 0, 1]),
        ]

        self.motors: List[Motor] = list()

        self.id = p.loadURDF(resource_filename("lobster_simulator", "data/Model_URDF.SLDASM.urdf"),
                             [0, 0, 0],
                             p.getQuaternionFromEuler([math.pi / 2, 0, 0]))

        for i in range(len(motor_positions)):
            self.motors.append(T200(self.id, '', motor_positions[i], motor_directions[i], max_rpm_change_per_second))

        p.changeDynamics(self.id, -1, linearDamping=0.9, angularDamping=0.9)

        self.motor_debug_lines = list()

        self.rpm_motors = list()
        self.desired_rpm_motors = list()
        for i in range(8):
            self.rpm_motors.append(0)
            self.desired_rpm_motors.append(0)
            self.motor_debug_lines.append(DebugLine(motor_positions[i], motor_positions[i]))

        self.buoyancySphereShape = p.createVisualShape(p.GEOM_SPHERE, radius=0.2, rgbaColor=[1, 0, 0, 1])
        self.buoyancyPointIndicator = p.createMultiBody(0, -1, self.buoyancySphereShape, [0, 0, 0],
                                                        useMaximalCoordinates=0)

        self.depth_sensor = DepthSensor(self.id, [1, 0, 0], None, 0)

        self.buoyancy = 100

    def set_buoyancy(self, value):
        self.buoyancy = value

    def set_desired_rpm_motors(self, desired_rpm_motors):
        for i in range(len(desired_rpm_motors)):
            self.motors[i].set_desired_rpm(desired_rpm_motors[i])

    def set_desired_rpm_motor(self, index, desired_rpm):
        self.desired_rpm_motors[index] = desired_rpm
        print(desired_rpm)

    def set_desired_thrust_motors(self, desired_thrusts):
        for i in range(len(desired_thrusts)):
            self.motors[i].set_desired_thrust(desired_thrusts[i])

    def set_desired_thrust_motor(self, index: int, desired_thrust: float):
        self.motors[index].set_desired_thrust(desired_thrust)

    def update(self, dt):
        lobster_pos, lobster_orn = self.get_position_and_orientation()
        print(self.depth_sensor.get_depth())

        for i in range(8):
            self.motors[i].update(dt)

        # Apply forces for the  facing motors
        for i in range(8):
            self.motors[i].apply_thrust()
            self.motor_debug_lines[i].update(self.motors[i].position,
                                             self.motors[i].position + self.motors[i].direction * self.motors[i].get_thrust() / 100,
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
