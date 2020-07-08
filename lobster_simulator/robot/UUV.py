import numpy as np

from typing import List

from pkg_resources import resource_filename

from lobster_simulator.tools.PybulletAPI import PybulletAPI, Frame
from lobster_simulator.common.general_exceptions import ArgumentNoneError
from lobster_simulator.robot.Motor import Motor
from lobster_simulator.sensors.Accelerometer import Accelerometer
from lobster_simulator.sensors.DepthSensor import DepthSensor
from lobster_simulator.sensors.Gyroscope import Gyroscope
from lobster_simulator.sensors.Magnetometer import Magnetometer
from lobster_simulator.simulation_time import SimulationTime
from lobster_simulator.tools.DebugLine import DebugLine


class UUV:

    def __init__(self, config):
        if config is None:
            raise ArgumentNoneError("config parameter should not be None")

        self.center_of_volume = config['center_of_volume']

        self.id = PybulletAPI.loadURDF(resource_filename("lobster_simulator", "data/Model_URDF.SLDASM.urdf"),
                                       [0, 0, -1],
                                       PybulletAPI.getQuaternionFromEuler([0, 0, 0]))

        config_motors = config['motors']

        self.motors: List[Motor] = list()
        for i in range(len(config_motors)):
            self.motors.append(Motor.new_T200(self.id, config_motors[i]['name'],
                                              np.array(config_motors[i]['position']),
                                              np.array(config_motors[i]['direction'])))

        PybulletAPI.changeDynamics(self.id, linearDamping=0.9, angularDamping=0.9)

        self.motor_debug_lines = list()
        self._motor_count = len(config_motors)
        self.rpm_motors = list()
        self.desired_rpm_motors = list()
        for i in range(self._motor_count):
            self.rpm_motors.append(0)
            self.desired_rpm_motors.append(0)
            self.motor_debug_lines.append(DebugLine(self.motors[i].position, self.motors[i].position))

        # self.buoyancySphereShape = p.createVisualShape(p.GEOM_SPHERE, radius=0.2, rgbaColor=[1, 0, 0, 1])
        # self.buoyancyPointIndicator = p.createMultiBody(0, -1, self.buoyancySphereShape, [0, 0, 0],
        #                                                 useMaximalCoordinates=0)

        self.depth_sensor = DepthSensor(self, [1, 0, 0], None, SimulationTime(4000))
        # self.imu = IMU(self.id, [0, 0, 0], [0, 0, 0, 0], SimulationTime(1000))
        self.accelerometer = Accelerometer(self, [1, 0, 0], None, SimulationTime(4000))
        self.gyroscope = Gyroscope(self, [1, 0, 0], None, SimulationTime(4000))
        self.magnetometer = Magnetometer(self, [1, 0, 0], None, SimulationTime(4000))

        self.max_thrust = 100
        self.buoyancy = 550

    def set_buoyancy(self, value):
        self.buoyancy = value

    def set_desired_rpm_motors(self, desired_rpm_motors):
        for i in range(len(desired_rpm_motors)):
            self.motors[i].set_desired_rpm(desired_rpm_motors[i])

    def set_desired_rpm_motor(self, index, desired_rpm):
        self.desired_rpm_motors[index] = desired_rpm

    def set_desired_thrust_motors(self, desired_thrusts):
        for i in range(len(desired_thrusts)):
            self.motors[i].set_desired_thrust(desired_thrusts[i])

    def set_desired_thrust_motor(self, index: int, desired_thrust: float):
        self.motors[index].set_desired_thrust(desired_thrust)

    def update(self, dt: SimulationTime, time: SimulationTime):
        """
        Updates the UUV by the specified time.
        :param dt: dt in microseconds
        :param time: time in microseconds
        """
        lobster_pos, lobster_orn = self.get_position_and_orientation()

        self.depth_sensor.update(time)
        self.accelerometer.update(time)
        self.gyroscope.update(time)
        self.magnetometer.update(time)

        for i in range(self._motor_count):
            self.motors[i].update(dt.microseconds)

        # Apply forces for the  facing motors
        for i in range(self._motor_count):
            self.motors[i].apply_thrust()
            self.motor_debug_lines[i].update(self.motors[i].position,
                                             self.motors[i].position
                                             + self.motors[i].direction * self.motors[i].get_thrust() / 100,
                                             self.id)

        # Determine the point where the buoyancy force acts on the robot
        buoyancy_force_pos = np.reshape(np.array(PybulletAPI.getMatrixFromQuaternion(lobster_orn)), (3, 3)).dot(
            np.array(self.center_of_volume)) + lobster_pos

        # Apply the buoyancy force
        PybulletAPI.applyExternalForce(objectUniqueId=self.id,
                                       forceObj=[0, 0, self.buoyancy], posObj=buoyancy_force_pos,
                                       frame=Frame.WORLD_FRAME)

    def get_position_and_orientation(self):
        return PybulletAPI.getBasePositionAndOrientation(self.id)

    def get_position(self):
        position, _ = self.get_position_and_orientation()
        return position

    def get_orientation(self):
        _, orientation = self.get_position_and_orientation()
        return orientation

    def get_velocity(self):
        return PybulletAPI.getBaseVelocity(self.id)[0]

    def get_angular_velocity(self):
        return PybulletAPI.getBaseVelocity(self.id)[1]

    def set_position_and_orientation(self, position=None, orientation=None):
        if position is None:
            position = self.get_position()
        if orientation is None:
            orientation = self.get_orientation()

        PybulletAPI.resetBasePositionAndOrientation(self.id, position, orientation)

    def set_velocity(self, linear_velocity=None, angular_velocity=None):
        if linear_velocity is None:
            linear_velocity = self.get_velocity()
        if angular_velocity is None:
            angular_velocity = self.get_angular_velocity()

        PybulletAPI.resetBaseVelocity(self.id, linear_velocity, angular_velocity)
