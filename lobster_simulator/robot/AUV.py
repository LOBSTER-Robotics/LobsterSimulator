import numpy as np

from typing import List

from pkg_resources import resource_filename

from lobster_simulator.common.Vec3 import Vec3
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

        self._center_of_volume = config['center_of_volume']

        self._id = PybulletAPI.loadURDF(resource_filename("lobster_simulator", "data/Model_URDF.SLDASM.urdf"),
                                        Vec3([0, 0, -1]),
                                        PybulletAPI.getQuaternionFromEuler([0, 0, 0]))

        config_motors = config['motors']

        self._motors: List[Motor] = list()
        for i in range(len(config_motors)):
            self._motors.append(Motor.new_T200(self._id, config_motors[i]['name'],
                                               np.array(config_motors[i]['position']),
                                               np.array(config_motors[i]['direction'])))

        PybulletAPI.changeDynamics(self._id, linearDamping=0.9, angularDamping=0.9)

        self._motor_debug_lines = list()
        self._motor_count = len(config_motors)
        self._rpm_motors = list()
        self._desired_rpm_motors = list()
        for i in range(self._motor_count):
            self._rpm_motors.append(0)
            self._desired_rpm_motors.append(0)
            self._motor_debug_lines.append(DebugLine(self._motors[i]._position, self._motors[i]._position))

        # self.buoyancySphereShape = p.createVisualShape(p.GEOM_SPHERE, radius=0.2, rgbaColor=[1, 0, 0, 1])
        # self.buoyancyPointIndicator = p.createMultiBody(0, -1, self.buoyancySphereShape, [0, 0, 0],
        #                                                 useMaximalCoordinates=0)

        self._depth_sensor = DepthSensor(self, Vec3([1, 0, 0]), None, SimulationTime(4000))
        # self.imu = IMU(self.id, [0, 0, 0], [0, 0, 0, 0], SimulationTime(1000))
        self._accelerometer = Accelerometer(self, Vec3([1, 0, 0]), None, SimulationTime(4000))
        self._gyroscope = Gyroscope(self, Vec3([1, 0, 0]), None, SimulationTime(4000))
        self._magnetometer = Magnetometer(self, Vec3([1, 0, 0]), None, SimulationTime(4000))

        self._max_thrust = 100
        self._buoyancy = 550

    def set_buoyancy(self, value):
        self._buoyancy = value

    def set_desired_rpm_motors(self, desired_rpm_motors):
        for i in range(len(desired_rpm_motors)):
            self._motors[i].set_desired_rpm(desired_rpm_motors[i])

    def set_desired_rpm_motor(self, index, desired_rpm):
        self._desired_rpm_motors[index] = desired_rpm

    def set_desired_thrust_motors(self, desired_thrusts):
        for i in range(len(desired_thrusts)):
            self._motors[i].set_desired_thrust(desired_thrusts[i])

    def set_desired_thrust_motor(self, index: int, desired_thrust: float):
        self._motors[index].set_desired_thrust(desired_thrust)

    def update(self, dt: SimulationTime, time: SimulationTime):
        """
        Updates the UUV by the specified time.
        :param dt: dt in microseconds
        :param time: time in microseconds
        """
        lobster_pos, lobster_orn = self.get_position_and_orientation()

        self._depth_sensor.update(time)
        self._accelerometer.update(time)
        self._gyroscope.update(time)
        self._magnetometer.update(time)

        for i in range(self._motor_count):
            self._motors[i].update(dt.microseconds)

        # Apply forces for the  facing motors
        for i in range(self._motor_count):
            self._motors[i].apply_thrust()
            self._motor_debug_lines[i].update(self._motors[i]._position,
                                              self._motors[i]._position
                                              + self._motors[i]._direction * self._motors[i].get_thrust() / 100,
                                              self._id)

        # Determine the point where the buoyancy force acts on the robot
        buoyancy_force_pos = np.reshape(np.array(PybulletAPI.getMatrixFromQuaternion(lobster_orn)), (3, 3)).dot(
            np.array(self._center_of_volume)) + lobster_pos

        # Apply the buoyancy force
        PybulletAPI.applyExternalForce(objectUniqueId=self._id,
                                       forceObj=[0, 0, self._buoyancy], posObj=buoyancy_force_pos,
                                       frame=Frame.WORLD_FRAME)

    def get_position_and_orientation(self):
        return PybulletAPI.getBasePositionAndOrientation(self._id)

    def get_position(self):
        position, _ = self.get_position_and_orientation()
        return position

    def get_orientation(self):
        _, orientation = self.get_position_and_orientation()
        return orientation

    def get_velocity(self) -> Vec3:
        return PybulletAPI.getBaseVelocity(self._id)[0]

    def get_angular_velocity(self):
        return PybulletAPI.getBaseVelocity(self._id)[1]

    def set_position_and_orientation(self, position=None, orientation=None):
        if position is None:
            position = self.get_position()
        if orientation is None:
            orientation = self.get_orientation()

        PybulletAPI.resetBasePositionAndOrientation(self._id, position, orientation)

    def set_velocity(self, linear_velocity=None, angular_velocity=None):
        if linear_velocity is None:
            linear_velocity = self.get_velocity()
        if angular_velocity is None:
            angular_velocity = self.get_angular_velocity()

        PybulletAPI.resetBaseVelocity(self._id, linear_velocity, angular_velocity)
