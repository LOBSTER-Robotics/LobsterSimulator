from typing import List, Tuple, Dict

import numpy as np
from pkg_resources import resource_filename

from lobster_simulator.common.general_exceptions import ArgumentNoneError
from lobster_simulator.robot.thruster import Thruster
from lobster_simulator.robot.buoyancy import Buoyancy
from lobster_simulator.sensors.accelerometer import Accelerometer
from lobster_simulator.sensors.dvl import DVL
from lobster_simulator.sensors.pressure_sensor import DepthSensor
from lobster_simulator.sensors.gyroscope import Gyroscope
from lobster_simulator.sensors.magnetometer import Magnetometer
from lobster_simulator.common.simulation_time import SimulationTime
from lobster_common.constants import *
from lobster_simulator.common.pybullet_api import Frame
from lobster_simulator.common.pybullet_api import PybulletAPI as p
from lobster_simulator.common.translation import *


class AUV:

    def __init__(self, config):
        if config is None:
            raise ArgumentNoneError("config parameter should not be None")

        self._center_of_volume = Vec3(config['center_of_volume'])

        self.damping_matrix: np.ndarray = np.diag(config['damping_matrix_diag'])

        self._id = p.loadURDF(resource_filename("lobster_simulator", "data/scout-alpha.urdf"),
                              Vec3([0, 0, 2]),
                              p.getQuaternionFromEuler(Vec3([0, 0, 0])))

        self._buoyancy = Buoyancy(self, 0.10, 2, resolution=config.get('buoyancy_resolution'))
        config_thrusters = config['thrusters']

        self.thrusters: Dict[str, Thruster] = dict()
        for i in range(len(config_thrusters)):
            self.thrusters[config_thrusters[i]['name']] = Thruster.new_T200(self, config_thrusters[i]['name'],
                                                                            Vec3(config_thrusters[i]['position']),
                                                                            Vec3(config_thrusters[i]['direction']))

        # Set damping to zero, because default is not zero
        p.changeDynamics(self._id, linearDamping=0.0, angularDamping=0.0)

        self._motor_debug_lines = list()
        self._motor_count = len(config_thrusters)
        self._rpm_motors = list()
        self._desired_rpm_motors: List[float] = list()

        self._depth_sensor = DepthSensor(self, Vec3([1, 0, 0]), SimulationTime(4000))
        self._accelerometer = Accelerometer(self, Vec3([1, 0, 0]), SimulationTime(4000))
        self._gyroscope = Gyroscope(self, Vec3([1, 0, 0]), SimulationTime(4000))
        self._magnetometer = Magnetometer(self, Vec3([1, 0, 0]), SimulationTime(4000))
        self._dvl = DVL(self, Vec3([-.5, 0, 0.10]), SimulationTime(4000))

        self._max_thrust = 100

    # TODO: This should in the future be based on the volume of the robot.
    def set_buoyancy(self, value: float):
        """
        Sets the force of the buoyance that acts on the robot
        :param value: Bouyance force in Newtons.
        """
        self._buoyancy = value

    def get_thruster(self, name: str):
        return self.thrusters[name]

    def update(self, dt: SimulationTime, time: SimulationTime):
        """
        Updates the UUV by the specified time.
        :param dt: dt in microseconds
        :param time: time in microseconds
        """
        self._depth_sensor.update(time, dt)
        self._accelerometer.update(time, dt)
        self._gyroscope.update(time, dt)
        self._magnetometer.update(time, dt)
        self._dvl.update(time, dt)

        self._buoyancy._update()

        for thruster in self.thrusters.values():
            thruster._update(dt)

        self._apply_damping()

    def get_position_and_orientation(self) -> Tuple[Vec3, Quaternion]:
        """
        Gets both the position and the orientation of the robot.
        :return: Tuple with the position and the orientation.
        """
        return p.getBasePositionAndOrientation(self._id)

    def get_position(self) -> Vec3:
        """
        Position of the robot.
        :return: Vec3 with the position.
        """
        position, _ = self.get_position_and_orientation()
        return position

    def get_orientation(self) -> Quaternion:
        """
        Gives the orientation of the robot in the world frame.
        :return: Orientation as a quaternion.
        """
        _, orientation = self.get_position_and_orientation()
        return orientation

    def get_velocity(self) -> Vec3:
        """
        Gets the linear velocity of the Robot in the World frame.
        """
        return p.getBaseVelocity(self._id)[0]

    def get_angular_velocity(self):
        return p.getBaseVelocity(self._id)[1]

    def apply_force(self, force_pos: Vec3, force: Vec3, relative_direction: bool = True) -> None:
        """
        Applies a force to the robot (should only be used for testing purposes).
        :param force_pos: Position of the acting force, given in the local frame
        :param force: Force vector
        :param relative_direction: Determines whether or not the force is defined in the local or world frame.
        """

        if not relative_direction:
            # If the force direction is given in the world frame, it should be rotated to the local frame
            force = vec3_rotate_vector_to_local(self.get_orientation(), force)

        # Apply the force in the local frame
        p.applyExternalForce(self._id, force, force_pos, Frame.LINK_FRAME)

    def set_position_and_orientation(self, position: Vec3 = None, orientation: Quaternion = None) -> None:
        """
        Sets the position and/or the orientation of the robot (should only be used for testing purposes).
        :param position: Position
        :param orientation: Orientation
        """
        if position is None:
            position = self.get_position()
        if orientation is None:
            orientation = self.get_orientation()

        p.resetBasePositionAndOrientation(self._id, position, orientation)

    def set_velocity(self, linear_velocity: Vec3 = None, angular_velocity: Vec3 = None, local_frame=False) -> None:
        """
        Sets the linear and/or angular velocity of the robot (should only be used for testing purposes).
        :param linear_velocity: The linear velocity that the robot should have.
        :param angular_velocity: The angular velocity that the robot should have.
        :param local_frame: Whether the velocities are given in the local or world frame.
        """
        if linear_velocity is None:
            linear_velocity = self.get_velocity()
        if angular_velocity is None:
            angular_velocity = self.get_angular_velocity()

        if local_frame:
            linear_velocity = vec3_rotate_vector_to_world(self.get_orientation(), linear_velocity)
            angular_velocity = vec3_rotate_vector_to_world(self.get_orientation(), angular_velocity)

        p.resetBaseVelocity(self._id, linear_velocity, angular_velocity)

    def _apply_damping(self):
        """
        Applies damping to the robot on its linear and angular velocity.
        See https://docs.lobster-robotics.com/scout/robots/scout-alpha/scout-simulator-model
        """

        # Don't apply damping if the robot is above water
        # TODO. This creates a unrealistic hard boundary for enabling and disabling damping, so perhaps this should be
        #  improved in the future.
        if self.get_position()[Z] < 0:
            return

        velocity = vec3_rotate_vector_to_local(self.get_orientation(), self.get_velocity())
        angular_velocity = vec3_rotate_vector_to_local(self.get_orientation(), self.get_angular_velocity())

        damping = -np.dot(self.damping_matrix, np.concatenate((velocity.numpy(), angular_velocity.numpy())))

        # Multiply the damping for now to get slightly more accurate results
        damping *= 10

        linear_damping_force = Vec3(damping[:3])
        angular_damping_torque = Vec3(damping[3:])

        p.applyExternalForce(self._id, forceObj=linear_damping_force, posObj=Vec3([0, 0, 0]),
                             frame=Frame.LINK_FRAME)
        p.applyExternalTorque(self._id, torqueObj=angular_damping_torque, frame=Frame.LINK_FRAME)

    def remove(self):
        p.removeBody(self._id)
        self._dvl.remove()
        self._buoyancy.remove()
        for thruster in self.thrusters.values():
            thruster.remove()