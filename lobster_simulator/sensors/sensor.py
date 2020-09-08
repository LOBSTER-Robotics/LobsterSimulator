# This is needed to resolve the Lobster class type, since it can't be imported due to a cyclic dependency
from __future__ import annotations

from abc import ABC, abstractmethod
from typing import List, TYPE_CHECKING, Union, Optional
import numpy as np

from lobster_simulator.common.calculations import interpolate
from lobster_simulator.common.general_exceptions import ArgumentLengthError
from lobster_simulator.common.pybullet_api import PybulletAPI

if TYPE_CHECKING:
    from lobster_simulator.robot.auv import AUV
    from lobster_common.quaternion import Quaternion

from lobster_common.vec3 import Vec3

from lobster_simulator.common.simulation_time import SimulationTime


class Sensor(ABC):

    def __init__(self, robot: AUV, position: Vec3, time_step: SimulationTime, time: SimulationTime, orientation: Quaternion,
                 noise_stds: Optional[Union[List[float], float]]):
        """
        Parameters
        ----------
        robot : AUV
            The robot the sensor is attached to.
        position : array[3]
            The local position of the sensor on the robot.
        orientation : array[4]
            The orientation of the sensor on the robot as a quaternion.
        time_step : int
            The time step between two polls on the sensor in microseconds.
        orientation : Quaternion
            Orientation of the sensor w.r.t. the robot.
        noise_stds :Union[List[float], float]
            Number or list of numbers with the standard deviation for each of the outputs of the sensor.
        """

        if orientation is None:
            orientation = PybulletAPI.getQuaternionFromEuler(Vec3([0, 0, 0]))

        self._robot: AUV = robot
        self._sensor_position: Vec3 = position
        self._sensor_orientation: Quaternion = orientation
        self._time_step = time_step

        self._queue = list()

        self._next_sample_time: SimulationTime = SimulationTime(initial_microseconds=time.microseconds)
        self._previous_update_time: SimulationTime = SimulationTime(initial_microseconds=time.microseconds)
        self._previous_real_value = self._get_real_values(SimulationTime(1))

        self.noise_stds = None
        if noise_stds:
            self.set_noise(noise_stds)

    def update(self, time: SimulationTime, dt: SimulationTime):
        """
        Updates a sensor, by generating new outputs by interpolating between values on the current and previous time
        step
        :param dt:
        :param time: Current time in the simulator
        """

        real_values = self._get_real_values(dt)

        while self._next_sample_time <= time:

            value_outputs = list()
            for i in range(len(real_values)):
                value = interpolate(x=self._next_sample_time.microseconds,
                                    x1=self._previous_update_time.microseconds,
                                    x2=time.microseconds,
                                    y1=self._previous_real_value[i],
                                    y2=real_values[i])

                if self.noise_stds:
                    value += np.random.normal(0, self.noise_stds[i])

                value_outputs.append(value)

            self._queue.append(value_outputs)
            self._next_sample_time += self._time_step

        self._previous_real_value = real_values
        self._previous_update_time = SimulationTime(time.microseconds)

    def set_noise(self, noise_stds: Union[List[float], float]):
        if not isinstance(noise_stds, List):
            noise_stds = [noise_stds]

        if len(self._previous_real_value) != len(noise_stds):
            raise ArgumentLengthError("The length of the list of standard deviations of the noise should be the "
                                      "same length as the amount of values the sensor produces.")

        self.noise_stds = noise_stds

    def pop_next_value(self):
        """Pops the oldest sensor value from the list"""
        if len(self._queue) == 0:
            return None
        return self._queue.pop(0)

    def pop_all_values(self) -> List:
        """Gives a list with all the sensors values, from oldest to newest."""
        values = self._queue
        self._queue = list()
        return values

    def get_sensor_position(self) -> Vec3:
        return self._sensor_position

    def get_sensor_orientation(self) -> Quaternion:
        return self._sensor_orientation

    def get_latest_value(self):
        if self._queue:
            return self._queue[-1]

        return None

    @abstractmethod
    def _get_real_values(self, dt: SimulationTime) -> List[float]:
        """
        :param dt: dt in microseconds
        :return: The real values of the data that the sensor meassures.
        """
        raise NotImplementedError("This method should be implemented")
