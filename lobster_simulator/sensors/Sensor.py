# This is needed to resolve the Lobster class type, since it can't be imported due to a cyclic dependency
from __future__ import annotations

from abc import ABC, abstractmethod
from typing import List, TYPE_CHECKING
import numpy as np


from lobster_simulator.tools.PybulletAPI import PybulletAPI

if TYPE_CHECKING:
    from lobster_simulator.robot.AUV import AUV
    from lobster_simulator.common.Quaternion import Quaternion

from lobster_simulator.common.Vec3 import Vec3

from lobster_simulator.simulation_time import SimulationTime


class Sensor(ABC):

    def __init__(self, robot: AUV, position: Vec3, orientation: Quaternion, time_step: SimulationTime):
        """
        Parameters
        ----------
        pybullet_id : int
            The pybullet id of the robot the sensor is attached to.
        position : array[3]
            The local position of the sensor on the robot.
        orientation : array[4]
            The orientation of the sensor on the robot as a quaternion.
        time_step : int
            The time step between two polls on the sensor in microseconds.
        """
        if orientation is None:
            orientation = PybulletAPI.getQuaternionFromEuler(Vec3([0, 0, 0]))

        self._robot: AUV = robot
        self._position: Vec3 = position
        self._sensor_orientation: Quaternion = orientation
        self._time_step = time_step

        self._queue = list()

        self._next_sample_time: SimulationTime = SimulationTime(0)
        self._previous_update_time: SimulationTime = SimulationTime(0)
        self._previous_real_value = self._get_real_values(SimulationTime(1))

    def update(self, time: SimulationTime):
        """

        :param time: time in microseconds
        :return:
        """
        # print(time, self.previous_update_time)
        dt = time - self._previous_update_time
        real_values = self._get_real_values(dt)

        while self._next_sample_time <= time:

            value_outputs = list()
            for i in range(len(real_values)):
                value_dt = (real_values[i] - self._previous_real_value[i]) / dt.microseconds
                value_output = self._previous_real_value[i] + value_dt * (
                        self._next_sample_time - self._previous_update_time).microseconds


                # print(value_output, real_values[i], self.previous_real_value[i])
                value_outputs.append(value_output)


            self._queue.append(value_outputs)
            self._next_sample_time += self._time_step

        self._previous_real_value = real_values
        self._previous_update_time = SimulationTime(time.microseconds)

    def pop_next_value(self):
        if len(self._queue) == 0:
            return None
        return self._queue.pop()

    def get_all_values(self):
        values = self._queue
        self._queue = list()
        return values

    def get_sensor_position(self):
        return self._position

    def get_sensor_orientation(self):
        return self._sensor_orientation

    @abstractmethod
    def _get_real_values(self, dt: SimulationTime) -> List[float]:
        """
        :param dt: dt in microseconds
        :return:
        """
        raise NotImplementedError("This method should be implemented")
