from __future__ import annotations

from abc import ABC, abstractmethod
from typing import List
import numpy as np
import pybullet as p

from lobster_simulator.simulation_time import SimulationTime


class Sensor(ABC):

    def __init__(self, robot: Lobster, position: np.array, orientation: np.array, time_step: SimulationTime):
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
            orientation = p.getQuaternionFromEuler([0, 0, 0])

        self.robot = robot
        self.position = position
        self.sensor_orientation = orientation
        self.time_step = time_step

        self.queue = list()

        self.next_sample_time: SimulationTime = SimulationTime(0)
        self.previous_update_time: SimulationTime = SimulationTime(0)
        self._previous_real_value = self._get_real_values(SimulationTime(1))

    def update(self, time: SimulationTime):
        """

        :param time: time in microseconds
        :return:
        """
        # print(time, self.previous_update_time)
        dt = time - self.previous_update_time
        real_values = self._get_real_values(dt)

        while self.next_sample_time <= time:

            value_outputs = list()
            for i in range(len(real_values)):
                value_dt = (real_values[i] - self._previous_real_value[i]) / dt.microseconds
                value_output = self._previous_real_value[i] + value_dt * (
                        self.next_sample_time - self.previous_update_time).microseconds


                # print(value_output, real_values[i], self.previous_real_value[i])
                value_outputs.append(value_output)


            self.queue.append(value_outputs)
            self.next_sample_time += self.time_step

        self._previous_real_value = real_values
        self.previous_update_time = SimulationTime(time.microseconds)

    def pop_next_value(self):
        if len(self.queue) == 0:
            return None
        return self.queue.pop()

    def get_all_values(self):
        values = self.queue
        self.queue = list()
        return values

    def get_sensor_position(self):
        return self.position

    def get_sensor_orientation(self):
        return self.sensor_orientation
    #
    # @abstractmethod
    # def _get_initial_values(self) -> List[float]:
    #     """
    #     :return:
    #     """
    #     raise NotImplementedError("This method should be implemented")

    @abstractmethod
    def _get_real_values(self, dt: SimulationTime) -> List[float]:
        """
        :param dt: dt in microseconds
        :return:
        """
        raise NotImplementedError("This method should be implemented")
