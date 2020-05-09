from abc import ABC, abstractmethod
from typing import List


class Sensor(ABC):

    def __init__(self, pybullet_id, position, orientation, time_step):
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

        self.pybullet_id = pybullet_id
        self.position = position
        self.orientation = orientation
        self.time_step = time_step

        self.queue = list()

        self.next_sample_time: int = 0
        self.previous_update_time: int = 0
        self.previous_real_value = self._get_real_values(0)

    def update(self, time: int):
        """

        :param time: time in microseconds
        :return:
        """
        dt = time - self.previous_update_time
        real_values = self._get_real_values(dt)

        while self.next_sample_time <= time:

            value_outputs = list()
            for i in range(len(real_values)):
                value_dt = (real_values[i] - self.previous_real_value[i]) / dt
                value_output = self.previous_real_value[i] + value_dt * (
                        self.next_sample_time - self.previous_update_time)
                value_outputs.append(value_output)

            self.queue.append(value_outputs)
            self.next_sample_time += self.time_step

        self.previous_update_time = time

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
        return self.orientation

    @abstractmethod
    def _get_real_values(self, dt: int) -> List[float]:
        """

        :param dt: dt in microseconds
        :return:
        """
        raise NotImplementedError("This method should be implemented")
