import math

import pybullet as p

from lobster_simulator.sensors.Sensor import Sensor
from lobster_simulator.tools.Translation import vec3_local_to_world_id


class DepthSensor(Sensor):

    def __init__(self, pybullet_id, position, orientation, time_step):
        super(DepthSensor, self).__init__(pybullet_id, position, orientation, time_step)

        self.queue = list()

        self.next_sample_time = 0
        self.previous_update_time = 0
        self.previous_update_depth = 0

    def update(self, time):
        dt = time - self.previous_update_time
        current_depth = self.__get_depth()

        while self.next_sample_time <= time:
            slope = (current_depth - self.previous_update_depth) / dt
            output = self.previous_update_depth + slope * (self.next_sample_time - self.previous_update_time)
            self.queue.append(output)

            self.next_sample_time += self.time_step

        self.previous_update_time = time
        self.previous_update_depth = current_depth

    def pop_next_value(self):
        if len(self.queue) == 0:
            return None
        return self.queue.pop()

    def get_all_values(self):
        values = self.queue
        self.queue = list()
        return values

    def __get_depth(self):
        return vec3_local_to_world_id(self.pybullet_id, self.position)[2]


