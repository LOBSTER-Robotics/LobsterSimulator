import math
from typing import List

import pybullet as p

from lobster_simulator.sensors.Sensor import Sensor
from lobster_simulator.tools.Translation import vec3_local_to_world_id


class DepthSensor(Sensor):

    def __init__(self, pybullet_id, position, orientation, time_step):
        super(DepthSensor, self).__init__(pybullet_id, position, orientation, time_step)

        self.previous_real_value = [0]

    def _get_initial_values(self) -> List[float]:
        return [vec3_local_to_world_id(self.pybullet_id, self.position)[2]]

    def _get_real_values(self, dt: int) -> List[float]:
        # print(f"getting values: {vec3_local_to_world_id(self.pybullet_id, self.position)[2]}")
        return [vec3_local_to_world_id(self.pybullet_id, self.position)[2]]

    def get_depth(self):
        return self._previous_real_value[0]
