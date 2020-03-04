import pybullet as p

from lobster_simulator.sensors.Sensor import Sensor
from lobster_simulator.tools.Translation import vec3_local_to_world_id


class DepthSensor(Sensor):

    def get_depth(self):
        return vec3_local_to_world_id(self.pybullet_id, self.position)[2]
