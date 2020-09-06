from __future__ import annotations

from typing import List, TYPE_CHECKING, Union

from lobster_common.quaternion import Quaternion
from lobster_common.vec3 import Vec3
from lobster_simulator.common.simulation_time import SimulationTime

if TYPE_CHECKING:
    from lobster_simulator.robot.auv import AUV

from lobster_simulator.sensors.sensor import Sensor
from lobster_common.constants import *
from lobster_simulator.common.translation import vec3_local_to_world_id


class DepthSensor(Sensor):

    def __init__(self, robot: AUV, position: Vec3, time_step: SimulationTime, orientation: Quaternion = None,
                 saltwater=False, noise_stds: Union[List[float], float] = None):
        if saltwater:
            self._water_density = DENSITY_SALTWATER
        else:
            self._water_density = DENSITY_FRESHWATER

        super(DepthSensor, self).__init__(robot, position, time_step, orientation, noise_stds)

    def _get_real_values(self, dt: int) -> List[float]:
        # print(f"getting values: {vec3_local_to_world_id(self.pybullet_id, self.position)[2]}")
        # print(vec3_local_to_world_id(self._robot._id, self._position)[2])
        depth = vec3_local_to_world_id(self._robot._id, self._sensor_position)[2]
        pa_to_kPa = 0.001
        # Pressure in Kilo pascal
        pressure = (depth * (self._water_density * GRAVITY) + STANDARD_ATMOSPHERE_PASCAL) * pa_to_kPa
        return [pressure]

    def get_pressure(self):
        """
        # Pressure in Kilo pascal
        """
        return self._previous_real_value[0]
