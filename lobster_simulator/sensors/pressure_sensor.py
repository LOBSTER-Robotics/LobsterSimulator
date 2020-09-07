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


class PressureSensor(Sensor):

    _KPA_TO_PA = 1000
    _STANDARD_ATMOSPHERE = 101300

    # kg/m^3
    _DENSITY_FRESHWATER = 997
    _DENSITY_SALTWATER = 1029
    _OFFSET = 0

    def __init__(self, robot: AUV, position: Vec3, time_step: SimulationTime, orientation: Quaternion = None,
                 saltwater=False, noise_stds: Union[List[float], float] = None):
        if saltwater:
            self._water_density = self._DENSITY_SALTWATER
        else:
            self._water_density = self._DENSITY_FRESHWATER

        super(PressureSensor, self).__init__(robot, position, time_step, orientation, noise_stds)

    def _get_real_values(self, dt: int) -> List[float]:
        # print(f"getting values: {vec3_local_to_world_id(self.pybullet_id, self.position)[2]}")
        # print(vec3_local_to_world_id(self._robot._id, self._position)[2])
        depth = vec3_local_to_world_id(self._robot.object_id, self._sensor_position)[2]

        pressure = (depth * ((self._water_density * GRAVITY) + self._OFFSET) + self._STANDARD_ATMOSPHERE) / self._KPA_TO_PA

        return [pressure]

    def get_pressure(self):
        return self._previous_real_value[0]
