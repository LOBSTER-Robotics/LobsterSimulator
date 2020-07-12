from __future__ import annotations

from typing import List, TYPE_CHECKING

if TYPE_CHECKING:
    from lobster_simulator.robot.AUV import UUV

from lobster_simulator.sensors.Sensor import Sensor
from lobster_simulator.tools.Constants import GRAVITY
from lobster_simulator.tools.Translation import vec3_local_to_world_id


class DepthSensor(Sensor):

    _KPA_TO_PA = 1000
    _STANDARD_ATMOSPHERE = 101300

    # kg/m^3
    _DENSITY_FRESHWATER = 997
    _DENSITY_SALTWATER = 1029
    _OFFSET = 0

    def __init__(self, robot: UUV, position, orientation, time_step, saltwater=False):
        if saltwater:
            self._water_density = self._DENSITY_SALTWATER
        else:
            self._water_density = self._DENSITY_FRESHWATER

        super(DepthSensor, self).__init__(robot, position, orientation, time_step)

    def _get_real_values(self, dt: int) -> List[float]:
        # print(f"getting values: {vec3_local_to_world_id(self.pybullet_id, self.position)[2]}")
        # print(vec3_local_to_world_id(self._robot._id, self._position)[2])
        depth = -vec3_local_to_world_id(self._robot._id, self._position)[2]

        pressure = (depth * ((self._water_density * GRAVITY) + self._OFFSET) + self._STANDARD_ATMOSPHERE) / self._KPA_TO_PA

        return [pressure]

    def get_pressure(self):
        return self._previous_real_value[0]
