from __future__ import annotations

from typing import List, TYPE_CHECKING

if TYPE_CHECKING:
    from lobster_simulator.robot.UUV import UUV

from lobster_simulator.sensors.Sensor import Sensor
from lobster_simulator.tools.Constants import GRAVITY
from lobster_simulator.tools.Translation import vec3_local_to_world_id


class DepthSensor(Sensor):

    KPA_TO_PA = 1000
    STANDARD_ATMOSPHERE = 101300

    # kg/m^3
    DENSITY_FRESHWATER = 997
    DENSITY_SALTWATER = 1029
    OFFSET = 0

    def __init__(self, robot: UUV, position, orientation, time_step, saltwater=False):
        if saltwater:
            self.water_density = self.DENSITY_SALTWATER
        else:
            self.water_density = self.DENSITY_FRESHWATER

        super(DepthSensor, self).__init__(robot, position, orientation, time_step)

        self.previous_real_value = [0]

    def _get_real_values(self, dt: int) -> List[float]:
        # print(f"getting values: {vec3_local_to_world_id(self.pybullet_id, self.position)[2]}")
        depth = -vec3_local_to_world_id(self.robot.id, self.position)[2]

        pressure = (depth * ((self.water_density * GRAVITY) + self.OFFSET) + self.STANDARD_ATMOSPHERE) / self.KPA_TO_PA

        return [pressure]

    def get_pressure(self):
        return self._previous_real_value[0]
