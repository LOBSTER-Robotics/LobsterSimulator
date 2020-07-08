from __future__ import annotations

from typing import List, TYPE_CHECKING

if TYPE_CHECKING:
    from lobster_simulator.robot.UUV import UUV

from lobster_simulator.sensors.Sensor import Sensor
from lobster_simulator.simulation_time import SimulationTime
from lobster_simulator.tools.Translation import *

MAGNETIC_FIELD = [1, 0, 0]


class Gyroscope(Sensor):

    def __init__(self, robot: UUV, position: np.array, orientation: np.array, time_step: SimulationTime):
        super().__init__(robot, position, orientation, time_step)

    def _get_real_values(self, dt: SimulationTime):
        rotation = self._robot.get_angular_velocity()

        # Rotate rotational velocity to robot reference frame
        robot_rotation = vec3_rotate_vector_to_local(self._robot.get_orientation(), rotation)

        # Rotate rotational velocity to sensor reference frame
        sensor_rotation = vec3_rotate_vector_to_local(self._sensor_orientation, rotation)

        return [sensor_rotation]

    def get_gyroscope_value(self):
        return self._previous_real_value[0]
