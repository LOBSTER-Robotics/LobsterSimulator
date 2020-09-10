from __future__ import annotations

from typing import List, TYPE_CHECKING, Union

if TYPE_CHECKING:
    from lobster_simulator.robot.auv import AUV

from lobster_simulator.sensors.sensor import Sensor
from lobster_simulator.common.simulation_time import SimulationTime
from lobster_simulator.common.translation import *

MAGNETIC_FIELD = [1, 0, 0]


class Gyroscope(Sensor):

    def __init__(self, robot: AUV, position: Vec3, time_step: SimulationTime, time: SimulationTime, orientation: Quaternion = None, noise_stds: Union[List[float], float] = None):
        super().__init__(robot, position, time_step, time, orientation, noise_stds)

    def _get_real_values(self, dt: SimulationTime):
        rotation = self._robot.get_angular_velocity()

        # Rotate rotational velocity to robot reference frame
        robot_rotation = vec3_rotate_vector_to_local(self._robot.get_orientation(), rotation)

        # Rotate rotational velocity to sensor reference frame
        sensor_rotation = vec3_rotate_vector_to_local(self._sensor_orientation, robot_rotation)

        return [sensor_rotation]

    def get_gyroscope_value(self):
        return self._previous_real_value[0]
