from __future__ import annotations

from typing import TYPE_CHECKING, List, Union

if TYPE_CHECKING:
    from lobster_simulator.robot.auv import AUV

from lobster_simulator.sensors.sensor import Sensor
from lobster_simulator.common.simulation_time import SimulationTime, MICROSECONDS_IN_SECONDS
from lobster_common.constants import *
from lobster_simulator.common.translation import *


class Accelerometer(Sensor):

    def __init__(self, robot: AUV, position: Vec3, time_step: SimulationTime, orientation: Quaternion = None, noise_stds: Union[List[float], float] = None):
        self._previous_linear_velocity = Vec3([0, 0, 0])
        super().__init__(robot, position=position, time_step=time_step, orientation=orientation, noise_stds=noise_stds)

    def update(self, time: SimulationTime, dt: SimulationTime):
        super().update(time, dt)
        self._previous_linear_velocity = self._get_linear_velocity()

    def _get_real_values(self, dt: SimulationTime) -> List[Vec3]:
        linear_velocity = self._get_linear_velocity()
        acceleration = (linear_velocity - self._previous_linear_velocity) * MICROSECONDS_IN_SECONDS / dt.microseconds

        acceleration += Vec3([0, 0, GRAVITY])

        # Rotate the gravity vector to the robot reference frame
        acceleration_local_frame = vec3_rotate_vector_to_local(self._robot.get_orientation(), acceleration)

        # Rotate the gravity vector to the sensor reference frame
        acceleration_sensor_frame = vec3_rotate_vector_to_local(self._sensor_orientation, acceleration_local_frame)

        return [acceleration_sensor_frame]

    def _get_linear_velocity(self) -> Vec3:
        return self._robot.get_velocity()

    def get_accelerometer_value(self):
        return self._previous_real_value[0]
