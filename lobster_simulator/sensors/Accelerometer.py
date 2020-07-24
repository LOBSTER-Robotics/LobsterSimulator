from __future__ import annotations

from typing import TYPE_CHECKING, List

if TYPE_CHECKING:
    from lobster_simulator.robot.AUV import AUV

from lobster_simulator.sensors.Sensor import Sensor
from lobster_simulator.simulation_time import SimulationTime, MICROSECONDS_IN_SECONDS
from lobster_simulator.tools.Constants import GRAVITY
from lobster_simulator.tools.Translation import *


class Accelerometer(Sensor):

    GRAVITY_VEC = Vec3([0, 0, GRAVITY])

    def __init__(self, robot: AUV, position: Vec3, orientation: Quaternion, time_step: SimulationTime):
        self._previous_linear_velocity = Vec3([0, 0, 0])
        super().__init__(robot, position, orientation, time_step)

    def update(self, time: SimulationTime, dt: SimulationTime):
        super().update(time, dt)
        self._previous_linear_velocity = self._get_linear_velocity()

    def _get_real_values(self, dt: SimulationTime) -> List[Vec3]:
        linear_velocity = self._get_linear_velocity()
        acceleration = 0
        if dt.microseconds > 0:
            acceleration = (linear_velocity - self._previous_linear_velocity) * MICROSECONDS_IN_SECONDS / dt.microseconds

        acceleration += self.GRAVITY_VEC

        # Rotate the gravity vector to the robot reference frame
        acceleration_local_frame = vec3_rotate_vector_to_local(self._robot.get_orientation(), acceleration)

        # Rotate the gravity vector to the sensor reference frame
        acceleration_sensor_frame = vec3_rotate_vector_to_local(self._sensor_orientation, acceleration_local_frame)

        return [acceleration_sensor_frame]

    def _get_linear_velocity(self) -> Vec3:
        return self._robot.get_velocity()

    def get_accelerometer_value(self):
        return self._previous_real_value[0]
