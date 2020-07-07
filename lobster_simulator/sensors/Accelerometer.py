from __future__ import annotations

from typing import List

from lobster_simulator.robot import Lobster
from lobster_simulator.sensors.Sensor import Sensor
from lobster_simulator.simulation_time import SimulationTime, MICROSECONDS_IN_SECONDS
from lobster_simulator.tools.Translation import *

GRAVITY = np.array([0, 0, -9.81])


class Accelerometer(Sensor):

    def __init__(self, robot: Lobster, position: np.array, orientation: np.array, time_step: SimulationTime):
        self._previous_linear_velocity = np.array([0, 0, 0])
        super().__init__(robot, position, orientation, time_step)

    def update(self, time: SimulationTime):
        super().update(time)
        self._previous_linear_velocity = self._get_linear_velocity()

    def _get_real_values(self, dt: SimulationTime) -> List[np.ndarray]:
        linear_velocity = self._get_linear_velocity()
        linear_acceleration = 0
        if dt.microseconds > 0:
            linear_acceleration = (linear_velocity - self._previous_linear_velocity) * MICROSECONDS_IN_SECONDS / dt.microseconds

        # Rotate the gravity vector to the robot reference frame
        gravity_local_frame = vec3_rotate_vector_to_local(self.robot.get_orientation(), GRAVITY)

        # Rotate the gravity vector to the sensor reference frame
        gravity_sensor_frame = vec3_rotate_vector_to_local(self.sensor_orientation, gravity_local_frame)

        linear_acceleration += gravity_sensor_frame

        return [linear_acceleration]

    def _get_linear_velocity(self) -> np.ndarray:
        return np.array(self.robot.get_velocity())

    def get_accelerometer_value(self):
        return self._previous_real_value[0]
