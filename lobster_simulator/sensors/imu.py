from __future__ import annotations

from typing import List, TYPE_CHECKING, Union

import numpy as np

if TYPE_CHECKING:
    from lobster_simulator.robot import AUV

from lobster_simulator.sensors.Sensor import Sensor
from lobster_simulator.simulation_time import SimulationTime
from lobster_simulator.common.PybulletAPI import PybulletAPI

ACCELERATION = 0


class IMU(Sensor):

    def __init__(self, robot: AUV, position: np.ndarray, orientation: np.ndarray, time_step: SimulationTime,
                 noise_stds: Union[List[float], float]):
        self._previous_linear_velocity = np.array([0, 0, 0])
        super().__init__(robot, position, time_step, orientation, noise_stds)

    def update(self, time: SimulationTime, dt: SimulationTime):
        super().update(time)
        self._previous_linear_velocity = self._get_linear_velocity()

    def _get_real_values(self, dt: SimulationTime):
        linear_velocity = self._get_linear_velocity()
        linear_acceleration = 0
        if dt.microseconds > 0:
            linear_acceleration = (linear_velocity - self._previous_linear_velocity) * 1000000 / dt.microseconds

        angular_velocity = np.array(self._robot.get_angular_velocity())
        orientation = np.array(self._robot.get_orientation())
        return orientation, linear_acceleration, angular_velocity,

    def _get_linear_velocity(self):
        return np.array(self._robot.get_velocity())

    def get_magnetometer_value(self):
        return PybulletAPI.getEulerFromQuaternion(self._previous_real_value[0])

    def get_accelerometer_value(self):
        return self._previous_real_value[1]

    def get_gyroscope_value(self):
        return self._previous_real_value[2]
    
    def get_orientation(self):
        return self._previous_real_value[0]
