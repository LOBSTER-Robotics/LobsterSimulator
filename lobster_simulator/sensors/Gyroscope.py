from __future__ import annotations

import numpy as np
import pybullet as p

from lobster_simulator.sensors.Sensor import Sensor
from lobster_simulator.simulation_time import SimulationTime
from lobster_simulator.tools.Translation import *

MAGNETIC_FIELD = [1, 0, 0]


class Gyroscope(Sensor):

    def __init__(self, robot: Lobster, position: np.array, orientation: np.array, time_step: SimulationTime):
        self._previous_linear_velocity = np.array([0, 0, 0])
        super().__init__(robot, position, orientation, time_step)

    def _get_real_values(self, dt: SimulationTime):
        rotation = self.robot.get_angular_velocity()

        # Rotate rotational velocity to robot reference frame
        robot_rotation = vec3_rotate_vector_to_local(self.robot.get_orientation(), rotation)

        # Rotate rotational velocity to sensor reference frame
        sensor_rotation = vec3_rotate_vector_to_local(self.sensor_orientation, rotation)

        return [sensor_rotation]

    def get_gyroscope_value(self):
        return self._previous_real_value[0]
