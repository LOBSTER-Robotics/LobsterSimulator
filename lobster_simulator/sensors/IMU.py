import numpy as np
import pybullet as p

from lobster_simulator.sensors.Sensor import Sensor
from typing import List
from lobster_simulator.simulation_time import SimulationTime

ACCELERATION = 0


class IMU(Sensor):

    def __init__(self, pybullet_id, position, orientation, time_step):
        self._previous_linear_velocity = np.array([0, 0, 0])
        super().__init__(pybullet_id, position, orientation, time_step)

    def update(self, time: SimulationTime):
        super().update(time)
        self._previous_linear_velocity = self._get_linear_velocity()

    def _get_real_values(self, dt: SimulationTime):
        linear_velocity = self._get_linear_velocity()
        linear_acceleration = 0
        if dt.microseconds > 0:
            linear_acceleration = (linear_velocity - self._previous_linear_velocity) * 1000000 / dt.microseconds
        angular_velocity = np.array(p.getBaseVelocity(self.pybullet_id)[1])
        orientation = np.array(p.getBasePositionAndOrientation(self.pybullet_id)[1])
        return orientation, linear_acceleration, angular_velocity,

    def _get_linear_velocity(self):
        return np.array(p.getBaseVelocity(self.pybullet_id)[0])

    def get_magnetometer_value(self):
        return p.getEulerFromQuaternion(self._previous_real_value[0])

    def get_accelerometer_value(self):
        return self._previous_real_value[1]

    def get_gyroscope_value(self):
        return self._previous_real_value[2]
    
    def get_orientation(self):
        return np.array(p.getBasePositionAndOrientation(self.pybullet_id)[1])