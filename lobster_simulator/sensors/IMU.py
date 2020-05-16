import numpy as np
import pybullet as p

from lobster_simulator.sensors.Sensor import Sensor
from lobster_simulator.simulation_time import SimulationTime

ACCELERATION = 0


class IMU(Sensor):

    def __init__(self, pybullet_id, position, orientation, time_step):
        self.previous_velocity = np.array([0, 0, 0])
        self.previous_real_value = (np.array([0, 0, 0]))
        super().__init__(pybullet_id, position, orientation, time_step)

    def update(self, time: SimulationTime):
        super().update(time)
        self.previous_velocity = np.array(p.getBaseVelocity(self.pybullet_id)[0])

    def _get_real_values(self, dt: SimulationTime):
        velocity = np.array(p.getBaseVelocity(self.pybullet_id)[0])
        acceleration = (velocity - self.previous_velocity) * 1000000 / dt.microseconds
        return acceleration
