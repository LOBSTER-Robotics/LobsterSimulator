import numpy as np
import pybullet as p

from lobster_simulator.sensors.Sensor import Sensor

VELOCITY = 0
ACCELERATION = 1


class IMU(Sensor):

    def __init__(self, pybullet_id, position, orientation, time_step):
        super().__init__(pybullet_id, position, orientation, time_step)

        self.previous_velocity = np.array([0, 0, 0])
        self.previous_real_value = (np.array([0, 0, 0]), self.previous_velocity)

    def _get_real_values(self, dt: int):
        velocity = np.array(p.getBaseVelocity(self.pybullet_id)[0])
        acceleration = (velocity - self.previous_real_value[VELOCITY]) / dt
        return acceleration, velocity
