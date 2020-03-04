import numpy as np
import pybullet as p

from lobster_simulator.sensors.Sensor import Sensor


class IMU(Sensor):

    def __init__(self, pybullet_id, position, orientation, frequency):
        super().__init__(pybullet_id, position, orientation, frequency)

        self.velocity = np.array([0, 0, 0])
        self.acceleration = np.array([0, 0, 0])

    def update(self, dt):
        self.acceleration = (np.array(p.getBaseVelocity(self.pybullet_id)[0]) - self.velocity) / dt
        self.velocity = np.array(p.getBaseVelocity(self.pybullet_id)[0])

    def get_sensor_position(self):
        return self.position

    def get_sensor_orientation(self):
        return self.orientation

    def get_orientation(self):
        return p.getBasePositionAndOrientation(self.pybullet_id)[1]

    def get_velocity(self):
        return self.velocity
