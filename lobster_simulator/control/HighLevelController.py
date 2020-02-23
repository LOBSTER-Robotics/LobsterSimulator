import numpy as np
import pybullet as p

from .PID import PID
from lobster_simulator.tools import Translation
from lobster_simulator.tools.Constants import *


class HighLevelController:
    motor_outputs = [0, 0, 0, 0, 0, 0]

    orientation_pids = [
        PID(p=8, i=0, d=0, min_value=-10, max_value=10),
        PID(p=8, i=0, d=0, min_value=-10, max_value=10),  # Not used
        PID(p=8, i=0, d=0, min_value=-10, max_value=10)
    ]

    rate_pids = [
        PID(p=0.2, i=0.4, d=0, min_value=-1, max_value=1),  # PITCH
        PID(p=0.2, i=0.4, d=0, min_value=-1, max_value=1),  # ROLL
        PID(p=0.2, i=0.4, d=0, min_value=-1, max_value=1)   # YAW
    ]

    forward_thrust_pid = PID(p=0.1, i=0.4, d=0, min_value=-1, max_value=1)

    def __init__(self):
        self.relative_yaw = 0
        self.relative_pitch = 0
        self.target_rates = [0, 0, 0]
        self.relative_desired_location = [0, 0, 0]
        self.rates = [0, 0, 0]

    def set_target_rate(self, direction, target):
        self.target_rates[direction] = target

    def update(self, position, orientation, velocity, desired_location):
        self.relative_desired_location = Translation.vec3_world_to_local(
            position,
            orientation,
            desired_location
        )

        self.relative_pitch = np.arctan2(self.relative_desired_location[1], self.relative_desired_location[2])
        self.relative_yaw = np.arctan2(self.relative_desired_location[0], self.relative_desired_location[2])

        self.orientation_pids[PITCH].update(self.relative_pitch, 1. / 240.)
        self.orientation_pids[YAW].update(-self.relative_yaw, 1. / 240.)

        self.target_rates[PITCH] = self.orientation_pids[PITCH].output
        self.target_rates[YAW] = self.orientation_pids[YAW].output

        for i in range(3):
            self.rate_pids[i].set_target(self.target_rates[i])

        # Translate world frame angular velocities to local frame angular velocities
        local_rotation = np.dot(
            np.linalg.inv(np.reshape(np.array(p.getMatrixFromQuaternion(orientation)), (3, 3))),
            velocity[1])

        pitch_rate, yaw_rate, roll_rate = local_rotation

        self.rates = [pitch_rate, roll_rate, yaw_rate]

        # print([f'{i:.2f}' for i in [pitch_rate, yaw_rate, roll_rate]], end='')

        for i in range(3):
            self.rate_pids[i].update(self.rates[i], 1. / 240.)

        self.motor_outputs[0] = -self.rate_pids[YAW].output
        self.motor_outputs[1] = self.rate_pids[YAW].output

        self.motor_outputs[2] = self.rate_pids[PITCH].output
        self.motor_outputs[3] = -self.rate_pids[PITCH].output

        self.motor_outputs[4] = self.rate_pids[ROLL].output
        self.motor_outputs[5] = -self.rate_pids[ROLL].output
