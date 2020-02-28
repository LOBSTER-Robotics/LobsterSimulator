import numpy as np
import pybullet as p

from .PID import PID
from lobster_simulator.tools import Translation
from lobster_simulator.tools.Constants import *


class HighLevelController:
    motor_rpm_outputs = [0, 0, 0, 0, 0, 0, 0, 0]

    orientation_pids = [
        PID(p=8, i=0, d=2, min_value=-10, max_value=10),
        PID(p=8, i=0, d=0, min_value=-10, max_value=10),  # Not used
        PID(p=8, i=0, d=2, min_value=-10, max_value=10)
    ]

    rate_pids = [
        PID(p=1000, i=0, d=10, min_value=-4000, max_value=4000),  # PITCH
        PID(p=1000, i=0, d=2, min_value=-4000, max_value=4000),  # ROLL
        PID(p=1000, i=0, d=0, min_value=-4000, max_value=4000)   # YAW
    ]

    forward_thrust_pid = PID(p=0.1, i=0.4, d=0, min_value=-1, max_value=1)

    def __init__(self):
        self.relative_yaw = 0
        self.relative_pitch = 0
        self.target_rates = [0, 0, 0]
        self.relative_desired_location = [0, 0, 0]
        self.rates = [0, 0, 0]

        self.forward_rpm_slider = p.addUserDebugParameter("forward rpm", -3700, 3900, 0)
        self.upward_rpm_slider = p.addUserDebugParameter("upward rpm", -3700, 3900, 0)
        self.sideward_rpm_slider = p.addUserDebugParameter("sideward rpm", -3700, 3900, 0)

    def set_target_rate(self, direction, target):
        self.target_rates[direction] = target

    def update(self, position, orientation, velocity, desired_location):
        self.relative_desired_location = Translation.vec3_world_to_local(
            position,
            orientation,
            desired_location
        )

        self.relative_pitch = np.arctan2(self.relative_desired_location[1], self.relative_desired_location[0])
        self.relative_yaw = np.arctan2(self.relative_desired_location[2], self.relative_desired_location[0])

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

        roll_rate, pitch_rate, yaw_rate = local_rotation

        self.rates = [pitch_rate, roll_rate, yaw_rate]

        # print([f'{i:.2f}' for i in [pitch_rate, yaw_rate, roll_rate]], end='')
        # print(self.target_rates[ROLL] - roll_rate)

        for i in range(3):
            self.rate_pids[i].update(self.rates[i], 1. / 240.)

        for i in range(4):
            self.motor_rpm_outputs[i] = p.readUserDebugParameter(self.forward_rpm_slider)

        for i in range(4, 6):
            self.motor_rpm_outputs[i] = p.readUserDebugParameter(self.upward_rpm_slider)

        for i in range(6, 8):
            self.motor_rpm_outputs[i] = p.readUserDebugParameter(self.sideward_rpm_slider)

        self.motor_rpm_outputs[0] -= self.rate_pids[YAW].output
        self.motor_rpm_outputs[1] += self.rate_pids[YAW].output

        self.motor_rpm_outputs[2] += self.rate_pids[PITCH].output
        self.motor_rpm_outputs[3] -= self.rate_pids[PITCH].output

        self.motor_rpm_outputs[4] += self.rate_pids[ROLL].output
        self.motor_rpm_outputs[5] -= self.rate_pids[ROLL].output



