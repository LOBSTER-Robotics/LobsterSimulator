from typing import List

from pkg_resources import resource_filename

from lobster_simulator.robot.AUV import AUV
from lobster_simulator.tools.DebugVisualization import DebugLine
from .PID import PID
from lobster_simulator.tools import Translation
from lobster_simulator.tools.Constants import *
from lobster_simulator.tools.Translation import *

import numpy as np

import pybullet as p


class HighLevelController:
    """
    This class is used for the control of the robot when the simulator is being run from this project for testing
    purposes.
    """

    motor_rpm_outputs: List[int] = [0, 0, 0, 0, 0, 0, 0, 0]

    orientation_pids = [
        PID(p=20, i=0, d=0, min_value=-100, max_value=100),
        PID(p=8, i=0, d=0, min_value=-10, max_value=10),  # Not used
        PID(p=20, i=0, d=0, min_value=-100, max_value=100)
    ]

    rate_pids = [
        PID(p=500, i=0, d=400, min_value=-4000, max_value=4000),  # PITCH
        PID(p=500, i=0, d=400, min_value=-4000, max_value=4000),  # ROLL
        PID(p=500, i=0, d=400, min_value=-4000, max_value=4000)   # YAW
    ]

    forward_thrust_pid = PID(p=0.1, i=0.4, d=0, min_value=-1, max_value=1)

    def __init__(self, gui):
        self.relative_yaw = 0
        self.relative_pitch = 0
        self.relative_roll = 0
        self.target_rates = [0, 0, 0]
        self.desired_pitch_yaw_vector = [0, 0, 0]
        self.rates = [0, 0, 0]
        self.gui = gui

        if gui:
            self.forward_rpm_slider = PybulletAPI.addUserDebugParameter("forward rpm", -3700, 3900, 0)
            self.upward_rpm_slider = PybulletAPI.addUserDebugParameter("upward rpm", -3700, 3900, 0)
            self.sideward_rpm_slider = PybulletAPI.addUserDebugParameter("sideward rpm", -3700, 3900, 0)

        self.visualisation = PybulletAPI.loadURDF(resource_filename("lobster_simulator",
                                                                    "data/scout-alpha-visual.urdf"), Vec3([0, 0, 0]))
        print(self.visualisation)

    def set_target_rate(self, direction, target):
        self.target_rates[direction] = target

    def update(self, position: Vec3, orientation: Quaternion, velocity: Vec3, angular_velocity: Vec3,
               desired_location: Vec3, desired_orientation: Quaternion, dt):

        PybulletAPI.resetBasePositionAndOrientation(self.visualisation, position, desired_orientation)

        self.desired_pitch_yaw_vector = Translation.vec3_world_to_local(
            position,
            orientation,
            desired_location
        )

        self.desired_pitch_yaw_vector = Vec3([1, 0, 0]).rotate(desired_orientation).rotate_inverse(orientation)


        print(p.getAxisAngleFromQuaternion(desired_orientation.array))
        print(p.getAxisDifferenceQuaternion(orientation.array, desired_orientation.array))
        print(self.desired_pitch_yaw_vector)
        print()

        self.relative_pitch = np.arctan2(self.desired_pitch_yaw_vector[1], self.desired_pitch_yaw_vector[0])
        self.relative_yaw = -np.arctan2(self.desired_pitch_yaw_vector[2], self.desired_pitch_yaw_vector[0])

        self.relative_roll = -PybulletAPI.getEulerFromQuaternion(quaternion=orientation)[0]

        self.orientation_pids[PITCH].update(self.relative_pitch, dt)
        self.orientation_pids[YAW].update(self.relative_yaw, dt)
        self.orientation_pids[ROLL].update(self.relative_roll, dt)

        self.target_rates[PITCH] = self.orientation_pids[PITCH].output
        self.target_rates[YAW] = self.orientation_pids[YAW].output
        self.target_rates[ROLL] = self.orientation_pids[ROLL].output

        for i in range(3):
            self.rate_pids[i].set_target(self.target_rates[i])

        # Translate world frame angular velocities to local frame angular velocities
        local_rotation = vec3_rotate_vector_to_local(orientation, angular_velocity)

        roll_rate, pitch_rate, yaw_rate = local_rotation

        self.rates = [pitch_rate, roll_rate, yaw_rate]

        if self.gui:
            for i in range(3):
                self.rate_pids[i].update(self.rates[i], 1. / 240.)

            for i in range(4):
                self.motor_rpm_outputs[i] = int(PybulletAPI.readUserDebugParameter(self.forward_rpm_slider))

            for i in range(4, 6):
                self.motor_rpm_outputs[i] = int(PybulletAPI.readUserDebugParameter(self.upward_rpm_slider))

            for i in range(6, 8):
                self.motor_rpm_outputs[i] = int(PybulletAPI.readUserDebugParameter(self.sideward_rpm_slider))
        else:
            for i in range(8):
                self.motor_rpm_outputs[i] = 0

        self.motor_rpm_outputs[0] -= self.rate_pids[YAW].output
        self.motor_rpm_outputs[1] += self.rate_pids[YAW].output

        self.motor_rpm_outputs[2] += self.rate_pids[PITCH].output
        self.motor_rpm_outputs[3] -= self.rate_pids[PITCH].output

        self.motor_rpm_outputs[4] += self.rate_pids[ROLL].output
        self.motor_rpm_outputs[5] -= self.rate_pids[ROLL].output
