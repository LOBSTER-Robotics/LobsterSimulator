from typing import List, Dict

from pkg_resources import resource_filename

from lobster_simulator.common.Gamepad import Gamepad
from lobster_simulator.robot.AUV import AUV
from lobster_simulator.tools.DebugVisualization import DebugLine
from .PID import PID
from lobster_simulator.tools import Translation
from lobster_simulator.tools.Constants import *
from lobster_simulator.tools.Translation import *

import numpy as np

import inputs

import pybullet as p

class HighLevelController:
    """
    This class is used for the control of the robot when the simulator is being run from this project for testing
    purposes.
    """

    motor_rpm_outputs: List[int] = [0, 0, 0, 0, 0, 0, 0, 0]

    orientation_pids = [
        PID(p=20, i=10, d=0, min_value=-100, max_value=100),
        PID(p=8, i=0, d=0, min_value=-10, max_value=10),  # Not used
        PID(p=20, i=10, d=0, min_value=-100, max_value=100)
    ]

    rate_pids = [
        PID(p=500, i=100, d=400, min_value=-4000, max_value=4000),  # PITCH
        PID(p=500, i=100, d=400, min_value=-4000, max_value=4000),  # ROLL
        PID(p=500, i=100, d=400, min_value=-4000, max_value=4000)  # YAW
    ]

    position_pids = [
        PID(p=2, i=2, d=5, min_value=-3, max_value=3),  # X
        PID(p=2, i=2, d=5, min_value=-3, max_value=3),  # Y
        PID(p=2, i=2, d=5, min_value=-3, max_value=3)  # Z
    ]

    velocity_pids = [
        PID(p=2000, i=50, d=1000, min_value=-3700, max_value=3900),  # X
        PID(p=1000, i=50, d=1000, min_value=-3700, max_value=3900),  # Y
        PID(p=1000, i=50, d=1000, min_value=-3700, max_value=3900)  # Z
    ]

    # acceleration_pids = [
    #     PID(p=2000, i=0, d=10000, min_value=-3700, max_value=3900),  # X
    #     PID(p=2000, i=0, d=10000, min_value=-3700, max_value=3900),  # Y
    #     PID(p=2000, i=0, d=10000, min_value=-3700, max_value=3900)   # Z
    # ]

    forward_thrust_pid = PID(p=0.1, i=0.4, d=0, min_value=-1, max_value=1)

    def __init__(self, gui, desired_position, desired_orientation, position_control=True):
        self.desired_position: Vec3 = desired_position
        self.desired_orientation: Vec3 = desired_orientation

        self.relative_yaw = 0
        self.relative_pitch = 0
        self.relative_roll = 0
        self.target_rates = [0, 0, 0]
        self.desired_pitch_yaw_vector = [0, 0, 0]
        self.rates = [0, 0, 0]
        self.gui = gui
        self.previous_velocity = Vec3([0, 0, 0])

        self.position_control = position_control

        self.desired_velocity = Vec3([0, 0, 0])
        self.desired_rates = Vec3([0, 0, 0])

        if gui:
            self.forward_velocity_slider = PybulletAPI.addUserDebugParameter("forward", -3, 3, 0)
            self.upward_velocity_slider = PybulletAPI.addUserDebugParameter("upward", -3, 3, 0)
            self.sideward_velocity_slider = PybulletAPI.addUserDebugParameter("sideward", -3, 3, 0)

        if self.position_control:
            self.visualisation = PybulletAPI.loadURDF(resource_filename("lobster_simulator",
                                                                        "data/scout-alpha-visual.urdf"), Vec3([0, 0, 0]))

        self.gamepad = Gamepad()

    def set_target_rate(self, direction, target):
        self.target_rates[direction] = target

    @staticmethod
    def key_is_down(key: str, keyboard_events: Dict):
        return keyboard_events.get(ord(key)) == PybulletAPI.KEY_IS_DOWN

    def _update_desired(self, orientation):
        keyboard_events = PybulletAPI.getKeyboardEvents()

        if self.position_control:
            desired_position = Translation.vec3_rotate_vector_to_local(orientation, self.desired_position)
            if self.key_is_down('q', keyboard_events):
                desired_position[Z] -= 0.004

            if self.key_is_down('e', keyboard_events):
                desired_position[Z] += 0.004
            if self.key_is_down('w', keyboard_events):
                desired_position[X] += 0.004
            if self.key_is_down('s', keyboard_events):
                desired_position[X] -= 0.004
            if self.key_is_down('a', keyboard_events):
                desired_position[Y] -= 0.004
            if self.key_is_down('d', keyboard_events):
                desired_position[Y] += 0.004

            desired_position[X] += self.gamepad.y / 200
            desired_position[Y] += self.gamepad.x / 200
            desired_position[Z] += self.gamepad.z / 200 - self.gamepad.rz / 200

            self.desired_position = Translation.vec3_rotate_vector_to_world(orientation, desired_position)

            if self.key_is_down('j', keyboard_events):
                self.desired_orientation[Z] -= 0.003
            if self.key_is_down('l', keyboard_events):
                self.desired_orientation[Z] += 0.003
            if self.key_is_down('u', keyboard_events):
                self.desired_orientation[X] -= 0.003
            if self.key_is_down('o', keyboard_events):
                self.desired_orientation[X] += 0.003
            if self.key_is_down('i', keyboard_events):
                self.desired_orientation[Y] -= 0.003
            if self.key_is_down('k', keyboard_events):
                self.desired_orientation[Y] += 0.003

            self.desired_orientation[Y] -= self.gamepad.ry / 200
            self.desired_orientation[Z] += self.gamepad.rx / 200
        else:
            self.desired_velocity[X] = self.gamepad.y * 3
            self.desired_velocity[Y] = self.gamepad.x * 3
            self.desired_velocity[Z] = self.gamepad.z * 3 - self.gamepad.rz * 3

            self.desired_rates[Y] = self.gamepad.ry
            self.desired_rates[Z] = self.gamepad.rx

    def update(self, position: Vec3, orientation: Quaternion, velocity: Vec3, angular_velocity: Vec3, dt):

        self._update_desired(orientation=orientation)

        if self.position_control:
            PybulletAPI.resetBasePositionAndOrientation(self.visualisation, self.desired_position,
                                                        PybulletAPI.getQuaternionFromEuler(self.desired_orientation))

        #
        # Position
        #
        if self.position_control:
            local_frame_desired_location = Translation.vec3_rotate_vector_to_local(orientation, self.desired_position)
            local_frame_location = Translation.vec3_rotate_vector_to_local(orientation, position)

            self.position_pids[X].set_target(local_frame_desired_location[X])
            self.position_pids[Y].set_target(local_frame_desired_location[Y])
            self.position_pids[Z].set_target(local_frame_desired_location[Z])

            self.position_pids[X].update(local_frame_location[X], dt)
            self.position_pids[Y].update(local_frame_location[Y], dt)
            self.position_pids[Z].update(local_frame_location[Z], dt)

            self.desired_velocity = Vec3([self.position_pids[X].output,
                                          -self.position_pids[Y].output,
                                          -self.position_pids[Z].output])

        self.velocity_pids[X].set_target(self.desired_velocity[X])
        self.velocity_pids[Y].set_target(self.desired_velocity[Y])
        self.velocity_pids[Z].set_target(self.desired_velocity[Z])

        local_frame_velocity = Translation.vec3_rotate_vector_to_local(orientation, velocity)
        self.velocity_pids[X].update(local_frame_velocity[X], dt)
        self.velocity_pids[Y].update(local_frame_velocity[Y], dt)
        self.velocity_pids[Z].update(local_frame_velocity[Z], dt)

        self.previous_velocity = Vec3(local_frame_velocity.array.copy())

        for i in range(4):
            self.motor_rpm_outputs[i] = self.velocity_pids[X].output

        for i in range(4, 6):
            self.motor_rpm_outputs[i] = -self.velocity_pids[Y].output

        for i in range(6, 8):
            self.motor_rpm_outputs[i] = -self.velocity_pids[Z].output

        #
        # Orientation
        #
        # if self.position_control:
        self.desired_pitch_yaw_vector = Vec3([1, 0, 0]).rotate(PybulletAPI.getQuaternionFromEuler(
            self.desired_orientation)).rotate_inverse(orientation)

        self.relative_pitch = np.arctan2(self.desired_pitch_yaw_vector[1], self.desired_pitch_yaw_vector[0])
        self.relative_yaw = -np.arctan2(self.desired_pitch_yaw_vector[2], self.desired_pitch_yaw_vector[0])

        self.relative_roll = -PybulletAPI.getEulerFromQuaternion(quaternion=orientation)[0]

        self.orientation_pids[PITCH].update(self.relative_pitch, dt)
        self.orientation_pids[YAW].update(self.relative_yaw, dt)
        self.orientation_pids[ROLL].update(self.relative_roll, dt)

        self.desired_rates = Vec3([self.orientation_pids[PITCH].output,
                                   self.orientation_pids[YAW].output,
                                   self.orientation_pids[ROLL].output])

        self.target_rates[PITCH] = self.desired_rates[PITCH]
        self.target_rates[YAW] = self.desired_rates[YAW]
        self.target_rates[ROLL] = self.desired_rates[ROLL]

        for i in range(3):
            self.rate_pids[i].set_target(self.target_rates[i])

        for i in range(3):
            self.rate_pids[i].update(self.rates[i], dt)

        # Translate world frame angular velocities to local frame angular velocities
        local_rotation = vec3_rotate_vector_to_local(orientation, angular_velocity)

        roll_rate, pitch_rate, yaw_rate = local_rotation

        self.rates = [pitch_rate, roll_rate, yaw_rate]

        self.motor_rpm_outputs[0] -= self.rate_pids[YAW].output
        self.motor_rpm_outputs[1] += self.rate_pids[YAW].output

        self.motor_rpm_outputs[2] += self.rate_pids[PITCH].output
        self.motor_rpm_outputs[3] -= self.rate_pids[PITCH].output

        self.motor_rpm_outputs[4] += self.rate_pids[ROLL].output
        self.motor_rpm_outputs[5] -= self.rate_pids[ROLL].output
