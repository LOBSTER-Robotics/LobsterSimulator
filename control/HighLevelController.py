from typing import List, Dict

from pkg_resources import resource_filename

from control.Gamepad import Gamepad
from lobster_simulator.common.PybulletAPI import PybulletAPI
from .PID import PID
from lobster_simulator.common import Translation
from lobster_common.constants import *
from lobster_simulator.common.Translation import *

import numpy as np


class HighLevelController:
    """
    This class is used for the control of the robot when the simulator is being run from this project for testing
    purposes.
    """

    motor_thrust_outputs: List[int] = [0, 0, 0, 0, 0, 0, 0, 0]

    orientation_pids = [
        PID(p=2, i=0, d=0, min_value=-100, max_value=100),  # PITCH
        PID(p=2, i=0, d=0, min_value=-10, max_value=10),    # ROLL
        PID(p=2, i=0, d=0, min_value=-100, max_value=100)   # YAW
    ]

    rate_pids = [
        PID(p=100000/80, i=0, d=0, min_value=-(40.2073 + 51.48491), max_value=40.2073 + 51.48491),  # PITCH
        PID(p=100000/80, i=0, d=0, min_value=-(40.2073 + 51.48491), max_value=40.2073 + 51.48491),  # ROLL
        PID(p=100000/80, i=0, d=0, min_value=-(40.2073 + 51.48491), max_value=40.2073 + 51.48491)   # YAW
    ]

    position_pids = [
        PID(p=2, i=0, d=1, min_value=-10, max_value=10),  # X
        PID(p=2, i=0, d=1, min_value=-10, max_value=10),  # Y
        PID(p=2, i=0, d=1, min_value=-10, max_value=10, windup_guard=1)  # Z
    ]

    velocity_pids = [
        PID(p=20000/80, i=0, d=0, min_value=-40.2073, max_value=51.48491),  # X
        PID(p=20000/80, i=0, d=0, min_value=-40.2073, max_value=51.48491),  # Y
        PID(p=1000/80, i=0, d=0,  min_value=-40.2073, max_value=51.48491)   # Z
    ]

    forward_thrust_pid = PID(p=0.1, i=0.4, d=0, min_value=-1, max_value=1)

    def __init__(self, gui: bool, desired_position: Vec3, desired_orientation: Vec3, position_control: bool = True):
        self.desired_position = desired_position
        self.desired_orientation = desired_orientation

        self.relative_yaw = 0
        self.relative_pitch = 0
        self.relative_roll = 0
        self.desired_pitch_yaw_vector = [0, 0, 0]
        self.gui = gui
        self.previous_velocity = Vec3([0, 0, 0])

        self.position_control = position_control

        self.desired_velocity = Vec3([0.0, 0.0, 0.0])
        self.desired_rates = Vec3([0.0, 0.0, 0.0])

        if gui:
            self.forward_velocity_slider = PybulletAPI.addUserDebugParameter("forward", -3, 3, 0)
            self.upward_velocity_slider = PybulletAPI.addUserDebugParameter("upward", -3, 3, 0)
            self.sideward_velocity_slider = PybulletAPI.addUserDebugParameter("sideward", -3, 3, 0)

        if self.position_control:
            self.visualisation = PybulletAPI.loadURDF(resource_filename("lobster_simulator",
                                                                        "data/scout-alpha-visual.urdf"), Vec3([0, 0, 0]))

        self.gamepad = Gamepad()
        self.gamepad.start()

    def set_target_rate(self, direction, target):
        self.desired_rates[direction] = target

    @staticmethod
    def key_is_down(key: str, keyboard_events: Dict):
        return keyboard_events.get(ord(key)) == PybulletAPI.KEY_IS_DOWN

    def _update_desired(self, orientation):
        keyboard_events = PybulletAPI.getKeyboardEvents()

        if self.position_control:
            desired_position = Translation.vec3_rotate_vector_to_local(orientation, self.desired_position)
            if self.key_is_down('q', keyboard_events):
                desired_position[Z] -= 0.008
            if self.key_is_down('e', keyboard_events):
                desired_position[Z] += 0.008
            if self.key_is_down('w', keyboard_events):
                desired_position[X] += 0.008
            if self.key_is_down('s', keyboard_events):
                desired_position[X] -= 0.008
            if self.key_is_down('a', keyboard_events):
                desired_position[Y] -= 0.008
            if self.key_is_down('d', keyboard_events):
                desired_position[Y] += 0.008

            desired_position[X] += self.gamepad.y / 40
            desired_position[Y] += self.gamepad.x / 40
            desired_position[Z] += self.gamepad.z / 40 - self.gamepad.rz / 40

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
            self.desired_velocity = Vec3([0, 0, 0])
            if self.key_is_down('q', keyboard_events):
                self.desired_velocity[Z] -= 10
            if self.key_is_down('e', keyboard_events):
                self.desired_velocity[Z] += 10
            if self.key_is_down('w', keyboard_events):
                self.desired_velocity[X] += 10
            if self.key_is_down('s', keyboard_events):
                self.desired_velocity[X] -= 10
            if self.key_is_down('a', keyboard_events):
                self.desired_velocity[Y] -= 10
            if self.key_is_down('d', keyboard_events):
                self.desired_velocity[Y] += 10

            self.desired_velocity[X] += self.gamepad.y * 10
            self.desired_velocity[Y] += self.gamepad.x * 10
            # self.desired_velocity[Z] = self.gamepad.z * 3 - self.gamepad.rz * 3

            self.desired_rates = Vec3([0, 0, 0])
            if self.key_is_down('j', keyboard_events):
                self.desired_rates[YAW] += 2
            if self.key_is_down('l', keyboard_events):
                self.desired_rates[YAW] -= 2
            if self.key_is_down('u', keyboard_events):
                self.desired_rates[ROLL] += 2
            if self.key_is_down('o', keyboard_events):
                self.desired_rates[ROLL] -= 2
            if self.key_is_down('i', keyboard_events):
                self.desired_rates[PITCH] += 2
            if self.key_is_down('k', keyboard_events):
                self.desired_rates[PITCH] -= 2

            self.desired_rates[YAW] -= self.gamepad.rx * 2.0
            self.desired_rates[PITCH] += self.gamepad.ry * 2.0
            self.desired_rates[ROLL] += self.gamepad.z * 2.0 - self.gamepad.rz * 2.0

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
                                          self.position_pids[Y].output,
                                          self.position_pids[Z].output])

        self.velocity_pids[X].set_target(self.desired_velocity[X])
        self.velocity_pids[Y].set_target(self.desired_velocity[Y])
        self.velocity_pids[Z].set_target(self.desired_velocity[Z])

        local_frame_velocity = Translation.vec3_rotate_vector_to_local(orientation, velocity)
        self.velocity_pids[X].update(local_frame_velocity[X], dt)
        self.velocity_pids[Y].update(local_frame_velocity[Y], dt)
        self.velocity_pids[Z].update(local_frame_velocity[Z], dt)

        self.previous_velocity = Vec3(local_frame_velocity.numpy().copy())

        for i in range(4):
            self.motor_thrust_outputs[i] = self.velocity_pids[X].output

        for i in range(4, 6):
            self.motor_thrust_outputs[i] = self.velocity_pids[Y].output

        for i in range(6, 8):
            self.motor_thrust_outputs[i] = self.velocity_pids[Z].output

        #
        # Orientation
        #
        if self.position_control:
            self.desired_pitch_yaw_vector = Vec3([1, 0, 0]).rotate(PybulletAPI.getQuaternionFromEuler(
                self.desired_orientation)).rotate_inverse(orientation)

            self.relative_yaw = np.arctan2(self.desired_pitch_yaw_vector[1], self.desired_pitch_yaw_vector[0])
            self.relative_pitch = -np.arctan2(self.desired_pitch_yaw_vector[2], self.desired_pitch_yaw_vector[0])

            self.relative_roll = -PybulletAPI.getEulerFromQuaternion(quaternion=orientation)[0]

            self.orientation_pids[PITCH].update(self.relative_pitch, dt)
            self.orientation_pids[YAW].update(self.relative_yaw, dt)
            self.orientation_pids[ROLL].update(self.relative_roll, dt)

            self.desired_rates[PITCH] = self.orientation_pids[PITCH].output
            self.desired_rates[YAW]   = self.orientation_pids[YAW].output
            self.desired_rates[ROLL]  = self.orientation_pids[ROLL].output


        local_angular_velocity = vec3_rotate_vector_to_local(orientation, angular_velocity)
        roll_rate, pitch_rate, yaw_rate = local_angular_velocity
        rates = [roll_rate, pitch_rate, yaw_rate]

        self.rate_pids[PITCH].set_target(self.desired_rates[PITCH])
        self.rate_pids[YAW].set_target(self.desired_rates[YAW])
        self.rate_pids[ROLL].set_target(self.desired_rates[ROLL])

        self.rate_pids[ROLL].update(-rates[X], dt)
        self.rate_pids[PITCH].update(-rates[Y], dt)
        self.rate_pids[YAW].update(-rates[Z], dt)

        # Translate world frame angular velocities to local frame angular velocities
        self.motor_thrust_outputs[0] -= self.rate_pids[PITCH].output
        self.motor_thrust_outputs[1] += self.rate_pids[PITCH].output

        self.motor_thrust_outputs[2] += self.rate_pids[YAW].output
        self.motor_thrust_outputs[3] -= self.rate_pids[YAW].output

        self.motor_thrust_outputs[4] += self.rate_pids[ROLL].output
        self.motor_thrust_outputs[5] -= self.rate_pids[ROLL].output
        self.motor_thrust_outputs[6] -= self.rate_pids[ROLL].output
        self.motor_thrust_outputs[7] += self.rate_pids[ROLL].output
