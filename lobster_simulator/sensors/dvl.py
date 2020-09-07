from __future__ import annotations

import math
import numpy as np
from typing import List, TYPE_CHECKING, Tuple, Optional

from lobster_simulator.common.calculations import *
from lobster_simulator.common.pybullet_api import PybulletAPI

if TYPE_CHECKING:
    from lobster_simulator.robot.auv import AUV
from lobster_simulator.sensors.sensor import Sensor
from lobster_simulator.common.simulation_time import *
from lobster_common.constants import *
from lobster_simulator.common.debug_visualization import DebugLine
from lobster_simulator.common.translation import *

MINIMUM_ALTITUDE = 0.05  # meters
MAXIMUM_ALTITUDE = 50  # meters

MINIMUM_TIME_STEP = SimulationTime(int(seconds_to_microseconds(1 / 26)))
MAXIMUM_TIME_STEP = SimulationTime(int(seconds_to_microseconds(1 / 4)))

RED = [1, 0, 0]
GREEN = [0, 1, 0]


class DVL(Sensor):

    def __init__(self, robot: AUV, position: Vec3, time_step: SimulationTime, orientation: Quaternion = None):
        super().__init__(robot, position=position, time_step=time_step, orientation=orientation, noise_stds=None)

        self._previous_altitudes = [2 * MAXIMUM_ALTITUDE, 2 * MAXIMUM_ALTITUDE, 2 * MAXIMUM_ALTITUDE,
                                    2 * MAXIMUM_ALTITUDE]
        self._previous_velocity = Vec3([0, 0, 0])

        angle = math.radians(22.5)
        beam_offset = 50 * math.tan(angle)
        self.beam_end_points = [
            Vec3([0, beam_offset, 50]),
            Vec3([0, -beam_offset, 50]),
            Vec3([beam_offset, 0, 50]),
            Vec3([-beam_offset, 0, 50])
        ]

        self.beamVisualizers = [DebugLine(self._sensor_position, self.beam_end_points[i], color=[1, 0, 0], width=2,
                                          parentIndex=self._robot.object_id) for i in range(4)]

    # The dvl doesn't use the base sensor update method, because it has a variable frequency which is not supported.
    def update(self, time: SimulationTime, dt: SimulationTime) -> None:

        altitudes = list()

        actual_altitude, current_velocity = self._get_real_values(dt)


        for i in range(4):
            # The raytest endpoint is twice as far as the range of the dvl, because this makes it possible to
            # smoothly interpolate between the transition between not having a lock and having a lock
            raytest_endpoint = 2 * self.beam_end_points[i]

            auv_frame_endpoint = vec3_local_to_world(self._sensor_position, self._sensor_orientation,
                                                     raytest_endpoint)
            world_frame_endpoint = vec3_local_to_world(self._robot.get_position(), self._robot.get_orientation(),
                                                       auv_frame_endpoint)

            result = PybulletAPI.rayTest(self._get_position(), world_frame_endpoint)

            altitudes.append(result[0] * 2 * MAXIMUM_ALTITUDE)

            # Change the color of the beam visualizer only if the state of the lock changes.
            if (self._previous_altitudes[i] >= MAXIMUM_ALTITUDE) != (altitudes[i] >= MAXIMUM_ALTITUDE):
                color = RED if altitudes[i] >= MAXIMUM_ALTITUDE else GREEN

                self.beamVisualizers[i].update(self._sensor_position, self.beam_end_points[i], color=color,
                                               frame_id=self._robot.object_id)

        self._queue = list()

        while self._next_sample_time <= time:
            interpolated_altitudes = list()
            for i in range(4):
                interpolated_altitudes.append(interpolate(x=self._next_sample_time.microseconds,
                                                          x1=self._previous_update_time.microseconds,
                                                          x2=time.microseconds,
                                                          y1=self._previous_altitudes[i],
                                                          y2=altitudes[i]))

            interpolated_velocity = interpolate_vec(x=self._next_sample_time.microseconds,
                                                    x1=self._previous_update_time.microseconds,
                                                    x2=time.microseconds,
                                                    y1=self._previous_velocity,
                                                    y2=current_velocity)

            interpolated_bottom_lock = all(i < MAXIMUM_ALTITUDE for i in interpolated_altitudes)

            self._queue.append(
                (
                    self._next_sample_time.seconds,
                    {
                        'time': self._time_step.milliseconds,
                        'vx': interpolated_velocity[X],
                        'vy': interpolated_velocity[Y],
                        'vz': interpolated_velocity[Z],
                        'altitude': actual_altitude,
                        'velocity_valid': interpolated_bottom_lock,
                        "format": "json_v1"
                    }
                )
            )

            # The timestep of the DVL depends on the altitude (higher altitude is lower frequency)
            time_step_micros = interpolate(actual_altitude,
                                           MINIMUM_ALTITUDE, MAXIMUM_ALTITUDE,
                                           MINIMUM_TIME_STEP.microseconds, MAXIMUM_TIME_STEP.microseconds)

            time_step_micros = clip(time_step_micros, MINIMUM_TIME_STEP.microseconds, MAXIMUM_TIME_STEP.microseconds)

            self._time_step = SimulationTime(int(time_step_micros))

            self._next_sample_time += self._time_step

        self._previous_update_time = SimulationTime(time.microseconds)
        self._previous_altitudes = altitudes
        self._previous_velocity = current_velocity

    def _get_position(self):
        """Returns the position of the DVL in the world frame."""
        return vec3_local_to_world(self._robot.get_position(), self._robot.get_orientation(), self._sensor_position)

    def _get_real_values(self, dt: SimulationTime) -> List:

        robot_altitude = self._robot.get_altitude()
        if robot_altitude is not None:
            altitude = robot_altitude - self._sensor_position[Z]
        else:
            altitude = None

        velocity = self._robot.get_velocity()

        return [altitude, velocity]

    def get_altitude(self) -> Optional[Tuple[float, float]]:
        """
        Gives the latest altitude as an average of the 4 altitudes meassured by the 4 beams with a timestamp.
        :return: Tuple where the first value is the time in seconds and the second value is the altitude
        """
        last_value = self.get_last_value()

        if last_value is None:
            return None

        return last_value[0], last_value[1]['altitude']

    def get_velocity(self) -> Optional[Tuple[float, Vec3]]:
        """
        Gives the latest velocity of the robot in the dvl sensor frame.
        :return: Tuple where the first value is the time in seconds and the second value is the velocity
        """
        last_value = self.get_last_value()

        if last_value is None:
            return None

        return last_value[0], Vec3([last_value[1]['vx'], last_value[1]['vy'], last_value[1]['vz']])

    def remove(self):
        for beam in self.beamVisualizers:
            beam.remove()
