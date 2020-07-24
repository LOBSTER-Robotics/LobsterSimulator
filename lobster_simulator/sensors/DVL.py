from __future__ import annotations

import math
import numpy as np
from typing import List, TYPE_CHECKING

from lobster_simulator.common.Calculations import *

if TYPE_CHECKING:
    from lobster_simulator.robot.AUV import AUV
from lobster_simulator.sensors.Sensor import Sensor
from lobster_simulator.simulation_time import *
from lobster_simulator.tools.Constants import *
from lobster_simulator.tools.DebugVisualization import DebugLine
from lobster_simulator.tools.Translation import *

SEAFLOOR_DEPTH = 100  # meters

MINIMUM_ALTITUDE = 0.05  # meters
MAXIMUM_ALTITUDE = 50  # meters

MINIMUM_TIME_STEP = SimulationTime(int(seconds_to_microseconds(1 / 26)))
MAXIMUM_TIME_STEP = SimulationTime(int(seconds_to_microseconds(1 / 4)))

RED = [1, 0, 0]
GREEN = [0, 1, 0]


class DVL(Sensor):

    def __init__(self, robot: AUV, position: Vec3, orientation: Quaternion, time_step: SimulationTime):
        super().__init__(robot, position, orientation, time_step)

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
                                          parentIndex=self._robot._id) for i in range(4)]

    # The dvl doesn't use the base sensor update method, because it has a variable frequency which is not supported.
    def update(self, time: SimulationTime):

        altitudes = list()

        dt = time - self._previous_update_time
        current_distance_to_seafloor, current_velocity  = self._get_real_values(dt)

        for i in range(4):
            # The raytest endpoint is twice as far as the range of the dvl, because this makes it possible to
            # smoothly interpolate between the transition between not having a lock and having a lock
            raytest_endpoint = 2 * self.beam_end_points[i]

            auv_frame_endpoint = vec3_local_to_world(self._sensor_position, self._sensor_orientation,
                                                     raytest_endpoint)
            world_frame_endpoint = vec3_local_to_world(self._robot.get_position(), self._robot.get_orientation(),
                                                       auv_frame_endpoint)

            result = PybulletAPI.rayTest(self.get_position(), world_frame_endpoint)

            altitudes.append(result[0] * 100)

            # Change the color of the beam visualizer only if the state of the lock changes.
            if (self._previous_altitudes[i] >= MAXIMUM_ALTITUDE) != (altitudes[i] >= MAXIMUM_ALTITUDE):
                color = RED if altitudes[i] >= MAXIMUM_ALTITUDE else GREEN

                self.beamVisualizers[i].update(self._sensor_position, self.beam_end_points[i], color=color,
                                               frame_id=self._robot._id)

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

            # TODO: check if the DVL indeed gives the altitude as the average of the 4 altitudes
            average_interpolated_altitude = float(np.mean(interpolated_altitudes))

            interpolated_bottom_lock = all(i < MAXIMUM_ALTITUDE for i in interpolated_altitudes)

            self._queue.append(
                {
                    'time': self._time_step.milliseconds,
                    'vx': interpolated_velocity[X],
                    'vy': interpolated_velocity[Y],
                    'vz': interpolated_velocity[Z],
                    # TODO check whether the dvl gives the altitude straight down or relative to its orientation
                    # 'altitude': average_interpolated_altitude,
                    'altitude': current_distance_to_seafloor,
                    'velocity_valid': interpolated_bottom_lock,
                    "format": "json_v1"
                }
            )

            # The timestep of the DVL depends on the altitude (higher altitude is lower frequency)
            time_step_micros = interpolate(average_interpolated_altitude,
                                           MINIMUM_ALTITUDE, MAXIMUM_ALTITUDE,
                                           MINIMUM_TIME_STEP.microseconds, MAXIMUM_TIME_STEP.microseconds)

            time_step_micros = clip(time_step_micros, MINIMUM_TIME_STEP.microseconds, MAXIMUM_TIME_STEP.microseconds)

            self._time_step = SimulationTime(int(time_step_micros))

            self._next_sample_time += self._time_step

        self._previous_update_time = SimulationTime(time.microseconds)
        self._previous_altitudes = altitudes
        self._previous_velocity = current_velocity

    def get_position(self):
        return vec3_local_to_world(self._robot.get_position(), self._robot.get_orientation(), self._sensor_position)

    def _get_real_values(self, dt: SimulationTime) -> List:
        location = self._robot.get_position()

        distance_to_seafloor = SEAFLOOR_DEPTH - location[Z]

        velocity = self._robot.get_velocity()

        return [distance_to_seafloor, velocity]
