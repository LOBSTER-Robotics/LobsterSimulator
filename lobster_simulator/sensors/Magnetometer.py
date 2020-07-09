from __future__ import annotations

from typing import List, TYPE_CHECKING

if TYPE_CHECKING:
    from lobster_simulator.robot.AUV import UUV

from lobster_simulator.sensors.Sensor import Sensor
from lobster_simulator.simulation_time import SimulationTime
from lobster_simulator.tools.Translation import *

MAGNETIC_FIELD = [1, 0, 0]


class Magnetometer(Sensor):

    def __init__(self, robot: UUV, position: np.array, orientation: np.array, time_step: SimulationTime):
        super().__init__(robot, position, orientation, time_step)

    def _get_real_values(self, dt: SimulationTime):
        robot_orientation = self._robot.get_orientation()

        # Rotates the magnetic field in the frame of the robot
        magnetic_field_in_robot_frame = vec3_rotate_vector_to_local(robot_orientation, MAGNETIC_FIELD)

        # Rotates the magnetic field in the frame of the magnetometer
        magnetometer_value = vec3_rotate_vector_to_local(self._sensor_orientation, magnetic_field_in_robot_frame)

        return [magnetometer_value]

    def get_magnetometer_value(self):
        return self._previous_real_value[0]
