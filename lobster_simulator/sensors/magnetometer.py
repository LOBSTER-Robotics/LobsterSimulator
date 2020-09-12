from __future__ import annotations

from typing import List, TYPE_CHECKING, Union

if TYPE_CHECKING:
    from lobster_simulator.robot.auv import AUV

from lobster_simulator.sensors.sensor import Sensor
from lobster_simulator.common.simulation_time import SimulationTime
from lobster_simulator.common.translation import *
from lobster_common.constants import MAGNETIC_FIELD

class Magnetometer(Sensor):

    def __init__(self, robot: AUV, position: Vec3, time_step: SimulationTime, time: SimulationTime, orientation: Quaternion = None,
                 noise_stds: Union[List[float], float] = None):
        super().__init__(robot, position=position, orientation=orientation, time_step=time_step, noise_stds=noise_stds, time=time)

    def _get_real_values(self, dt: SimulationTime):
        robot_orientation = self._robot.get_orientation()

        # Rotates the magnetic field in the frame of the robot
        magnetic_field_in_robot_frame = vec3_rotate_vector_to_local(robot_orientation, MAGNETIC_FIELD)

        # Rotates the magnetic field in the frame of the magnetometer
        magnetometer_value = vec3_rotate_vector_to_local(self._sensor_orientation, magnetic_field_in_robot_frame)

        return [magnetometer_value]

    def get_magnetometer_value(self):
        return self._previous_real_value[0]
