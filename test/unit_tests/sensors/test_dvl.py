import unittest
from typing import List

from lobster_common.constants import *

from lobster_simulator.simulator import Simulator
from lobster_common.vec3 import Vec3
from lobster_simulator.sensors.dvl import SEAFLOOR_DEPTH


class DVLTest(unittest.TestCase):

    def test_altitude(self):
        simulator = Simulator(4000, gui=False)
        simulator.create_robot()
        previous_altitude = SEAFLOOR_DEPTH - simulator.robot._dvl.get_position()[2]

        for _ in range(100):
            simulator.do_step()
            actual_altitude = SEAFLOOR_DEPTH - simulator.robot._dvl.get_position()[2]
            sensor_data_list: List = simulator.robot._dvl.pop_all_values()

            for sensor_data in sensor_data_list:
                # Velocity_valid should always be true otherwise the dvl is too far away from the surface and this test wouldn't work.
                self.assertTrue(sensor_data['velocity_valid'])
                sensor_altitude = sensor_data['altitude']

                min_altitude = min((actual_altitude, previous_altitude))
                max_altitude = max((actual_altitude, previous_altitude))

                # Make sure the sensor altitude is between the actual altitudes, since it is interpolated between the
                # two
                self.assertLessEqual(min_altitude, sensor_altitude)
                self.assertLessEqual(sensor_altitude, max_altitude)

            previous_altitude = actual_altitude

    def test_velocity(self):
        simulator = Simulator(4000, gui=False)
        simulator.create_robot()

        previous_velocity = simulator.robot.get_velocity()
        simulator.robot.set_velocity(linear_velocity=Vec3([1, 1, 1]))
        simulator.do_step()

        for _ in range(100):
            actual_velocity = simulator.robot.get_velocity()
            simulator.do_step()
            sensor_data_list: List = simulator.robot._dvl.pop_all_values()

            # Since the dvl runs
            for sensor_data in sensor_data_list:
                # Velocity_valid should always be true otherwise the dvl is too far away from the surface and this test wouldn't work.
                self.assertTrue(sensor_data['velocity_valid'])
                vx, vy, vz = sensor_data['vx'], sensor_data['vy'], sensor_data['vz']

                min_vx = min((actual_velocity[X], previous_velocity[X]))
                max_vx = max((actual_velocity[X], previous_velocity[X]))

                min_vy = min((actual_velocity[Y], previous_velocity[Y]))
                max_vy = max((actual_velocity[Y], previous_velocity[Y]))

                min_vz = min((actual_velocity[Z], previous_velocity[Z]))
                max_vz = max((actual_velocity[Z], previous_velocity[Z]))

                # Make sure the sensor velocities are between the actual velocities, since it is interpolated between
                # the two
                self.assertLessEqual(min_vx, vx)
                self.assertLessEqual(min_vy, vy)
                self.assertLessEqual(min_vz, vz)
                self.assertLessEqual(vx, max_vx)
                self.assertLessEqual(vy, max_vy)
                self.assertLessEqual(vz, max_vz)

            previous_velocity = Vec3(actual_velocity)
