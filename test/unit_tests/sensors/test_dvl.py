import unittest

from lobster_common.constants import *

from lobster_simulator.common.pybullet_api import PybulletAPI
from lobster_simulator.common.simulation_time import SimulationTime
from lobster_simulator.simulator import Simulator
from lobster_common.vec3 import Vec3
from lobster_simulator.sensors.dvl import SEAFLOOR_DEPTH


class DVLTest(unittest.TestCase):

    def test_altitude(self):
        dt = SimulationTime(4000)
        simulator = Simulator(4000, gui=False)
        # Load a plane so that the DVL works
        PybulletAPI.loadURDF("plane.urdf", Vec3([0, 0, 30]))
        simulator.create_robot()
        simulator.do_step()
        previous_altitude = simulator.robot._dvl._get_real_values(dt)[0]

        downwards_velocity = Vec3([0, 0, 1])

        amount_sensor_updates = 0
        for _ in range(500):
            simulator.do_step()
            simulator.robot.set_velocity(linear_velocity=downwards_velocity)

            actual_altitude = simulator.robot._dvl._get_real_values(dt)[0]
            sensor_data = simulator.robot._dvl.get_last_value()

            # Since the dvl runs at a slower rate than the simulator, it's possible that there is no new data point
            if sensor_data is not None:
                amount_sensor_updates += 1
                sensor_altitude = sensor_data[1]['altitude']

                min_altitude = min((actual_altitude, previous_altitude))
                max_altitude = max((actual_altitude, previous_altitude))

                # Make sure the sensor altitude is between the actual altitudes, since it is interpolated between the
                # two
                self.assertLessEqual(min_altitude, sensor_altitude)
                self.assertLessEqual(sensor_altitude, max_altitude)

            previous_altitude = actual_altitude

        # If there weren't at least 10 sensor updates the there went something wrong with the test.
        self.assertGreater(amount_sensor_updates, 10)

    def test_velocity(self):
        simulator = Simulator(4000, gui=False)
        simulator.create_robot()
        simulator.robot.set_velocity(linear_velocity=Vec3([1, 1, 1]))

        previous_velocity = simulator.robot.get_velocity()
        simulator.do_step()

        amount_sensor_updates = 0
        for _ in range(500):
            actual_velocity = simulator.robot.get_velocity()
            simulator.do_step()

            sensor_data = simulator.robot._dvl.get_last_value()

            # Since the dvl runs
            if sensor_data:
                amount_sensor_updates += 1
                vx, vy, vz = sensor_data[1]['vx'], sensor_data[1]['vy'], sensor_data[1]['vz']

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

        # If there weren't at least 10 sensor updates the there went something wrong with the test.
        self.assertGreater(amount_sensor_updates, 10)