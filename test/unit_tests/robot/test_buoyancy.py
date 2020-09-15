import json
import unittest
from unittest import mock
from unittest.mock import Mock, MagicMock

from lobster_common.constants import Z
from lobster_common.quaternion import Quaternion
from lobster_common.vec3 import Vec3
from pkg_resources import resource_stream

from lobster_simulator.common.pybullet_api import PybulletAPI
from lobster_simulator.common.simulation_time import SimulationTime
from lobster_simulator.robot import buoyancy, auv
from lobster_simulator.robot.buoyancy import NoTestPointsCreated


class BuoyancyTest(unittest.TestCase):

    def test_creating_test_points_scout_alpha(self):
        PybulletAPI.initialize(SimulationTime(1000), False)

        model_config = 'scout-alpha.json'
        with resource_stream('lobster_simulator', f'data/{model_config}') as f:
            self.robot_config = json.load(f)

        # Currently resolution is not existing?
        resolution = self.robot_config.get('buoyancy_resolution')

        robot = auv.AUV(SimulationTime(5000), self.robot_config)
        buoyancy_obj = buoyancy.Buoyancy(robot, 0.10, 2, resolution=resolution)
        self.assertGreater(len(buoyancy_obj.test_points), 0)

    def test_not_finding_test_points(self):
        with mock.patch("lobster_simulator.robot.buoyancy.Buoyancy._check_ray_hits_robot", return_value=False):
            with self.assertRaises(NoTestPointsCreated):
                buoyancy.Buoyancy(Mock(), 1, 1)

    def test_incorrect_radius_raises(self):
        with self.assertRaises(ValueError):
            buoyancy.Buoyancy(Mock(), 0, 1)

    def test_incorrect_length_raises(self):
        with self.assertRaises(ValueError):
            buoyancy.Buoyancy(Mock(), 1, 0)

    def test_finding_test_points_not_raising_error(self):
        with mock.patch("lobster_simulator.robot.buoyancy.Buoyancy._check_ray_hits_robot", return_value=True):
            buoyancy_obj = buoyancy.Buoyancy(Mock(), 1, 1)
        self.assertGreater(len(buoyancy_obj.test_points), 0)

    def test_advanced_buoyancy_is_calculated_when_robot_is_close_to_surface(self):
        z_pos = 0.9
        # Length should be twice as much as distance from robot position to sea surface such that buoyancy will be calculated.
        robot_length = (z_pos + 0.1) * 2
        robot_mock = MagicMock()
        with mock.patch("lobster_simulator.robot.buoyancy.Buoyancy._check_ray_hits_robot", return_value=True):
            buoyancy_obj = buoyancy.Buoyancy(robot_mock, 0.5, robot_length)

        robot_mock.apply_force = Mock()

        robot_mock.get_position_and_orientation = Mock(return_value=(Vec3((0, 0, z_pos)), Quaternion((0, 0, 0, 1))))

        buoyancy_obj.update()

        robot_mock.get_position_and_orientation.assert_called_once()

        # First and third argument could change in the future.
        robot_mock.apply_force.assert_called_once()
        self.assertEqual(buoyancy_obj._buoyancy, -robot_mock.apply_force.call_args[0][1][Z],
                         msg="buoyancy force is not as expected to be when robot is fully underwater.")
