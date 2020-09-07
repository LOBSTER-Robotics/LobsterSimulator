import unittest
from unittest import mock
from unittest.mock import patch, MagicMock

from lobster_common.vec3 import Vec3

from lobster_simulator.common.debug_visualization import DebugLine
from lobster_simulator.common.pybullet_api import PybulletAPI
from lobster_simulator.common.simulation_time import SimulationTime


class TestDebugLine(unittest.TestCase):

    @mock.patch('time.time', mock.MagicMock(return_value=0))
    def setUp(self):
        PybulletAPI.initialize(SimulationTime(1000), False)
        PybulletAPI.gui = MagicMock(return_value=True)
        # Use side_effect to spy on a method: Can assert calls but functionality doesn't change.
        PybulletAPI.addUserDebugLine = MagicMock(side_effect=PybulletAPI.addUserDebugLine)
        self._debug_line = DebugLine(Vec3([0, 0, 0]), Vec3([0, 0, 0]))

    def test_create_debug_line_called(self):
        # Assert add debug line called in constructor
        PybulletAPI.addUserDebugLine.assert_called_once()

    @mock.patch('time.time', mock.MagicMock(return_value=0))
    def test_not_updating_too_frequent(self):
        self._debug_line.update(Vec3([0, 0, 0]), Vec3([0, 0, 0]))
        # Assert debug line is not called twice because cannot create line too frequent
        self.assertEqual(1, PybulletAPI.addUserDebugLine.call_count)

    def test_updating_when_time_has_passed(self):
        # Mocking time to return number bigger than update frequency
        with patch('time.time', mock.MagicMock(return_value=DebugLine._MIN_UPDATE_INTERVAL + 1E-6)):
            self._debug_line.update(Vec3([0, 0, 0]), Vec3([0, 0, 0]))
        # Assert debug line is not called twice because cannot create line too frequent
        self.assertEqual(2, PybulletAPI.addUserDebugLine.call_count)
