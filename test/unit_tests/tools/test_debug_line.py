import unittest
from unittest import mock
from unittest.mock import MagicMock, patch

from lobster_simulator.common.DebugVisualization import DebugLine


class TestDebugLine(unittest.TestCase):

    @mock.patch('time.time', mock.MagicMock(return_value=0))
    def setUp(self):
        DebugLine._update_debug_line = MagicMock()
        self._debug_line = DebugLine([0, 0, 0], [0, 0, 0])

    def test_create_debug_line_called(self):
        # Assert add debug line called in constructor
        self._debug_line._update_debug_line.assert_called_once()

    @mock.patch('time.time', mock.MagicMock(return_value=0))
    def test_not_updating_too_frequent(self):
        self._debug_line.update([0, 0, 0], [0, 0, 0])
        # Assert debug line is not called twice because cannot create line too frequent
        self.assertEqual(1, self._debug_line._update_debug_line.call_count)

    def test_updating_when_time_has_passed(self):
        # Mocking time to return number bigger than update frequency
        with patch('time.time', mock.MagicMock(return_value=DebugLine._MIN_UPDATE_INTERVAL + 1E-6)):
            self._debug_line.update([0, 0, 0], [0, 0, 0])
        # Assert debug line is not called twice because cannot create line too frequent
        self.assertEqual(2, self._debug_line._update_debug_line.call_count)
