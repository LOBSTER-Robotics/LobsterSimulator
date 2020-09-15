import unittest
from unittest import mock
from unittest.mock import Mock

from lobster_simulator.common.pybullet_api import PybulletAPI
from lobster_simulator.common.simulation_time import SimulationTime


class PybulletApiTest(unittest.TestCase):

    def test_resetting(self):
        PybulletAPI._INSTANCE = Mock()
        # Reset is called when old instance is still connected to a physics server
        with mock.patch("pybullet.isConnected", return_value=True):
            with mock.patch("pybullet.resetSimulation", return_value=True) as resetSimulationMock:
                p = PybulletAPI(SimulationTime(1000))
                resetSimulationMock.assert_called_once()

        # Reset is not called when not connected
        with mock.patch("pybullet.isConnected", return_value=False):
            with mock.patch("pybullet.resetSimulation", return_value=True) as resetSimulationMock:
                p = PybulletAPI(SimulationTime(1000))
                resetSimulationMock.assert_not_called()

        PybulletAPI._INSTANCE = None

        with mock.patch("pybullet.resetSimulation", return_value=True) as resetSimulationMock:
            PybulletAPI(SimulationTime(1000))
            resetSimulationMock.assert_not_called()

        # Checking for no side effects
        self.assertIsNone(PybulletAPI._INSTANCE)
