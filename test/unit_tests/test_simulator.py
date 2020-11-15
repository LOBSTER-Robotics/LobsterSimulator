import math

import unittest
from unittest.mock import Mock

from lobster_simulator.common.pybullet_api import PybulletAPI
from lobster_simulator.simulator import Simulator


class SimulatorTest(unittest.TestCase):

    def test_add_ocean_floor(self):

        simulator = Simulator(4000, gui=False)

        with unittest.mock.patch("lobster_simulator.common.pybullet_api.PybulletAPI.loadURDF"):
            with unittest.mock.patch(
                    "lobster_simulator.common.pybullet_api.PybulletAPI.changeVisualShape"):

                simulator.add_ocean_floor(100)
                PybulletAPI.loadURDF.assert_called_once()
                PybulletAPI.changeVisualShape.assert_called_once()
