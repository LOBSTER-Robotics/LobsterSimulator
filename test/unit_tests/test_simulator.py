import math

import unittest
from unittest.mock import Mock

from lobster_common.vec3 import Vec3
import numpy as np

from lobster_simulator.common.pybullet_api import PybulletAPI
from lobster_simulator.simulator import Simulator


class SimulatorTest(unittest.TestCase):

    def test_add_ocean_floor(self):

        simulator = Simulator(4000, gui=False)

        temp_load = PybulletAPI.loadURDF
        temp_change_visual_shape = PybulletAPI.changeVisualShape

        PybulletAPI.loadURDF = Mock()
        PybulletAPI.changeVisualShape = Mock()

        simulator.add_ocean_floor(100)


        PybulletAPI.loadURDF.assert_called_once()
        PybulletAPI.changeVisualShape.assert_called_once()

        # Putting the original functions back, there might be a better way to do this.
        PybulletAPI.loadURDF = temp_load
        PybulletAPI.changeVisualShape = temp_change_visual_shape