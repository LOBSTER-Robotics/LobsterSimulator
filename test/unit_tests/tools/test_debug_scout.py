import unittest

from lobster_common.vec3 import Vec3

from lobster_simulator.common.debug_visualization import DebugScout
from lobster_simulator.common.pybullet_api import PybulletAPI
from lobster_simulator.common.simulation_time import SimulationTime


class TestDebugScout(unittest.TestCase):

    def setUp(self):
        PybulletAPI.initialize(SimulationTime(1000), False)
        self._debug_scout = DebugScout(Vec3([0, 0, 0]))

    def test_correctly_load_urdf_and_remove(self):
        self.assertIsNotNone(self._debug_scout.object_id)

        self._debug_scout.remove()

        self.assertIsNone(self._debug_scout.object_id)
