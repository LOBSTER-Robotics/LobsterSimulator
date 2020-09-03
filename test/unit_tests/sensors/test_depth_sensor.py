import unittest

from lobster_simulator.simulator import Simulator
from lobster_common.vec3 import Vec3


class DepthSensorTest(unittest.TestCase):

    def test_pressure(self):
        simulator = Simulator(4000, gui=False)
        robot = simulator.create_robot()
        robot.set_position_and_orientation(position=Vec3((0,0,0)))
        pressure = robot._depth_sensor.get_pressure()

        # 101.3 = (0 * ((self._water_density * GRAVITY) + self._OFFSET) + self._STANDARD_ATMOSPHERE) / self._KPA_TO_PA
        self.assertAlmostEqual(101.3, pressure)
