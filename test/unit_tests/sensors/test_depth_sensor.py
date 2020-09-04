import unittest

from lobster_simulator.simulator import Simulator
from lobster_common.vec3 import Vec3


class DepthSensorTest(unittest.TestCase):

    def test_pressure_at_surface(self):
        simulator = Simulator(4000, gui=False)
        robot = simulator.create_robot()
        robot.set_position_and_orientation(position=Vec3((0,0,0)))
        simulator.do_step()
        pressure = robot._pressure_sensor.get_pressure()
        # 101.3 = (0 * ((self._water_density * GRAVITY) + self._OFFSET) + self._STANDARD_ATMOSPHERE) / self._KPA_TO_PA
        self.assertAlmostEqual(101.3, pressure)

    def test_pressure_increases_when_going_down(self):
        """Move the robot down and assert that the pressure will only increase."""
        simulator = Simulator(4000, gui=False)
        simulator.create_robot()
        downwards_velocity = Vec3([0, 0, 1])
        simulator.robot.set_velocity(linear_velocity=downwards_velocity)
        simulator.do_step()
        previous_pressure = simulator.robot._pressure_sensor.get_pressure()

        for _ in range(100):
            simulator.robot.set_velocity(linear_velocity=downwards_velocity)
            simulator.do_step()
            current_pressure = simulator.robot._pressure_sensor.get_pressure()

            self.assertLess(previous_pressure, current_pressure)
            previous_pressure = current_pressure