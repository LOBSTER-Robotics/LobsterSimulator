import unittest

from lobster_simulator.simulator import Simulator


class TestSimulatorClass(unittest.TestCase):

    def test(self):
        self._simulator = Simulator(time_step=1, gui=False)
