import unittest

from lobster_simulator.common.calculations import interpolate


class CommonCalculationsTest(unittest.TestCase):

    def test_interpolation(self):

        min_input = 5
        max_input = 15

        min_output = -5
        max_output = 25

        inputs = [5, 7, 10, 12, 15, 20]
        outputs = [-5, 1, 10, 16, 25, 40]

        for i in range(len(inputs)):
            self.assertEqual(interpolate(inputs[i], min_input, max_input, min_output, max_output), outputs[i])

