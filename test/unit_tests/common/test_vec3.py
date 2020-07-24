import math

import pytest
import unittest

from lobster_simulator.common.Vec3 import Vec3
import numpy as np

from lobster_simulator.tools.PybulletAPI import PybulletAPI


class Vec3Test(unittest.TestCase):

    def test_rotate(self):
        for _ in range(10):
            vec = Vec3(np.random.rand(3))

            rotation = PybulletAPI.getQuaternionFromEuler(Vec3(np.random.rand(3) * 2 * math.pi))

            numpy_method = Vec3(rotation.get_rotation_matrix().dot(vec.array))

            rotate_method = vec.rotate(rotation)

            # assert numpy_method == rotate_method
            self.assertEqual(numpy_method, rotate_method)
