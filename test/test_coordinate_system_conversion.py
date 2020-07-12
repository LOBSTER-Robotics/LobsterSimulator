import unittest
import numpy as np

from lobster_simulator.common.Quaternion import Quaternion
from lobster_simulator.common.Vec3 import Vec3
from lobster_simulator.tools.PybulletAPI import PybulletAPI


class CoordinateSystemConversionTest(unittest.TestCase):

    @staticmethod
    def test_conversion():

        # Creating random vector
        vec = Vec3(np.random.rand(3))

        # Creating random rotation
        q = PybulletAPI.getQuaternionFromEuler(np.random.rand(3))

        # Rotate the vector by the rotation
        rotated_vec = vec.rotate(q)

        # transform the original vector to the NED coordinate system
        vec_NED = Vec3.fromNWE(vec)

        # transform the original quaternino to the NED coordinate system
        q_NED = Quaternion.fromNWE(q)

        # Rotate the converted vector by the converted rotation
        rotated_vec_NED = vec_NED.rotate(q_NED)

        # transform the rotated vector to the NED coordinate system
        rotated_vec_NED_transformed = Vec3.fromNWE(rotated_vec)

        print(rotated_vec_NED, rotated_vec_NED_transformed)

        assert rotated_vec_NED == rotated_vec_NED_transformed



