import unittest
import numpy as np

from lobster_common.quaternion import Quaternion
from lobster_common.vec3 import Vec3
from lobster_simulator.common.pybullet_api import PybulletAPI


class CoordinateSystemConversionTest(unittest.TestCase):

    def test_conversion(self):

        vecs = [Vec3([0.4986, 0.5636, 0.6595]), Vec3([0.6146, 0.6041, 0.9962]), Vec3([0.3084, 0.9931, 0.7015]),
                Vec3([0.2766, 0.0364, 0.6429]), Vec3([0.2327, 0.8240, 0.4630]), Vec3([0.0653, 0.1516, 0.5897]),
                Vec3([0.8067, 0.2428, 0.5630]), Vec3([0.2868, 0.8717, 0.1578]), Vec3([0.7447, 0.0236, 0.4079]),
                Vec3([0.2840, 0.5293, 0.1107])]

        # Creating random vector
        for vec in vecs:
            # Creating random rotation
            q = PybulletAPI.getQuaternionFromEuler(Vec3(np.random.rand(3)))

            # Rotate the vector by the rotation
            rotated_vec = vec.rotate(q)

            # transform the original vector to the NED coordinate system
            vec_NED = Vec3.fromENU(vec)

            # transform the original quaternino to the NED coordinate system
            q_NED = Quaternion.fromENU(q)

            # Rotate the converted vector by the converted rotation
            rotated_vec_NED = vec_NED.rotate(q_NED)

            # transform the rotated vector to the NED coordinate system
            rotated_vec_NED_transformed = Vec3.fromENU(rotated_vec)

            self.assertEqual(rotated_vec_NED, rotated_vec_NED_transformed)
