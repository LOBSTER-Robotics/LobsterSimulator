from typing import Any

import numpy as np

from lobster_simulator.common.general_exceptions import InputDimensionError



class Quaternion:

    def __init__(self, data: Any):
        """
        Creates a quaternion from a data array
        :param data: Array with length 4 in the form [x, y, z, w]
        """
        self.data: np.ndarray = np.asarray(data)

        if self.data.shape[0] != 4:
            raise InputDimensionError("A Quaternion needs an input array of length 4")

    def get_rotation_matrix(self):
        from lobster_simulator.tools.PybulletAPI import PybulletAPI
        return np.reshape(np.array(PybulletAPI.getMatrixFromQuaternion(self)), (3, 3))


    def __str__(self):
        return f"Quaternion<{self.data}>"