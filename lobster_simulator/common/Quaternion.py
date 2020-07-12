from typing import Any, List, Union

import numpy as np

from lobster_simulator.common.general_exceptions import InputDimensionError

class Quaternion:

    def __init__(self, data: Any):
        """
        Creates a quaternion from a data array
        :param data: Array with length 4 in the form [x, y, z, w]
        """
        self._data: np.ndarray = np.asarray(data)

        if self._data.shape[0] != 4:
            raise InputDimensionError("A Quaternion needs an input array of length 4")

    @staticmethod
    def fromNWE(quaternion: Union['Quaternion', List[float], np.ndarray]) -> 'Quaternion':
        """
        Creates a quaternion in the NED coordinate system from a given array or Quaternion in the NWU coordinate system
        :param quaternion: Quaternion or array that represents a quaternion
        :return: Quaternion in the NED coordinate system
        """

        # Conversion follows https://stackoverflow.com/a/18818267, it needs to be checked if this is correct
        if isinstance(quaternion, Quaternion):
            # Swapping the Y and Z axes
            quaternion._data[1] = -quaternion._data[1]
            quaternion._data[2] = -quaternion._data[2]
            return quaternion
        elif isinstance(quaternion, List) or isinstance(quaternion, np.ndarray):
            # Swapping the Y and Z axes
            quaternion[1] = -quaternion[1]
            quaternion[2] = -quaternion[2]
            return Quaternion(quaternion)

        raise TypeError(f"Can only create NED quaternion from quaternion of array, not {type(quaternion)}")

    def toNWE(self) -> np.ndarray:
        # Swapping the Y and Z axes
        array = self._data.copy()
        array[1] = -array[1]
        array[2] = -array[2]
        return array

    def get_rotation_matrix(self):
        from lobster_simulator.tools.PybulletAPI import PybulletAPI
        return np.reshape(np.array(PybulletAPI.getMatrixFromQuaternion(self)), (3, 3))

    def get_inverse_rotation_matrix(self):
        return np.linalg.inv(self.get_rotation_matrix())

    def __str__(self):
        return f"Quaternion<{self._data}>"