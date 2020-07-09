from typing import Any, Union, List

import numpy as np

from lobster_simulator.common.Quaternion import Quaternion
from lobster_simulator.common.general_exceptions import InputDimensionError


class Vec3Nump(np.ndarray):

    def __new__(cls, input_array):
        obj = np.asarray(input_array).view(cls)



    def __init__(self, data: Union[List[float], np.ndarray]):
        """
        Creates a 3 dimensional vector from a data array
        :param data: Array with length 4 in the form [x, y, z]
        """
        super().__init__(shape)
        self.data: np.ndarray = np.asarray(data)

        if self.data.shape[0] != 3:
            raise InputDimensionError("A Vec3 needs an input array of length 3")

    def rotate(self, quaternion: Quaternion):
        pass

    def __add__(self, other):
        self.data + other

    def __radd__(self, other):
        other + self.data

    def __getitem__(self, key):
        return self.data[key]

    def __setitem__(self, key, value):
        self.data[key] = value
