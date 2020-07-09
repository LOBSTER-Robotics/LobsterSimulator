from typing import Any

import numpy as np

from lobster_simulator.common.Quaternion import Quaternion
from lobster_simulator.common.general_exceptions import InputDimensionError


class Vec3:

    def __init__(self, data: Any):
        """
        Creates a 3 dimensional vector from a data array
        :param data: Array with length 4 in the form [x, y, z]
        """
        self.data: np.ndarray = np.asarray(data)

        if self.data.shape[0] != 3:
            raise InputDimensionError("A Vec3 needs an input array of length 3")

    def rotate(self, quaternion: Quaternion):
        pass

    def __add__(self, other):
        self.data + other

    def __radd__(self, other):
        other + self.data

