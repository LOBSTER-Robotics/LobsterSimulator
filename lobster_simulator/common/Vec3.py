from enum import Enum, auto
from typing import Any, Union, List, TYPE_CHECKING, Tuple

import numpy as np

from lobster_simulator.common.Quaternion import Quaternion
from lobster_simulator.common.general_exceptions import InputDimensionError, InvalidArgumentTypeError


class Vec3:

    def __init__(self, data: Union[List[float], Tuple[float, float, float], np.ndarray]):
        """
        Creates a 3 dimensional vector from a data array
        :param data: Array with length 4 in the form [x, y, z]
        """

        if not (isinstance(data, np.ndarray) or isinstance(data, List) or isinstance(data, Tuple)):
            raise InvalidArgumentTypeError(
                f"A Vec3 needs to be instantiated by a list of floats a tuple of floats or a numpy array not a {type(data)}")

        self._data: np.ndarray = np.asarray(data)
        assert self._data.shape
        if self._data.shape[0] != 3:
            raise InputDimensionError("A Vec3 needs an input array of length 3")
        elif self._data.dtype != float and self._data.dtype != int:
            raise InvalidArgumentTypeError(
                f"A Vec3 needs to be instantiated by an array of floats, not an array of {self._data.dtype}")

    @staticmethod
    def fromENU(vector: Union['Vec3', List[float], Tuple[float, float, float], np.ndarray]):
        if isinstance(vector, Vec3):
            # Swapping the Y and Z axes
            vector._data[1] = -vector._data[1]
            vector._data[2] = -vector._data[2]
            return vector
        elif isinstance(vector, List) or isinstance(vector, np.ndarray):
            # Swapping the Y and Z axes
            vector[1] = -vector[1]
            vector[2] = -vector[2]
            return Vec3(vector)
        elif isinstance(vector, Tuple):
            vector: Tuple[float, float, float] = (float(vector[0]), float(-vector[1]), float(-vector[2]))
            return Vec3(vector)

        raise TypeError(f"Can only create NED vector from Vec3 of array, not {type(vector)}")

    def asENU(self) -> np.ndarray:
        # Swapping the Y and Z axes
        array = self._data.copy()
        array[1] = -array[1]
        array[2] = -array[2]
        return array

    @property
    def array(self):
        return self._data

    def rotate(self, quaternion: Quaternion) -> 'Vec3':
        return Vec3(quaternion.get_rotation_matrix().dot(self._data))

    def rotate_inverse(self, quaternion: Quaternion) -> 'Vec3':
        return Vec3(quaternion.get_inverse_rotation_matrix().dot(self._data))

    def __add__(self, other):
        if isinstance(other, Vec3):
            return Vec3(self._data + other._data)
        elif isinstance(other, np.ndarray):
            to_return = other + self._data
            return Vec3(to_return)

        raise InvalidArgumentTypeError(f"A {type(other)} cannot be added to a Vec3")

    def __radd__(self, other):
        if isinstance(other, np.ndarray):
            new_vec = Vec3(other + self._data)
            return new_vec
        elif isinstance(other, float):
            return Vec3(other + self._data)

        raise InvalidArgumentTypeError(f"A Vec3 cannot be added to a {type(other)}]")

    def __sub__(self, other) -> 'Vec3':
        if isinstance(other, Vec3):
            return Vec3(self._data - other._data)

        raise InvalidArgumentTypeError(f"A {type(other)} cannot be subtracted from a Vec3")

    def __rsub__(self, other):
        return other - self._data

    def __mul__(self, other):
        if isinstance(other, float) or isinstance(other, int):
            return Vec3(self._data * other)
        elif isinstance(other, Vec3):
            return Vec3(self._data * other._data)

        raise InvalidArgumentTypeError(f"A Vec3 cannot be multiplied with a {type(other)}")

    def __truediv__(self, other):
        if isinstance(other, float) or isinstance(other, int):
            return Vec3(self._data / other)

        raise InvalidArgumentTypeError(f"A Vec3 cannot be divided by a {type(other)}")

    def __getitem__(self, key):
        return self._data[key]

    def __setitem__(self, key, value):
        self._data[key] = value

    def __str__(self):
        return f"Vec3<{self._data}>"

    def __eq__(self, other):
        if isinstance(other, Vec3):
            return (self._data == other._data).all()

        return False
