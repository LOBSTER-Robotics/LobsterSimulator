from abc import abstractmethod
from numbers import Number
from typing import Union

MICROSECONDS_PER_SECOND = int(1e6)
NANOSECONDS_PER_SECOND = int(1e9)


class Time(int):

    def __new__(cls, nanoseconds):
        return int.__new__(cls, nanoseconds)

    def __add__(self, other):
        if isinstance(other, Time):
            return Time(super(Time, self).__add__(other))

        raise TypeError(f"{type(other)} cannot be added to {type(self)}")

    def __repr__(self) -> str:
        return f"Time<{int(self)} nanoseconds>"

    def __str__(self) -> str:
        return f"Time<{super().__str__()[:-9]}.{super().__str__()[-9:]} seconds>"


class Seconds(Time):

    def __new__(cls, seconds):
        return super().__new__(cls, seconds * NANOSECONDS_PER_SECOND)


class NanoSeconds(Time):

    def __new__(cls, nanoseconds):
        return super().__new__(cls, nanoseconds)