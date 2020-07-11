import pytest

from lobster_simulator.common.TimeUnits import Seconds, NanoSeconds


def test_timeunit():

    s = Seconds(10)

    ns = NanoSeconds(1e9)

    print(s + ns)
    print(type(s + ns))

# test_timeunit()