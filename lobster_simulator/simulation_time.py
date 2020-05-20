MICROSECONDS_IN_SECONDS = 1000000


def microseconds_to_seconds(microseconds: int) -> float:
    return microseconds / MICROSECONDS_IN_SECONDS


def seconds_to_microseconds(seconds: float) -> int:
    return int(seconds * MICROSECONDS_IN_SECONDS)


class SimulationTime:

    def __init__(self, initial_microseconds: int = 0):
        # todo add microseconds class
        self._micro_seconds = initial_microseconds

    @property
    def seconds(self) -> float:
        return microseconds_to_seconds(self._micro_seconds)

    @property
    def microseconds(self) -> int:
        return self._micro_seconds

    def __add__(self, other):
        return SimulationTime(self._micro_seconds + other.microseconds)

    def __sub__(self, other):
        return SimulationTime(self._micro_seconds - other.microseconds)

    def __le__(self, other):
        return self._micro_seconds <= other.microseconds
