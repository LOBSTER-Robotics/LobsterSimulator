MICROSECONDS_IN_SECONDS = 1000000


def microseconds_to_seconds(microseconds: float) -> float:
    return microseconds / MICROSECONDS_IN_SECONDS


def seconds_to_microseconds(seconds: float) -> float:
    return seconds * MICROSECONDS_IN_SECONDS


def microseconds_to_milliseconds(microseconds: float) -> float:
    return microseconds / 1000


class SimulationTime:

    def __init__(self, initial_microseconds: int = 0):
        # todo add microseconds class
        self._micro_seconds = initial_microseconds

    @property
    def seconds(self) -> float:
        return microseconds_to_seconds(self._micro_seconds)

    @property
    def milliseconds(self) -> float:
        return microseconds_to_milliseconds(self._micro_seconds)

    @property
    def microseconds(self) -> int:
        return self._micro_seconds

    def __add__(self, other):
        return SimulationTime(self._micro_seconds + other.microseconds)

    def __sub__(self, other):
        return SimulationTime(self._micro_seconds - other.microseconds)

    def __le__(self, other):
        return self._micro_seconds <= other.microseconds

    def add_time_step(self, time_step: int):
        """
        Add time step in microseconds
        """
        self._micro_seconds += time_step
