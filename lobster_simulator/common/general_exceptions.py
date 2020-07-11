class ArgumentNoneError(Exception):
    """
    Raised when an argument of a function was None but shouldn't be.
    """
    pass


class InvalidArgumentTypeError(Exception):
    pass


class InputDimensionError(Exception):
    """
    Raised when the size of the input dimension is correct
    """