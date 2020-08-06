from lobster_common.vec3 import Vec3


def interpolate(x: float, x1: float, x2: float, y1: float, y2: float) -> float:
    return y1 + ((y2 - y1) / (x2 - x1) * (x - x1))


def interpolate_vec(x: float, x1: float, x2: float, y1: Vec3, y2: Vec3) -> Vec3:
    assert x1 <= x <= x2

    return y1 + ((y2 - y1) / (x2 - x1) * (x - x1))


def clip(value: float, min_value: float, max_value: float) -> float:
    return max(min(value, max_value), min_value)