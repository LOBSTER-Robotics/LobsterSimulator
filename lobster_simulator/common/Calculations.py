from lobster_simulator.common.Vec3 import Vec3


def interpolate(x: float, x1: float, x2: float, y1: float, y2: float) -> float:
    return y1 + ((y2 - y1) / (x2 - x1) * (x - x1))


def interpolate_vec(x: float, x1: float, x2: float, y1: Vec3, y2: Vec3) -> Vec3:
    return y1 + ((y2 - y1) / (x2 - x1) * (x - x1))