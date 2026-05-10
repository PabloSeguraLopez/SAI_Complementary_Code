import math


def clamp(value: float, low: float, high: float) -> float:
    """Clamp value in [low, high]."""
    return max(low, min(high, value))


def wrap_pi(angle: float) -> float:
    """Wrap angle to [-pi, pi)."""
    return (angle + math.pi) % (2.0 * math.pi) - math.pi