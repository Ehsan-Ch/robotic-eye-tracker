"""Simulation duration validation independent of the rendering backend."""
import math


def frame_count(seconds, dt):
    if (not isinstance(seconds, (int, float)) or isinstance(seconds, bool)
            or not math.isfinite(seconds) or not 0 < seconds <= 60
            or not math.isfinite(dt) or dt <= 0):
        raise ValueError('Duration must be finite and in (0, 60] seconds; timestep must be positive.')
    return math.ceil(seconds / dt)
