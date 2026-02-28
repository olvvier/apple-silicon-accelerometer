"""Zero-dependency signal processing utilities for IMU data."""

from __future__ import annotations

import math
from typing import Sequence


def magnitude(x: float, y: float, z: float) -> float:
    """Euclidean magnitude of a 3-axis sample."""
    return math.sqrt(x * x + y * y + z * z)


def remove_gravity(samples: Sequence, alpha: float = 0.98):
    """Remove gravity from accelerometer samples using a complementary filter.

    At rest, accelerometer reads ~1g from gravity. This subtracts
    a slow-moving gravity estimate to isolate dynamic acceleration.

    Parameters
    ----------
    samples : list of Sample or TimedSample
        Accelerometer samples (x, y, z in g).
    alpha : float
        Filter coefficient (0.9-0.99). Higher = slower gravity tracking.

    Returns list of tuples (x, y, z) with gravity removed.
    """
    if not samples:
        return []
    s0 = samples[0]
    gx, gy, gz = s0.x, s0.y, s0.z
    result = []
    for s in samples:
        gx = alpha * gx + (1 - alpha) * s.x
        gy = alpha * gy + (1 - alpha) * s.y
        gz = alpha * gz + (1 - alpha) * s.z
        result.append((s.x - gx, s.y - gy, s.z - gz))
    return result


def low_pass(samples: Sequence, cutoff_hz: float, sample_rate: float):
    """First-order IIR low-pass filter on 3-axis samples.

    Returns list of tuples (x, y, z).
    """
    if not samples:
        return []
    rc = 1.0 / (2.0 * math.pi * cutoff_hz)
    dt = 1.0 / sample_rate
    a = dt / (rc + dt)
    s0 = samples[0]
    px, py, pz = s0.x, s0.y, s0.z
    result = [(px, py, pz)]
    for s in samples[1:]:
        px += a * (s.x - px)
        py += a * (s.y - py)
        pz += a * (s.z - pz)
        result.append((px, py, pz))
    return result


def high_pass(samples: Sequence, cutoff_hz: float, sample_rate: float):
    """First-order IIR high-pass filter on 3-axis samples.

    Returns list of tuples (x, y, z).
    """
    if not samples:
        return []
    rc = 1.0 / (2.0 * math.pi * cutoff_hz)
    dt = 1.0 / sample_rate
    a = rc / (rc + dt)
    s0 = samples[0]
    prev_in = (s0.x, s0.y, s0.z)
    prev_out = (0.0, 0.0, 0.0)
    result = [prev_out]
    for s in samples[1:]:
        ox = a * (prev_out[0] + s.x - prev_in[0])
        oy = a * (prev_out[1] + s.y - prev_in[1])
        oz = a * (prev_out[2] + s.z - prev_in[2])
        prev_in = (s.x, s.y, s.z)
        prev_out = (ox, oy, oz)
        result.append(prev_out)
    return result


def bandpass(samples: Sequence, low_hz: float, high_hz: float,
             sample_rate: float):
    """Cascaded high-pass + low-pass bandpass filter.

    Returns list of tuples (x, y, z).
    """
    from collections import namedtuple
    _S = namedtuple('_S', ['x', 'y', 'z'])
    hp = high_pass(samples, low_hz, sample_rate)
    hp_samples = [_S(*s) for s in hp]
    return low_pass(hp_samples, high_hz, sample_rate)


def rolling_rms(samples: Sequence, window: int = 50):
    """Rolling root-mean-square of magnitude.

    Returns list of float (one per sample).
    """
    buf = []
    result = []
    for s in samples:
        m = s.x * s.x + s.y * s.y + s.z * s.z
        buf.append(m)
        if len(buf) > window:
            buf.pop(0)
        result.append(math.sqrt(sum(buf) / len(buf)))
    return result
