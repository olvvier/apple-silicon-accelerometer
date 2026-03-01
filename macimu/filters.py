"""Zero-dependency signal processing utilities for IMU data.

Filters use biquad (second-order IIR) sections for proper rolloff
(-12 dB/octave per section). Coefficients follow the Audio EQ Cookbook.
"""

from __future__ import annotations

import math
from typing import Sequence


def magnitude(x: float, y: float, z: float) -> float:
    """Euclidean magnitude of a 3-axis sample."""
    return math.sqrt(x * x + y * y + z * z)


# ---------------------------------------------------------------------------
# biquad engine (direct form II transposed)
# ---------------------------------------------------------------------------

def _biquad_coeffs_lp(cutoff_hz: float, sample_rate: float, Q: float = 0.7071):
    """Butterworth low-pass biquad coefficients."""
    w0 = 2.0 * math.pi * cutoff_hz / sample_rate
    alpha = math.sin(w0) / (2.0 * Q)
    cos_w0 = math.cos(w0)
    b0 = (1.0 - cos_w0) / 2.0
    b1 = 1.0 - cos_w0
    b2 = (1.0 - cos_w0) / 2.0
    a0 = 1.0 + alpha
    a1 = -2.0 * cos_w0
    a2 = 1.0 - alpha
    return (b0 / a0, b1 / a0, b2 / a0, a1 / a0, a2 / a0)


def _biquad_coeffs_hp(cutoff_hz: float, sample_rate: float, Q: float = 0.7071):
    """Butterworth high-pass biquad coefficients."""
    w0 = 2.0 * math.pi * cutoff_hz / sample_rate
    alpha = math.sin(w0) / (2.0 * Q)
    cos_w0 = math.cos(w0)
    b0 = (1.0 + cos_w0) / 2.0
    b1 = -(1.0 + cos_w0)
    b2 = (1.0 + cos_w0) / 2.0
    a0 = 1.0 + alpha
    a1 = -2.0 * cos_w0
    a2 = 1.0 - alpha
    return (b0 / a0, b1 / a0, b2 / a0, a1 / a0, a2 / a0)


def _biquad_filter_3ax(samples, b0, b1, b2, a1, a2):
    """Apply biquad to 3-axis samples (direct form II transposed)."""
    if not samples:
        return []
    dx1, dx2, dy1, dy2, dz1, dz2 = 0.0, 0.0, 0.0, 0.0, 0.0, 0.0
    result = []
    for s in samples:
        ox = b0 * s.x + dx1
        dx1 = b1 * s.x - a1 * ox + dx2
        dx2 = b2 * s.x - a2 * ox
        oy = b0 * s.y + dy1
        dy1 = b1 * s.y - a1 * oy + dy2
        dy2 = b2 * s.y - a2 * oy
        oz = b0 * s.z + dz1
        dz1 = b1 * s.z - a1 * oz + dz2
        dz2 = b2 * s.z - a2 * oz
        result.append((ox, oy, oz))
    return result


# ---------------------------------------------------------------------------
# public filters
# ---------------------------------------------------------------------------

class GravityKalman:
    """Kalman filter for real-time gravity estimation on 3-axis accelerometer.

    Tracks the gravity vector as state, treating dynamic acceleration as
    measurement noise. Statistically optimal for Gaussian noise and adapts
    the Kalman gain based on estimation uncertainty.

    Parameters
    ----------
    process_noise : float
        How fast gravity can change (Q). Lower = more stable estimate.
        Default 0.001 is good for a device that rotates slowly.
    measurement_noise : float
        Expected accelerometer noise + dynamic acceleration variance (R).
        Default 0.1 handles typical hand motion. Increase for vigorous use.
    """

    def __init__(self, process_noise: float = 0.001,
                 measurement_noise: float = 0.1):
        self._q = process_noise
        self._r = measurement_noise
        self._gx = 0.0
        self._gy = 0.0
        self._gz = -1.0
        self._px = 1.0
        self._py = 1.0
        self._pz = 1.0
        self._initialized = False

    def update(self, ax: float, ay: float, az: float):
        """Feed one accelerometer sample, returns (gx, gy, gz) gravity estimate."""
        if not self._initialized:
            self._gx, self._gy, self._gz = ax, ay, az
            self._initialized = True
            return (self._gx, self._gy, self._gz)

        # predict (gravity is ~constant, covariance grows by Q)
        px = self._px + self._q
        py = self._py + self._q
        pz = self._pz + self._q

        # update (Kalman gain per axis)
        kx = px / (px + self._r)
        ky = py / (py + self._r)
        kz = pz / (pz + self._r)

        self._gx += kx * (ax - self._gx)
        self._gy += ky * (ay - self._gy)
        self._gz += kz * (az - self._gz)

        self._px = (1.0 - kx) * px
        self._py = (1.0 - ky) * py
        self._pz = (1.0 - kz) * pz

        return (self._gx, self._gy, self._gz)

    @property
    def gravity(self):
        return (self._gx, self._gy, self._gz)

    def reset(self):
        self._initialized = False
        self._px = self._py = self._pz = 1.0


def remove_gravity(samples: Sequence, process_noise: float = 0.001,
                   measurement_noise: float = 0.1):
    """Remove gravity from accelerometer samples using a Kalman filter.

    At rest, accelerometer reads ~1g from gravity. This estimates
    the gravity vector optimally and subtracts it to isolate
    dynamic acceleration.

    Parameters
    ----------
    samples : list of Sample or TimedSample
        Accelerometer samples (x, y, z in g).
    process_noise : float
        Kalman Q -- how fast gravity estimate can drift. Lower = smoother.
    measurement_noise : float
        Kalman R -- expected noise + dynamic accel variance. Higher =
        trusts measurements less (better at ignoring transient motion).

    Returns list of tuples (x, y, z) with gravity removed.
    """
    if not samples:
        return []
    kf = GravityKalman(process_noise, measurement_noise)
    result = []
    for s in samples:
        gx, gy, gz = kf.update(s.x, s.y, s.z)
        result.append((s.x - gx, s.y - gy, s.z - gz))
    return result


def low_pass(samples: Sequence, cutoff_hz: float, sample_rate: float,
             order: int = 2):
    """Butterworth low-pass filter on 3-axis samples.

    Parameters
    ----------
    order : int
        Filter order (2 or 4). Each order of 2 adds -12 dB/octave rolloff.

    Returns list of tuples (x, y, z).
    """
    if not samples or cutoff_hz <= 0 or sample_rate <= 0:
        return list(samples) if samples else []
    coeffs = _biquad_coeffs_lp(cutoff_hz, sample_rate)
    out = _biquad_filter_3ax(samples, *coeffs)
    if order >= 4:
        from collections import namedtuple
        _S = namedtuple('_S', ['x', 'y', 'z'])
        out = _biquad_filter_3ax([_S(*s) for s in out], *coeffs)
    return out


def high_pass(samples: Sequence, cutoff_hz: float, sample_rate: float,
              order: int = 2):
    """Butterworth high-pass filter on 3-axis samples.

    Parameters
    ----------
    order : int
        Filter order (2 or 4). Each order of 2 adds -12 dB/octave rolloff.

    Returns list of tuples (x, y, z).
    """
    if not samples or cutoff_hz <= 0 or sample_rate <= 0:
        return list(samples) if samples else []
    coeffs = _biquad_coeffs_hp(cutoff_hz, sample_rate)
    out = _biquad_filter_3ax(samples, *coeffs)
    if order >= 4:
        from collections import namedtuple
        _S = namedtuple('_S', ['x', 'y', 'z'])
        out = _biquad_filter_3ax([_S(*s) for s in out], *coeffs)
    return out


def bandpass(samples: Sequence, low_hz: float, high_hz: float,
             sample_rate: float, order: int = 2):
    """Cascaded high-pass + low-pass bandpass filter.

    Returns list of tuples (x, y, z).
    """
    from collections import namedtuple
    _S = namedtuple('_S', ['x', 'y', 'z'])
    hp = high_pass(samples, low_hz, sample_rate, order=order)
    return low_pass([_S(*s) for s in hp], high_hz, sample_rate, order=order)


def filtfilt_low_pass(samples: Sequence, cutoff_hz: float, sample_rate: float):
    """Zero-phase low-pass filter (forward-backward biquad).

    Eliminates phase lag by applying the filter in both directions.
    Only suitable for offline / batch processing (not real-time).

    Returns list of tuples (x, y, z).
    """
    from collections import namedtuple
    _S = namedtuple('_S', ['x', 'y', 'z'])
    coeffs = _biquad_coeffs_lp(cutoff_hz, sample_rate)
    fwd = _biquad_filter_3ax(samples, *coeffs)
    rev = _biquad_filter_3ax([_S(*s) for s in reversed(fwd)], *coeffs)
    rev.reverse()
    return rev


def filtfilt_high_pass(samples: Sequence, cutoff_hz: float, sample_rate: float):
    """Zero-phase high-pass filter (forward-backward biquad).

    Eliminates phase lag. Only suitable for offline / batch processing.

    Returns list of tuples (x, y, z).
    """
    from collections import namedtuple
    _S = namedtuple('_S', ['x', 'y', 'z'])
    coeffs = _biquad_coeffs_hp(cutoff_hz, sample_rate)
    fwd = _biquad_filter_3ax(samples, *coeffs)
    rev = _biquad_filter_3ax([_S(*s) for s in reversed(fwd)], *coeffs)
    rev.reverse()
    return rev


def median_filter(samples: Sequence, window: int = 5):
    """Median filter on 3-axis samples for spike / outlier removal.

    Returns list of tuples (x, y, z).
    """
    if not samples:
        return []
    n = len(samples)
    half = window // 2
    result = []
    for i in range(n):
        lo = max(0, i - half)
        hi = min(n, i + half + 1)
        xs = sorted(s.x for s in samples[lo:hi])
        ys = sorted(s.y for s in samples[lo:hi])
        zs = sorted(s.z for s in samples[lo:hi])
        mid = len(xs) // 2
        result.append((xs[mid], ys[mid], zs[mid]))
    return result


def peak_detect(values: Sequence[float], threshold: float,
                min_spacing: int = 10):
    """Detect peaks in a 1D signal (e.g. magnitude).

    Parameters
    ----------
    values : list of float
    threshold : float
        Minimum peak height.
    min_spacing : int
        Minimum samples between consecutive peaks.

    Returns list of (index, value) for each detected peak.
    """
    if len(values) < 3:
        return []
    peaks = []
    last_peak = -min_spacing
    for i in range(1, len(values) - 1):
        if (values[i] > values[i - 1] and values[i] > values[i + 1]
                and values[i] >= threshold and (i - last_peak) >= min_spacing):
            peaks.append((i, values[i]))
            last_peak = i
    return peaks


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
