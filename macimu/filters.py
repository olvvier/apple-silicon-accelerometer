"""Zero-dependency signal processing utilities for IMU data.

Biquad filters use second-order IIR sections (-12 dB/octave per section).
Every filter works both in batch (list of samples) and streaming (one
sample at a time via stateful classes).
"""

from __future__ import annotations

import math
from collections import namedtuple
from typing import Sequence

Sample = namedtuple('Sample', ['x', 'y', 'z'])


def magnitude(x: float, y: float, z: float) -> float:
    """Euclidean magnitude of a 3-axis sample."""
    return math.sqrt(x * x + y * y + z * z)


# ---------------------------------------------------------------------------
# biquad coefficients (Audio EQ Cookbook)
# ---------------------------------------------------------------------------

def _biquad_coeffs_lp(cutoff_hz: float, sample_rate: float, Q: float = 0.7071):
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


# ---------------------------------------------------------------------------
# stateful streaming classes
# ---------------------------------------------------------------------------

class BiquadFilter:
    """Stateful biquad filter for sample-by-sample streaming.

    Parameters
    ----------
    mode : str
        'lp' for low-pass, 'hp' for high-pass.
    cutoff_hz : float
        Filter cutoff frequency in Hz.
    sample_rate : float
        Sample rate in Hz.
    order : int
        Filter order (2 or 4).
    """

    def __init__(self, mode: str, cutoff_hz: float, sample_rate: float,
                 order: int = 2):
        if mode == 'lp':
            self._coeffs = _biquad_coeffs_lp(cutoff_hz, sample_rate)
        elif mode == 'hp':
            self._coeffs = _biquad_coeffs_hp(cutoff_hz, sample_rate)
        else:
            raise ValueError("mode must be 'lp' or 'hp'")
        self._order = order
        self._d1 = [0.0] * 6
        self._d2 = [0.0] * 6 if order >= 4 else None

    def feed(self, x: float, y: float, z: float) -> tuple:
        """Filter one 3-axis sample. Returns (x, y, z)."""
        b0, b1, b2, a1, a2 = self._coeffs
        d = self._d1
        ox = b0 * x + d[0]; d[0] = b1 * x - a1 * ox + d[1]; d[1] = b2 * x - a2 * ox
        oy = b0 * y + d[2]; d[2] = b1 * y - a1 * oy + d[3]; d[3] = b2 * y - a2 * oy
        oz = b0 * z + d[4]; d[4] = b1 * z - a1 * oz + d[5]; d[5] = b2 * z - a2 * oz
        if self._d2 is not None:
            d = self._d2
            x, y, z = ox, oy, oz
            ox = b0*x + d[0]; d[0] = b1*x - a1*ox + d[1]; d[1] = b2*x - a2*ox
            oy = b0*y + d[2]; d[2] = b1*y - a1*oy + d[3]; d[3] = b2*y - a2*oy
            oz = b0*z + d[4]; d[4] = b1*z - a1*oz + d[5]; d[5] = b2*z - a2*oz
        return (ox, oy, oz)

    def feed_sample(self, s) -> Sample:
        """Filter one Sample, returns Sample(x, y, z)."""
        return Sample(*self.feed(s.x, s.y, s.z))

    def reset(self):
        self._d1 = [0.0] * 6
        if self._d2 is not None:
            self._d2 = [0.0] * 6


class GravityKalman:
    """Kalman filter for real-time gravity estimation on 3-axis accelerometer.

    Parameters
    ----------
    process_noise : float
        How fast gravity can change (Q). Lower = more stable.
    measurement_noise : float
        Expected noise + dynamic acceleration variance (R).
    warmup_samples : int
        Number of samples before the estimate is considered stable.
    """

    def __init__(self, process_noise: float = 0.001,
                 measurement_noise: float = 0.1,
                 warmup_samples: int = 200):
        self._q = process_noise
        self._r = measurement_noise
        self._warmup_n = warmup_samples
        self._gx = 0.0
        self._gy = 0.0
        self._gz = -1.0
        self._px = 1.0
        self._py = 1.0
        self._pz = 1.0
        self._count = 0
        self._initialized = False

    def update(self, ax: float, ay: float, az: float):
        """Feed one sample, returns (gx, gy, gz) gravity estimate."""
        self._count += 1
        if not self._initialized:
            self._gx, self._gy, self._gz = ax, ay, az
            self._initialized = True
            return (self._gx, self._gy, self._gz)

        px = self._px + self._q
        py = self._py + self._q
        pz = self._pz + self._q

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

    def feed_remove(self, ax: float, ay: float, az: float) -> tuple:
        """Feed one sample, returns dynamic acceleration (dx, dy, dz) with gravity removed."""
        gx, gy, gz = self.update(ax, ay, az)
        return (ax - gx, ay - gy, az - gz)

    def feed_remove_sample(self, s) -> Sample:
        """Feed one Sample, returns Sample(x, y, z) with gravity removed."""
        return Sample(*self.feed_remove(s.x, s.y, s.z))

    @property
    def warmed_up(self) -> bool:
        """True when enough samples have been processed for a stable estimate."""
        return self._count >= self._warmup_n

    @property
    def gravity(self):
        return (self._gx, self._gy, self._gz)

    def reset(self):
        self._initialized = False
        self._count = 0
        self._px = self._py = self._pz = 1.0


class StreamingPeakDetector:
    """Stateful peak detector for real-time sample-by-sample use.

    Parameters
    ----------
    threshold : float
        Minimum value to qualify as a peak.
    min_spacing : int
        Minimum samples between consecutive peaks.
    """

    def __init__(self, threshold: float, min_spacing: int = 10):
        self._threshold = threshold
        self._min_spacing = min_spacing
        self._prev2 = None
        self._prev1 = None
        self._since_last = min_spacing

    def feed(self, value: float) -> bool:
        """Feed one scalar value. Returns True if a peak was just detected."""
        self._since_last += 1
        result = False
        if (self._prev2 is not None and self._prev1 is not None
                and self._prev1 > self._prev2 and self._prev1 > value
                and self._prev1 >= self._threshold
                and self._since_last >= self._min_spacing):
            result = True
            self._since_last = 0
        self._prev2 = self._prev1
        self._prev1 = value
        return result

    def reset(self):
        self._prev2 = None
        self._prev1 = None
        self._since_last = self._min_spacing


class StreamingRMS:
    """Streaming rolling root-mean-square of 3-axis magnitude.

    Uses a circular buffer for O(1) per sample.

    Parameters
    ----------
    window : int
        Number of samples in the rolling window.
    """

    def __init__(self, window: int = 50):
        self._window = window
        self._buf = [0.0] * window
        self._idx = 0
        self._sum = 0.0
        self._count = 0

    def feed(self, x: float, y: float, z: float) -> float:
        """Feed one 3-axis sample, returns current RMS magnitude."""
        sq = x * x + y * y + z * z
        self._sum -= self._buf[self._idx]
        self._buf[self._idx] = sq
        self._sum += sq
        self._idx = (self._idx + 1) % self._window
        self._count = min(self._count + 1, self._window)
        return math.sqrt(self._sum / self._count)

    def reset(self):
        self._buf = [0.0] * self._window
        self._idx = 0
        self._sum = 0.0
        self._count = 0


# ---------------------------------------------------------------------------
# batch filters (all return Sample namedtuples)
# ---------------------------------------------------------------------------

def _biquad_filter_3ax(samples, b0, b1, b2, a1, a2):
    if not samples:
        return []
    dx1, dx2, dy1, dy2, dz1, dz2 = 0.0, 0.0, 0.0, 0.0, 0.0, 0.0
    result = []
    for s in samples:
        ox = b0 * s.x + dx1; dx1 = b1 * s.x - a1 * ox + dx2; dx2 = b2 * s.x - a2 * ox
        oy = b0 * s.y + dy1; dy1 = b1 * s.y - a1 * oy + dy2; dy2 = b2 * s.y - a2 * oy
        oz = b0 * s.z + dz1; dz1 = b1 * s.z - a1 * oz + dz2; dz2 = b2 * s.z - a2 * oz
        result.append(Sample(ox, oy, oz))
    return result


def remove_gravity(samples: Sequence, process_noise: float = 0.001,
                   measurement_noise: float = 0.1, skip_warmup: bool = False):
    """Remove gravity using a Kalman filter.

    Parameters
    ----------
    skip_warmup : bool
        If True, returns zeros for the first 200 samples (warmup period).

    Returns list of Sample(x, y, z) with gravity removed.
    """
    if not samples:
        return []
    kf = GravityKalman(process_noise, measurement_noise)
    result = []
    for s in samples:
        dx, dy, dz = kf.feed_remove(s.x, s.y, s.z)
        if skip_warmup and not kf.warmed_up:
            result.append(Sample(0.0, 0.0, 0.0))
        else:
            result.append(Sample(dx, dy, dz))
    return result


def low_pass(samples: Sequence, cutoff_hz: float, sample_rate: float,
             order: int = 2):
    """Butterworth low-pass filter. Returns list of Sample(x, y, z)."""
    if not samples or cutoff_hz <= 0 or sample_rate <= 0:
        return [Sample(s.x, s.y, s.z) for s in samples] if samples else []
    out = _biquad_filter_3ax(samples, *_biquad_coeffs_lp(cutoff_hz, sample_rate))
    if order >= 4:
        out = _biquad_filter_3ax(out, *_biquad_coeffs_lp(cutoff_hz, sample_rate))
    return out


def high_pass(samples: Sequence, cutoff_hz: float, sample_rate: float,
              order: int = 2):
    """Butterworth high-pass filter. Returns list of Sample(x, y, z)."""
    if not samples or cutoff_hz <= 0 or sample_rate <= 0:
        return [Sample(s.x, s.y, s.z) for s in samples] if samples else []
    out = _biquad_filter_3ax(samples, *_biquad_coeffs_hp(cutoff_hz, sample_rate))
    if order >= 4:
        out = _biquad_filter_3ax(out, *_biquad_coeffs_hp(cutoff_hz, sample_rate))
    return out


def bandpass(samples: Sequence, low_hz: float, high_hz: float,
             sample_rate: float, order: int = 2):
    """Cascaded high-pass + low-pass bandpass. Returns list of Sample(x, y, z)."""
    hp = high_pass(samples, low_hz, sample_rate, order=order)
    return low_pass(hp, high_hz, sample_rate, order=order)


def filtfilt_low_pass(samples: Sequence, cutoff_hz: float, sample_rate: float):
    """Zero-phase low-pass (forward-backward). Returns list of Sample(x, y, z)."""
    coeffs = _biquad_coeffs_lp(cutoff_hz, sample_rate)
    fwd = _biquad_filter_3ax(samples, *coeffs)
    rev = _biquad_filter_3ax(list(reversed(fwd)), *coeffs)
    rev.reverse()
    return rev


def filtfilt_high_pass(samples: Sequence, cutoff_hz: float, sample_rate: float):
    """Zero-phase high-pass (forward-backward). Returns list of Sample(x, y, z)."""
    coeffs = _biquad_coeffs_hp(cutoff_hz, sample_rate)
    fwd = _biquad_filter_3ax(samples, *coeffs)
    rev = _biquad_filter_3ax(list(reversed(fwd)), *coeffs)
    rev.reverse()
    return rev


def median_filter(samples: Sequence, window: int = 5):
    """Median filter for spike removal. Returns list of Sample(x, y, z)."""
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
        result.append(Sample(xs[mid], ys[mid], zs[mid]))
    return result


def peak_detect(values: Sequence[float], threshold: float,
                min_spacing: int = 10):
    """Detect peaks in a 1D signal. Returns list of (index, value)."""
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
    """Rolling RMS of magnitude. Returns list of float."""
    buf = []
    result = []
    for s in samples:
        m = s.x * s.x + s.y * s.y + s.z * s.z
        buf.append(m)
        if len(buf) > window:
            buf.pop(0)
        result.append(math.sqrt(sum(buf) / len(buf)))
    return result
