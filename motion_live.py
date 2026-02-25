#!/usr/bin/env python3
"""
demo app for spu_sensor.py - vibration detection, orientation gauges,
experimental heartbeat (bcg), lid angle & ambient light in a terminal dashboard
requires: sudo python3 motion_live.py
"""

import time
import sys
import os
import re
import json
import signal
import math
import datetime
import shutil
import subprocess
import pwd
import multiprocessing
import multiprocessing.shared_memory
from collections import deque

import struct

from macimu._spu import (
    sensor_worker, shm_read_new, shm_read_new_gyro, shm_snap_read,
    SHM_NAME, SHM_NAME_GYRO, SHM_SIZE,
    SHM_NAME_ALS, SHM_ALS_SIZE, SHM_NAME_LID, SHM_LID_SIZE,
    SHM_SNAP_HDR, ALS_REPORT_LEN,
)


RST = "\033[0m"
BOLD = "\033[1m"
DIM = "\033[2m"
RED = "\033[31m"
GRN = "\033[32m"
YEL = "\033[33m"
CYN = "\033[36m"
BRED = "\033[91m"
BGRN = "\033[92m"
BYEL = "\033[93m"
BCYN = "\033[96m"
BWHT = "\033[97m"
HIDE_CUR  = "\033[?25l"
SHOW_CUR  = "\033[?25h"
ENTER_ALT = "\033[?1049h"
EXIT_ALT  = "\033[?1049l"
CLEAR     = "\033[2J\033[H"

_ANSI_RE = re.compile(r'\033\[[^m]*m')


class VibrationDetector:
    def __init__(self, fs=100):
        self.fs = fs
        self.sample_count = 0

        # high-pass iir for gravity removal
        self.hp_alpha = 0.95
        self.hp_prev_raw = [0.0, 0.0, 0.0]
        self.hp_prev_out = [0.0, 0.0, 0.0]
        self.hp_ready = False

        N5 = fs * 5
        self.waveform = deque(maxlen=N5)
        self.waveform_xyz = deque(maxlen=N5)

        self.latest_raw = (0.0, 0.0, 0.0)
        self.latest_mag = 0.0

        # sta/lta at 3 timescales
        self.sta = [0.0, 0.0, 0.0]
        self.lta = [1e-10, 1e-10, 1e-10]
        self.sta_n = [3, 15, 50]
        self.lta_n = [100, 500, 2000]
        self.sta_lta_thresh_on = [3.0, 2.5, 2.0]
        self.sta_lta_thresh_off = [1.5, 1.3, 1.2]
        self.sta_lta_active = [False, False, False]
        SPARK_W = 30
        self.sta_lta_ring = [deque(maxlen=SPARK_W) for _ in range(3)]
        self.sta_lta_latest = [1.0, 1.0, 1.0]
        self._sta_dec = 0

        # dwt - 5 levels at 100hz
        self.dwt_buffer = deque(maxlen=512)
        SPEC_W = 50
        self.band_energy = [deque(maxlen=SPEC_W) for _ in range(5)]
        self.band_labels = ['50Hz', '25Hz', '12Hz', ' 6Hz', ' 3Hz']
        self._dwt_ok = False
        try:
            import pywt
            self._pywt = pywt
            self._dwt_ok = True
        except ImportError:
            self._pywt = None

        # cusum bilateral
        self.cusum_pos = 0.0
        self.cusum_neg = 0.0
        self.cusum_mu = 0.0
        self.cusum_k = 0.0005
        self.cusum_h = 0.01
        self.cusum_val = 0.0

        # kurtosis (1s window)
        self.kurt_buf = deque(maxlen=100)
        self.kurtosis = 3.0
        self._kurt_dec = 0

        # crest factor + mad peak (2s window)
        self.peak_buf = deque(maxlen=200)
        self.crest = 1.0
        self.rms = 0.0
        self.peak = 0.0
        self.mad_sigma = 0.0

        self.events = deque(maxlen=500)
        self.event_ts = deque(maxlen=200)

        # periodicity via autocorrelation
        self.period = None
        self.period_std = None
        self.period_cv = None
        self.period_freq = None
        self.acorr_ring = []

        # rms trend (10s, ~10hz output)
        self.rms_trend = deque(maxlen=100)
        self._rms_window = deque(maxlen=fs)
        self._rms_dec = 0

        # gyroscope latest values (deg/s)
        self.gyro_latest = (0.0, 0.0, 0.0)

        # Mahony AHRS — quaternion orientation (no gimbal lock)
        self._q = [1.0, 0.0, 0.0, 0.0]
        self._mahony_kp = 1.0
        self._mahony_ki = 0.05
        self._mahony_err_int = [0.0, 0.0, 0.0]
        self._orient_init = False

        # heartbeat bcg - bandpass 0.8-3hz via cascaded 1st order iir
        self.hr_hp_alpha = fs / (fs + 2.0 * math.pi * 0.8)
        self.hr_lp_alpha = 2.0 * math.pi * 3.0 / (2.0 * math.pi * 3.0 + fs)
        self.hr_hp_prev_in = 0.0
        self.hr_hp_prev_out = 0.0
        self.hr_lp_prev = 0.0
        self.hr_buf = deque(maxlen=fs * 10)
        self.hr_bpm = None
        self.hr_confidence = 0.0

        self._last_evt_t = 0.0

    def process_gyro(self, gx, gy, gz):
        self.gyro_latest = (gx, gy, gz)

    def _update_orientation(self, ax, ay, az):
        """Mahony AHRS filter: fuses accel (gravity) + gyro via quaternion."""
        a_norm = math.sqrt(ax * ax + ay * ay + az * az)
        if a_norm < 0.3:
            return

        gx = math.radians(self.gyro_latest[0])
        gy = math.radians(self.gyro_latest[1])
        gz = math.radians(self.gyro_latest[2])
        dt = 1.0 / self.fs

        if not self._orient_init:
            # bootstrap: align quaternion so that measured accel = -Z in body frame
            ax_n, ay_n, az_n = ax / a_norm, ay / a_norm, az / a_norm
            pitch0 = math.atan2(-ax_n, -az_n)
            roll0 = math.atan2(ay_n, -az_n)
            cp = math.cos(pitch0 * 0.5)
            sp = math.sin(pitch0 * 0.5)
            cr = math.cos(roll0 * 0.5)
            sr = math.sin(roll0 * 0.5)
            self._q = [
                cr * cp,
                sr * cp,
                cr * sp,
                -sr * sp,
            ]
            self._orient_init = True
            return

        q = self._q
        inv_norm = 1.0 / a_norm
        ax_n, ay_n, az_n = ax * inv_norm, ay * inv_norm, az * inv_norm

        # estimated gravity direction from current quaternion
        # (third column of rotation matrix transposed = R^T * [0,0,1])
        qw, qx, qy, qz = q
        vx = 2.0 * (qx * qz - qw * qy)
        vy = 2.0 * (qw * qx + qy * qz)
        vz = qw * qw - qx * qx - qy * qy + qz * qz

        # cross product: measured_accel × estimated_gravity → error
        # accel measures -gravity in body frame, so compare with -v
        ex = (ay_n * (-vz) - az_n * (-vy))
        ey = (az_n * (-vx) - ax_n * (-vz))
        ez = (ax_n * (-vy) - ay_n * (-vx))

        # PI correction
        self._mahony_err_int[0] += self._mahony_ki * ex * dt
        self._mahony_err_int[1] += self._mahony_ki * ey * dt
        self._mahony_err_int[2] += self._mahony_ki * ez * dt

        gx += self._mahony_kp * ex + self._mahony_err_int[0]
        gy += self._mahony_kp * ey + self._mahony_err_int[1]
        gz += self._mahony_kp * ez + self._mahony_err_int[2]

        # integrate quaternion derivative: q_dot = 0.5 * q ⊗ [0, gx, gy, gz]
        hdt = 0.5 * dt
        dw = (-qx * gx - qy * gy - qz * gz) * hdt
        dx = ( qw * gx + qy * gz - qz * gy) * hdt
        dy = ( qw * gy - qx * gz + qz * gx) * hdt
        dz = ( qw * gz + qx * gy - qy * gx) * hdt

        qw += dw; qx += dx; qy += dy; qz += dz

        # normalize
        n = math.sqrt(qw*qw + qx*qx + qy*qy + qz*qz)
        if n > 0:
            inv_n = 1.0 / n
            qw *= inv_n; qx *= inv_n; qy *= inv_n; qz *= inv_n

        self._q = [qw, qx, qy, qz]

    def process(self, ax, ay, az, t_now):
        self.sample_count += 1
        self.latest_raw = (ax, ay, az)
        self.latest_mag = math.sqrt(ax * ax + ay * ay + az * az)
        self._update_orientation(ax, ay, az)

        if not self.hp_ready:
            self.hp_prev_raw = [ax, ay, az]
            self.hp_prev_out = [0.0, 0.0, 0.0]
            self.hp_ready = True
            self.waveform.append(0.0)
            self.dwt_buffer.append(0.0)
            return 0.0

        a = self.hp_alpha
        hx = a * (self.hp_prev_out[0] + ax - self.hp_prev_raw[0])
        hy = a * (self.hp_prev_out[1] + ay - self.hp_prev_raw[1])
        hz = a * (self.hp_prev_out[2] + az - self.hp_prev_raw[2])
        self.hp_prev_raw = [ax, ay, az]
        self.hp_prev_out = [hx, hy, hz]
        mag = math.sqrt(hx * hx + hy * hy + hz * hz)

        self.waveform.append(mag)
        self.waveform_xyz.append((hx, hy, hz))
        self.dwt_buffer.append(mag)

        # heartbeat bandpass
        hp_out = self.hr_hp_alpha * (self.hr_hp_prev_out + mag - self.hr_hp_prev_in)
        self.hr_hp_prev_in = mag
        self.hr_hp_prev_out = hp_out
        lp_out = self.hr_lp_alpha * hp_out + (1.0 - self.hr_lp_alpha) * self.hr_lp_prev
        self.hr_lp_prev = lp_out
        self.hr_buf.append(lp_out)

        self._rms_window.append(mag)
        self._rms_dec += 1
        if self._rms_dec >= max(1, self.fs // 10):
            self._rms_dec = 0
            if self._rms_window:
                rv = math.sqrt(sum(x * x for x in self._rms_window) / len(self._rms_window))
                self.rms_trend.append(rv)

        evts = []

        # sta/lta
        e = mag * mag
        for i in range(3):
            self.sta[i] += (e - self.sta[i]) / self.sta_n[i]
            self.lta[i] += (e - self.lta[i]) / self.lta_n[i]
            ratio = self.sta[i] / (self.lta[i] + 1e-30)
            self.sta_lta_latest[i] = ratio
            was = self.sta_lta_active[i]
            if ratio > self.sta_lta_thresh_on[i] and not was:
                self.sta_lta_active[i] = True
                evts.append(('STA/LTA', i, ratio, mag))
            elif ratio < self.sta_lta_thresh_off[i]:
                self.sta_lta_active[i] = False

        self._sta_dec += 1
        if self._sta_dec >= max(1, self.fs // 30):
            self._sta_dec = 0
            for i in range(3):
                self.sta_lta_ring[i].append(self.sta_lta_latest[i])

        # cusum
        self.cusum_mu += 0.0001 * (mag - self.cusum_mu)
        self.cusum_pos = max(0.0, self.cusum_pos + mag - self.cusum_mu - self.cusum_k)
        self.cusum_neg = max(0.0, self.cusum_neg - mag + self.cusum_mu - self.cusum_k)
        self.cusum_val = max(self.cusum_pos, self.cusum_neg)
        if self.cusum_pos > self.cusum_h:
            evts.append(('CUSUM', 'pos', self.cusum_pos, mag))
            self.cusum_pos = 0.0
        if self.cusum_neg > self.cusum_h:
            evts.append(('CUSUM', 'neg', self.cusum_neg, mag))
            self.cusum_neg = 0.0

        # kurtosis
        self.kurt_buf.append(mag)
        self._kurt_dec += 1
        if self._kurt_dec >= 10 and len(self.kurt_buf) >= 50:
            self._kurt_dec = 0
            buf = list(self.kurt_buf)
            n = len(buf)
            mu = sum(buf) / n
            m2 = sum((x - mu) ** 2 for x in buf) / n
            m4 = sum((x - mu) ** 4 for x in buf) / n
            k = m4 / (m2 * m2 + 1e-30)
            self.kurtosis = k
            if k > 6:
                evts.append(('KURTOSIS', k, mag))

        # peak / mad
        self.peak_buf.append(mag)
        if len(self.peak_buf) >= 50 and self.sample_count % 10 == 0:
            srt = sorted(self.peak_buf)
            n = len(srt)
            median = srt[n // 2]
            mad = sorted(abs(x - median) for x in srt)[n // 2]
            sigma = 1.4826 * mad + 1e-30
            self.mad_sigma = sigma
            self.rms = math.sqrt(sum(x * x for x in self.peak_buf) / n)
            self.peak = max(abs(x) for x in self.peak_buf)
            self.crest = self.peak / (self.rms + 1e-30)
            dev = abs(mag - median) / sigma
            if dev > 8.0:
                evts.append(('PEAK', 'majeur', dev, mag))
            elif dev > 5.0:
                evts.append(('PEAK', 'fort', dev, mag))
            elif dev > 3.5:
                evts.append(('PEAK', 'moyen', dev, mag))
            elif dev > 2.0:
                evts.append(('PEAK', 'micro', dev, mag))

        if evts and (t_now - self._last_evt_t) > 0.01:
            self._last_evt_t = t_now
            self.event_ts.append(t_now)
            self._classify(evts, t_now, mag)

        return mag

    def compute_dwt(self):
        if not self._dwt_ok or len(self.dwt_buffer) < 64:
            return
        n = min(len(self.dwt_buffer), 512)
        data = list(self.dwt_buffer)[-n:]
        try:
            lvl = min(5, self._pywt.dwt_max_level(n, 'db4'))
            if lvl < 3:
                return
            coeffs = self._pywt.wavedec(data, 'db4', level=lvl)
            want = [5, 4, 3, 2, 1]
            for j, bi in enumerate(want):
                if bi < len(coeffs):
                    d = coeffs[bi]
                    eng = sum(v * v for v in d) / max(1, len(d))
                    self.band_energy[j].append(eng)
                else:
                    self.band_energy[j].append(0.0)
        except Exception:
            pass

    def detect_periodicity(self):
        if len(self.waveform) < self.fs * 2:
            self.period = None
            self.acorr_ring = []
            return
        buf = list(self.waveform)[-self.fs * 5:]
        n = len(buf)
        mean = sum(buf) / n
        centered = [x - mean for x in buf]
        var = sum(x * x for x in centered)
        if var < 1e-20:
            self.period = None
            self.acorr_ring = []
            return
        min_lag = max(5, int(self.fs * 0.05))
        max_lag = min(n // 2, int(self.fs * 2.5))
        acorr = []
        for lag in range(min_lag, max_lag):
            s = sum(centered[i] * centered[i + lag] for i in range(n - lag))
            acorr.append(s / var)
        self.acorr_ring = acorr
        if not acorr:
            self.period = None
            return
        best_i = max(range(len(acorr)), key=lambda i: acorr[i])
        best_val = acorr[best_i]
        best_lag = min_lag + best_i
        if best_val > 0.1:
            self.period = best_lag / self.fs
            self.period_freq = self.fs / best_lag
            self.period_cv = max(0.0, 1.0 - best_val)
            self.period_std = self.period * self.period_cv
        else:
            self.period = None
            self.period_freq = None
            self.period_cv = None
            self.period_std = None

    def detect_heartbeat(self):
        min_n = self.fs * 5
        if len(self.hr_buf) < min_n:
            self.hr_bpm = None
            self.hr_confidence = 0.0
            return
        buf = list(self.hr_buf)[-self.fs * 10:]
        n = len(buf)
        mean = sum(buf) / n
        centered = [x - mean for x in buf]
        var = sum(x * x for x in centered)
        if var < 1e-20:
            self.hr_bpm = None
            self.hr_confidence = 0.0
            return
        lag_lo = int(self.fs * 0.3)
        lag_hi = min(int(self.fs * 1.0), n // 2)
        if lag_lo >= lag_hi:
            self.hr_bpm = None
            self.hr_confidence = 0.0
            return
        best_r = -1.0
        best_lag = lag_lo
        for lag in range(lag_lo, lag_hi):
            s = sum(centered[i] * centered[i + lag] for i in range(n - lag))
            r = s / var
            if r > best_r:
                best_r = r
                best_lag = lag
        if best_r > 0.15:
            self.hr_bpm = 60.0 / (best_lag / self.fs)
            self.hr_confidence = min(1.0, best_r)
        else:
            self.hr_bpm = None
            self.hr_confidence = 0.0

    def _classify(self, detections, t, amp):
        sources = set(d[0] for d in detections)
        ns = len(sources)

        if ns >= 4 and amp > 0.05:
            sev, sym, lbl = 'CHOC_MAJEUR', '★', 'MAJOR'
        elif ns >= 3 and amp > 0.02:
            sev, sym, lbl = 'CHOC_MOYEN', '▲', 'shock'
        elif 'PEAK' in sources and amp > 0.005:
            sev, sym, lbl = 'MICRO_CHOC', '△', 'micro-choc'
        elif ('STA/LTA' in sources or 'CUSUM' in sources) and amp > 0.003:
            sev, sym, lbl = 'VIBRATION', '●', 'vibration'
        elif amp > 0.001:
            sev, sym, lbl = 'VIB_LEGERE', '○', 'light-vib'
        else:
            sev, sym, lbl = 'MICRO_VIB', '·', 'micro-vib'

        bands = []
        for j in range(5):
            if self.band_energy[j]:
                recent = list(self.band_energy[j])[-3:]
                if sum(recent) / len(recent) > 1e-10:
                    bands.append(self.band_labels[j].strip())

        self.events.append({
            'time': t,
            'tstr': datetime.datetime.fromtimestamp(t).strftime('%H:%M:%S.%f')[:11],
            'sev': sev, 'sym': sym, 'lbl': lbl,
            'amp': amp,
            'src': list(sources),
            'nsrc': ns,
            'bands': bands,
        })


class KeyboardFlashBridge:
    def __init__(self, enabled=True, binary_override=None, fade_ms=20, send_hz=80, run_as_user=True):
        self.enabled = enabled
        self.binary_override = binary_override
        self.fade_ms = max(0, int(fade_ms))
        self.send_dt = 1.0 / max(1.0, float(send_hz))
        self.run_as_user = run_as_user

        self.proc = None
        self.state = 'off'
        self.err = ''
        self.bin_path = None
        self.run_user = None

        self.level = 0.0
        self._last_update = None
        self._next_send = 0.0
        self._last_sent = -1.0

        self._noise = 0.0002
        self._peak = 0.0015
        self._decay_per_s = 7.0

    def _resolve_binary(self):
        candidates = []
        if self.binary_override:
            candidates.append(self.binary_override)
        env_bin = os.environ.get('KBPULSE_BIN')
        if env_bin:
            candidates.append(env_bin)

        cwd = os.getcwd()
        base = os.path.dirname(os.path.abspath(__file__))
        candidates.extend([
            os.path.join(cwd, 'KBPulse', 'bin', 'KBPulse'),
            os.path.join(base, 'KBPulse', 'bin', 'KBPulse'),
            os.path.join(base, '.localbin', 'KBPulse'),
            os.path.join(base, 'KBPulse', 'build', 'Release', 'KBPulse'),
            os.path.join(base, 'KBPulse', 'build', 'Debug', 'KBPulse'),
            os.path.join(base, 'KBPulse', 'KBPulse'),
            shutil.which('KBPulse'),
            shutil.which('kbpulse'),
        ])

        for c in candidates:
            if not c:
                continue
            if os.path.isfile(c) and os.access(c, os.X_OK):
                return c
        return None

    def start(self):
        if not self.enabled:
            self.state = 'disabled'
            return

        bin_path = self._resolve_binary()
        if not bin_path:
            self.enabled = False
            self.state = 'missing'
            self.err = 'set KBPULSE_BIN or build KBPulse'
            return

        self.bin_path = bin_path
        cmd = [bin_path, '--stdin-intensity', '--fade-ms', str(self.fade_ms)]
        sudo_user = os.environ.get('SUDO_USER')
        if self.run_as_user and os.geteuid() == 0 and sudo_user and sudo_user != 'root':
            try:
                pwd.getpwnam(sudo_user)
                cmd = ['sudo', '-u', sudo_user] + cmd
                self.run_user = sudo_user
            except KeyError:
                self.run_user = None

        try:
            self.proc = subprocess.Popen(
                cmd,
                stdin=subprocess.PIPE,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.PIPE,
                text=True,
                bufsize=1,
            )
            self.state = 'on'
        except Exception as e:
            self.enabled = False
            self.state = 'error'
            self.err = str(e)

    def _write_level(self, value, t_now):
        if not self.proc or not self.proc.stdin:
            return
        try:
            self.proc.stdin.write(f'{value:.4f}\n')
            self.proc.stdin.flush()
            self._last_sent = value
            self._next_send = t_now + self.send_dt
        except Exception as e:
            self.enabled = False
            self.state = 'broken-pipe'
            self.err = str(e)

    def update(self, mag, t_now):
        if not self.enabled or self.proc is None:
            return
        if self.proc.poll() is not None:
            self.enabled = False
            self.state = 'stopped'
            if self.proc.stderr:
                try:
                    self.err = (self.proc.stderr.read() or '').strip()[:120]
                except Exception:
                    pass
            return

        if self._last_update is None:
            dt = 0.0
        else:
            dt = max(0.0, t_now - self._last_update)
        self._last_update = t_now

        self._noise += 0.002 * (mag - self._noise)
        if mag > self._peak:
            self._peak += 0.15 * (mag - self._peak)
        else:
            self._peak += 0.01 * (mag - self._peak)

        span = max(1e-6, self._peak - self._noise)
        norm = (mag - self._noise) / span
        norm = max(0.0, min(1.0, norm))

        pulse = norm ** 0.65
        decay = math.exp(-self._decay_per_s * dt) if dt > 0 else 1.0
        self.level = max(pulse, self.level * decay)

        if t_now < self._next_send:
            return

        self._write_level(self.level, t_now)

    def status(self):
        if self.state == 'on':
            user = self.run_user or 'root'
            return f'KBPulse:on({user}) lvl:{self.level:0.2f} fade:{self.fade_ms}ms'
        if self.err:
            return f'KBPulse:{self.state} ({self.err[:34]})'
        return f'KBPulse:{self.state}'

    def stop(self):
        if not self.proc:
            return
        self._write_level(0.0, time.time())
        try:
            if self.proc.stdin:
                self.proc.stdin.close()
        except Exception:
            pass
        if self.proc.poll() is None:
            self.proc.terminate()
            try:
                self.proc.wait(timeout=0.4)
            except subprocess.TimeoutExpired:
                self.proc.kill()
        self.proc = None
        if self.state == 'on':
            self.state = 'off'


# --- terminal ui ---

W = 76
BLOCKS = ' ▁▂▃▄▅▆▇█'

def _gauge(value, vmin, vmax, width):
    """Horizontal gauge: ─ bar with ┼ at zero and ● at value position."""
    rng = vmax - vmin
    if rng == 0:
        rng = 1.0
    t = max(0.0, min(1.0, (value - vmin) / rng))
    pos = int(t * (width - 1))
    center = int((0.0 - vmin) / rng * (width - 1))
    bar = ['─'] * width
    if 0 <= center < width:
        bar[center] = '┼'
    bar[max(0, min(width - 1, pos))] = '●'
    return ''.join(bar)


def _lid_text(angle):
    return f'  {BWHT}{angle:.0f}°{RST}'


_ALS_SPEC_OFFSETS = [20, 24, 28, 32]
_ALS_LUX_OFF = 40
_ALS_BLOCKS = ' ▁▂▃▄▅▆▇█'
_SPECTRUM_KEYS = [
    (0.00, 120, 40, 220), (0.20, 40, 100, 220), (0.40, 30, 190, 190),
    (0.60, 50, 210, 50),  (0.80, 210, 210, 30), (1.00, 230, 60, 30),
]

def _spec_rgb(t):
    for i in range(len(_SPECTRUM_KEYS) - 1):
        t0, r0, g0, b0 = _SPECTRUM_KEYS[i]
        t1, r1, g1, b1 = _SPECTRUM_KEYS[i + 1]
        if t <= t1:
            f = (t - t0) / (t1 - t0) if t1 > t0 else 0.0
            return int(r0+(r1-r0)*f), int(g0+(g1-g0)*f), int(b0+(b1-b0)*f)
    return _SPECTRUM_KEYS[-1][1], _SPECTRUM_KEYS[-1][2], _SPECTRUM_KEYS[-1][3]

def _als_bar(raw, width):
    if raw is None or len(raw) < 44:
        return [f'  {DIM}waiting for ALS data...{RST}', '', '']

    intensity = max(0.0, min(1.0, struct.unpack_from('<f', raw, _ALS_LUX_OFF)[0]))
    ch = [struct.unpack_from('<I', raw, o)[0] for o in _ALS_SPEC_OFFSETS]
    ch_max = max(ch) if max(ch) > 0 else 1
    ch_norm = [v / ch_max for v in ch]

    heights = []
    nc = len(ch_norm)
    for i in range(width):
        t = i / max(1, width - 1) * (nc - 1)
        lo = min(int(t), nc - 2)
        frac = t - lo
        heights.append(ch_norm[lo] * (1 - frac) + ch_norm[lo + 1] * frac)

    curve = ''
    for i in range(width):
        lvl = max(0, min(8, int(heights[i] * 8.99)))
        r, g, b = _spec_rgb(i / max(1, width - 1))
        curve += f'\033[38;2;{r};{g};{b}m{_ALS_BLOCKS[lvl]}'
    curve += RST

    filled = max(1, int(intensity * width)) if intensity > 0.005 else 0
    bar = ''
    for i in range(width):
        r, g, b = _spec_rgb(i / max(1, width - 1))
        if i < filled:
            bar += f'\033[48;2;{r};{g};{b}m '
        else:
            bar += f'\033[48;2;25;25;35m '
    bar += RST

    return [
        f'  {curve}',
        f'  {bar}  {BWHT}{intensity:.3f}{RST} {DIM}lux{RST}',
        f'  {DIM}ch: {" ".join(str(v) for v in ch)}{RST}',
    ]


def _vlen(s):
    return len(_ANSI_RE.sub('', s))


def _sparkline(data, width, ceil=None):
    if not data:
        return ' ' * width
    d = list(data)
    if len(d) < width:
        d = [0.0] * (width - len(d)) + d
    elif len(d) > width:
        d = d[-width:]
    if ceil is None or ceil <= 0:
        ceil = max(abs(v) for v in d) if d else 1.0
    if ceil <= 0:
        ceil = 1.0
    out = []
    for v in d:
        frac = min(1.0, abs(v) / ceil)
        out.append(BLOCKS[min(8, int(frac * 8))])
    return ''.join(out)


def _spec_row(data, width, floor_db=-60, ceil_db=-10):
    chars = ' ·░▒▓█'
    if not data:
        return ' ' * width
    d = list(data)
    if len(d) < width:
        d = [0.0] * (width - len(d)) + d
    elif len(d) > width:
        d = d[-width:]
    out = []
    rng = ceil_db - floor_db
    for e in d:
        if e <= 0:
            out.append(' ')
            continue
        db = 10 * math.log10(e + 1e-20)
        frac = max(0.0, min(1.0, (db - floor_db) / rng))
        out.append(chars[min(5, int(frac * 5))])
    return ''.join(out)


def _sev_color(sev):
    return {
        'CHOC_MAJEUR': f'{BRED}{BOLD}',
        'CHOC_MOYEN': RED,
        'MICRO_CHOC': CYN,
        'VIBRATION': YEL,
        'VIB_LEGERE': GRN,
        'MICRO_VIB': DIM,
    }.get(sev, DIM)


def _line(content):
    vl = _vlen(content)
    pad = max(0, W - vl)
    return f"{DIM}│{RST}{content}{' ' * pad}{DIM}│{RST}"


def _sep(label=''):
    if label:
        rest = W - _vlen(label) - 1
        return f"{DIM}├─{label}{'─' * rest}┤{RST}"
    return f"{DIM}├{'─' * W}┤{RST}"


def _downsample(data, width):
    n = len(data)
    if n <= width:
        return list(data)
    step = n / width
    out = []
    for c in range(width):
        s_i = int(c * step)
        e_i = int((c + 1) * step)
        chunk = data[s_i:e_i]
        out.append(max(chunk) if chunk else 0.0)
    return out


def render(det, t_start, restarts, kbflash=None,
           lid_angle=None, als_raw=None):
    el = time.time() - t_start
    rate = det.sample_count / el if el > 1 else 0
    now = time.time()

    L = []
    a = L.append

    title = ' MOTION DETECTOR '
    top_bar = '─' * (W - len(title) - 1)
    a(f"{DIM}┌─{RST}{BWHT}{title}{RST}{DIM}{top_bar}┐{RST}")

    hdr = (f" {DIM}{el:>7.1f}s{RST}  {det.sample_count:>10,} smp  "
           f"{BWHT}{rate:>.0f}{RST} Hz  "
           f"R:{restarts}  Ev:{len(det.events)}")
    a(_line(hdr))

    GW = W - 4

    a(_sep(' Waveform |a_dyn| 5s '))
    wd = list(det.waveform)
    if wd:
        mx = max(max(abs(v) for v in wd), 0.0002)
        ds = _downsample(wd, GW)
        a(_line(f"  {GRN}{_sparkline(ds, GW, mx)}{RST}"))
        a(_line(f"  {DIM}{mx:.5f}g{' ' * (GW - 22)}0g{RST}"))
    else:
        a(_line(f"  {DIM}waiting...{RST}"))
        a(_line(''))

    a(_sep(' Axes X / Y / Z (5s) '))
    xyz = list(det.waveform_xyz)
    AW = GW - 4
    if xyz:
        xs = [t[0] for t in xyz]
        ys = [t[1] for t in xyz]
        zs = [t[2] for t in xyz]
        amx = max(max(abs(v) for v in xs + ys + zs), 0.0001)
        a(_line(f"  {RED}X{RST} {_sparkline(_downsample(xs, AW), AW, amx)}{RST}"))
        a(_line(f"  {GRN}Y{RST} {_sparkline(_downsample(ys, AW), AW, amx)}{RST}"))
        a(_line(f"  {CYN}Z{RST} {_sparkline(_downsample(zs, AW), AW, amx)}{RST}"))
    else:
        for ax_l in ('X', 'Y', 'Z'):
            a(_line(f"  {DIM}{ax_l}{RST}"))

    a(_sep(' Spectrogram DWT 5s '))
    SW = W - 10
    has_dwt = det._dwt_ok and any(len(b) > 0 for b in det.band_energy)
    if has_dwt:
        for j in range(5):
            row = _spec_row(list(det.band_energy[j]), SW)
            a(_line(f" {DIM}{det.band_labels[j]}{RST} {CYN}{row}{RST}"))
    else:
        msg = 'pip install PyWavelets' if not det._dwt_ok else 'accumulating...'
        a(_line(f"  {DIM}{msg}{RST}"))
        for _ in range(4):
            a(_line(''))

    a(_sep(' RMS trend 10s '))
    if det.rms_trend:
        a(_line(f"  {YEL}{_sparkline(list(det.rms_trend), GW)}{RST}"))
    else:
        a(_line(f"  {DIM}accumulating...{RST}"))

    a(_sep(' Detectors '))
    DW = 25
    names = ['fast', 'med ', 'slow']
    for i in range(3):
        sp = _sparkline(list(det.sta_lta_ring[i]), DW,
                        ceil=det.sta_lta_thresh_on[i] * 2)
        r = det.sta_lta_latest[i]
        thr = det.sta_lta_thresh_on[i]
        mark = '*' if r > thr else ' '
        col = BRED if r > thr else DIM
        if i == 0:
            extra = f"  K:{det.kurtosis:>5.1f}  CF:{det.crest:>5.1f}"
        elif i == 1:
            extra = f"  CUSUM:{det.cusum_val:>8.4f}"
        else:
            extra = f"  RMS:{det.rms:.5f}g Pk:{det.peak:.5f}g"
        a(_line(f" {DIM}STA {names[i]}{RST} {YEL}{sp}{RST}"
                f" {col}{r:>5.1f}{mark}{RST}{extra}"))

    a(_sep(' Autocorrelation (lag 0.05-2.5s) '))
    if det.acorr_ring:
        ac_ceil = max(0.05, max(abs(v) for v in det.acorr_ring) * 1.2)
        a(_line(f"  {BCYN}{_sparkline(det.acorr_ring, GW, ceil=ac_ceil)}{RST}"))
    else:
        a(_line(f"  {DIM}accumulating...{RST}"))

    a(_sep(' Pattern '))
    if det.period is not None and det.period_cv is not None and det.period_cv < 0.5:
        reg = max(0, min(100, int((1.0 - det.period_cv) * 100)))
        a(_line(f" Period:{det.period:.3f}s ±{det.period_std:.3f}"
                f"  Freq:{det.period_freq:.2f}Hz  Reg:{reg}%"))
        syms = ''.join(f"──{e['sym']}" for e in list(det.events)[-12:])
        a(_line(f" {DIM}{syms}──{RST}"))
    else:
        a(_line(f" {DIM}no regular pattern detected{RST}"))
        a(_line(''))

    hr_active = det.hr_bpm is not None and det.hr_confidence > 0.15
    if hr_active:
        bpm = det.hr_bpm
        period_s = 60.0 / bpm
        phase = (now % period_s) < (period_s * 0.3)
        hb_sym = f"{BRED}❤{RST}{DIM}" if phase else f"♡"
        a(_sep(f' Heartbeat BCG {hb_sym} '))
    else:
        a(_sep(' Heartbeat BCG '))
    if hr_active:
        conf = int(det.hr_confidence * 100)
        heart = f"{BRED}♥{RST}" if phase else f"{DIM}♡{RST}"
        a(_line(f" {heart} {BRED}{BOLD}{bpm:>5.1f} BPM{RST}"
                f"   confidence: {conf}%   band: 0.8-3Hz"))
        n_beats = max(1, int(GW / 3))
        beat_line = ''
        for b in range(n_beats):
            bp = ((now + b * period_s * 0.3) % period_s) < (period_s * 0.3)
            beat_line += f"{BRED}♥{RST}─" if bp else f"{DIM}♡{RST}─"
        a(_line(f" {beat_line}"))
    else:
        a(_line(f" {DIM}no heartbeat detected (rest wrists on laptop){RST}"))
        a(_line(''))

    a(_sep(' Orientation '))
    qw, qx, qy, qz = det._q
    sin_r = 2.0 * (qw * qx + qy * qz)
    cos_r = 1.0 - 2.0 * (qx * qx + qy * qy)
    roll_d = math.degrees(math.atan2(sin_r, cos_r))
    sin_p = 2.0 * (qw * qy - qz * qx)
    sin_p = max(-1.0, min(1.0, sin_p))
    pitch_d = math.degrees(math.asin(sin_p))
    sin_y = 2.0 * (qw * qz + qx * qy)
    cos_y = 1.0 - 2.0 * (qy * qy + qz * qz)
    yaw_d = math.degrees(math.atan2(sin_y, cos_y))
    gw = W - 18
    a(_line(f' {DIM}Roll {RST} {CYN}{_gauge(roll_d, -180, 180, gw)}{RST} {roll_d:>+7.1f}°'))
    a(_line(f' {DIM}Pitch{RST} {CYN}{_gauge(pitch_d, -90, 90, gw)}{RST} {pitch_d:>+7.1f}°'))
    a(_line(f' {DIM}Yaw  {RST} {CYN}{_gauge(yaw_d, -180, 180, gw)}{RST} {yaw_d:>+7.1f}°'))
    gx_v, gy_v, gz_v = det.gyro_latest
    a(_line(f' {DIM}ω: {gx_v:>+6.2f}  {gy_v:>+6.2f}  {gz_v:>+6.2f} °/s{RST}'))

    a(_sep(' Lid Angle '))
    if lid_angle is not None:
        a(_line(_lid_text(lid_angle)))
    else:
        a(_line(f'  {DIM}no lid data{RST}'))

    a(_sep(' Ambient Light '))
    for al in _als_bar(als_raw, W - 13):
        a(_line(al))

    a(_sep(' Events '))
    recent = list(det.events)[-5:]
    for ev in reversed(recent):
        c = _sev_color(ev['sev'])
        bands = ','.join(ev['bands'][:3]) if ev['bands'] else '-'
        a(_line(f" {DIM}{ev['tstr']}{RST} {c}{ev['sym']} {ev['lbl']:<11}{RST}"
                f" {ev['amp']:.5f}g {bands}"))
    for _ in range(max(0, 3 - len(recent))):
        a(_line(''))

    a(_sep())
    ax, ay, az = det.latest_raw
    a(_line(f" X:{ax:>+10.6f}g Y:{ay:>+10.6f}g Z:{az:>+10.6f}g"
            f"  |g|:{det.latest_mag:.6f}"))
    if kbflash is not None:
        a(_line(f" {DIM}{kbflash.status()}{RST}"))
    a(_line(f" {DIM}ctrl+c to save & quit{RST}"))
    a(f"{DIM}└{'─' * W}┘{RST}")

    return '\n'.join(L)


def main():
    use_kbpulse = True
    kbpulse_bin = None
    kbpulse_as_root = False
    i = 1
    while i < len(sys.argv):
        arg = sys.argv[i]
        if arg == '--no-kbpulse':
            use_kbpulse = False
        elif arg == '--kbpulse-bin' and i + 1 < len(sys.argv):
            kbpulse_bin = sys.argv[i + 1]
            i += 1
        elif arg == '--kbpulse-as-root':
            kbpulse_as_root = True
        elif arg in ('-h', '--help'):
            print(f'usage: sudo python3 {sys.argv[0]} [--no-kbpulse] [--kbpulse-bin /path/to/KBPulse] [--kbpulse-as-root]')
            return
        i += 1

    if os.geteuid() != 0:
        print(f"\033[91m\033[1m[!] run with: sudo python3 {sys.argv[0]}\033[0m")
        sys.exit(1)

    all_shms = [
        (SHM_NAME, SHM_SIZE), (SHM_NAME_GYRO, SHM_SIZE),
        (SHM_NAME_ALS, SHM_ALS_SIZE), (SHM_NAME_LID, SHM_LID_SIZE),
    ]
    for name, _ in all_shms:
        try:
            old = multiprocessing.shared_memory.SharedMemory(name=name, create=False)
            old.close()
            old.unlink()
        except FileNotFoundError:
            pass

    shm = multiprocessing.shared_memory.SharedMemory(
        name=SHM_NAME, create=True, size=SHM_SIZE)
    for i in range(SHM_SIZE):
        shm.buf[i] = 0

    shm_gyro = multiprocessing.shared_memory.SharedMemory(
        name=SHM_NAME_GYRO, create=True, size=SHM_SIZE)
    for i in range(SHM_SIZE):
        shm_gyro.buf[i] = 0

    shm_als = multiprocessing.shared_memory.SharedMemory(
        name=SHM_NAME_ALS, create=True, size=SHM_ALS_SIZE)
    for i in range(SHM_ALS_SIZE):
        shm_als.buf[i] = 0

    shm_lid = multiprocessing.shared_memory.SharedMemory(
        name=SHM_NAME_LID, create=True, size=SHM_LID_SIZE)
    for i in range(SHM_LID_SIZE):
        shm_lid.buf[i] = 0

    running = [True]
    restart_count = [0]

    def _stop(sig, frame):
        running[0] = False
    signal.signal(signal.SIGINT, _stop)
    signal.signal(signal.SIGTERM, _stop)

    sys.stdout.write(ENTER_ALT + HIDE_CUR)
    sys.stdout.flush()

    det = VibrationDetector(fs=100)
    t_start = time.time()
    last_total = 0
    last_gyro_total = 0
    last_als_count = 0
    last_lid_count = 0
    lid_angle = None
    als_raw = None
    last_draw = 0.0
    last_dwt = 0.0
    last_period = 0.0
    worker = None
    MAX_BATCH = 200
    kbflash = KeyboardFlashBridge(
        enabled=use_kbpulse,
        binary_override=kbpulse_bin,
        fade_ms=20,
        send_hz=80,
        run_as_user=(not kbpulse_as_root),
    )
    kbflash.start()

    try:
        while running[0]:
            if worker is None or not worker.is_alive():
                if worker is not None:
                    restart_count[0] += 1
                worker = multiprocessing.Process(
                    target=sensor_worker,
                    args=(SHM_NAME, restart_count[0]),
                    kwargs={
                        'gyro_shm_name': SHM_NAME_GYRO,
                        'als_shm_name': SHM_NAME_ALS,
                        'lid_shm_name': SHM_NAME_LID,
                    },
                    daemon=True)
                worker.start()

            time.sleep(0.005)
            now = time.time()

            samples, last_total = shm_read_new(shm.buf, last_total)
            if len(samples) > MAX_BATCH:
                samples = samples[-MAX_BATCH:]
            n_samples = len(samples)
            for idx, (sx, sy, sz) in enumerate(samples):
                t_sample = now - (n_samples - idx - 1) / det.fs
                dyn_mag = det.process(sx, sy, sz, t_sample)
                kbflash.update(dyn_mag, t_sample)

            gyro_samples, last_gyro_total = shm_read_new_gyro(
                shm_gyro.buf, last_gyro_total)
            if len(gyro_samples) > MAX_BATCH:
                gyro_samples = gyro_samples[-MAX_BATCH:]
            for (gx, gy, gz) in gyro_samples:
                det.process_gyro(gx, gy, gz)

            als_data, last_als_count = shm_snap_read(
                shm_als.buf, last_als_count, ALS_REPORT_LEN)
            if als_data is not None:
                als_raw = als_data

            lid_data, last_lid_count = shm_snap_read(
                shm_lid.buf, last_lid_count, 4)
            if lid_data is not None:
                lid_angle = struct.unpack('<f', lid_data)[0]

            if now - last_dwt >= 0.2:
                det.compute_dwt()
                last_dwt = now

            if now - last_period >= 1.0:
                det.detect_periodicity()
                det.detect_heartbeat()
                last_period = now

            if now - last_draw >= 0.1:
                frame = render(det, t_start, restart_count[0], kbflash=kbflash,
                              lid_angle=lid_angle,
                              als_raw=als_raw)
                sys.stdout.write(CLEAR + frame)
                sys.stdout.flush()
                last_draw = now

    finally:
        kbflash.stop()

        if worker and worker.is_alive():
            worker.kill()
            worker.join(timeout=2)

        sys.stdout.write(SHOW_CUR + EXIT_ALT + '\n')
        sys.stdout.flush()

        ts = datetime.datetime.now().strftime('%Y%m%d_%H%M%S')
        logpath = f'vibration_log_{ts}.json'
        print(f"{DIM}[*] saving {len(det.events)} events to {logpath}{RST}")
        obj = {
            'generated': datetime.datetime.now().isoformat(),
            'restarts': restart_count[0],
            'total_samples': det.sample_count,
            'events': [{
                'time': e['tstr'], 'severity': e['sev'],
                'amplitude': round(e['amp'], 6),
                'sources': e['src'], 'bands': e['bands'],
            } for e in det.events],
        }
        with open(logpath, 'w') as f:
            json.dump(obj, f, indent=1, default=str)
        print(f"{DIM}[ok] {det.sample_count} samples, "
              f"{restart_count[0]} restarts{RST}")

        for s in (shm, shm_gyro, shm_als, shm_lid):
            s.close()
            s.unlink()


if __name__ == '__main__':
    main()
