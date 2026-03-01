"""macimu - read the undocumented IMU on apple silicon macs via iokit hid"""

from __future__ import annotations

__version__ = "0.2.0"

import atexit
import csv
import math
import os
import struct
import time
import threading
import multiprocessing.shared_memory
from collections import namedtuple
from typing import Callable, Generator, Optional

Sample = namedtuple('Sample', ['x', 'y', 'z'])
TimedSample = namedtuple('TimedSample', ['t', 'x', 'y', 'z'])
ALSReading = namedtuple('ALSReading', ['lux', 'channels'])
Orientation = namedtuple('Orientation', ['roll', 'pitch', 'yaw', 'qw', 'qx', 'qy', 'qz'])

from ._spu import (
    sensor_worker,
    shm_read_new, shm_read_new_gyro,
    shm_read_new_accel_timed, shm_read_new_gyro_timed,
    shm_snap_read, check_available, get_device_info,
    SHM_NAME, SHM_NAME_GYRO, SHM_SIZE,
    SHM_NAME_ALS, SHM_ALS_SIZE, SHM_NAME_LID, SHM_LID_SIZE,
    SHM_SNAP_HDR, ALS_REPORT_LEN,
    ACCEL_SCALE, GYRO_SCALE,
    IMU_DECIMATION,
)
from .orientation import MahonyAHRS

_ALS_LUX_OFF = 40
_ALS_CH_OFFSETS = [20, 24, 28, 32]
_NATIVE_RATE_HZ = 800


class SensorNotFound(RuntimeError):
    """Raised when no SPU IMU device is found on this machine."""
    pass


class IMU:
    """High-level interface to the Apple Silicon SPU IMU.

    Requires root privileges (sudo). Spawns a background worker thread
    that reads HID reports and writes samples to shared memory.

    Parameters
    ----------
    accel : bool
        Enable accelerometer (default True).
    gyro : bool
        Enable gyroscope (default True).
    als : bool
        Enable ambient light sensor (default False).
    lid : bool
        Enable lid angle sensor (default False).
    orientation : bool
        Enable real-time orientation fusion via Mahony AHRS (default False).
        Requires both accel and gyro to be enabled.
    decimation : int or None
        Keep 1 in N raw HID reports. Lower = higher sample rate.
        Default 8 gives ~100 Hz from ~800 Hz native. Mutually exclusive
        with sample_rate.
    sample_rate : int or None
        Desired sample rate in Hz (e.g. 200). Computes decimation
        internally. Mutually exclusive with decimation.
    """

    def __init__(self, accel: bool = True, gyro: bool = True,
                 als: bool = False, lid: bool = False,
                 orientation: bool = False, decimation: Optional[int] = None,
                 sample_rate: Optional[int] = None) -> None:
        if decimation is not None and sample_rate is not None:
            raise ValueError("specify decimation or sample_rate, not both")
        if sample_rate is not None:
            decimation = max(1, round(_NATIVE_RATE_HZ / sample_rate))
        if decimation is None:
            decimation = IMU_DECIMATION

        self._want_accel = accel
        self._want_gyro = gyro
        self._want_als = als
        self._want_lid = lid
        self._want_orient = orientation
        self._decimation = decimation
        self._started = False
        self._worker: Optional[threading.Thread] = None
        self._shutdown = threading.Event()
        self._shms: list = []
        self._shm_accel = None
        self._shm_gyro = None
        self._shm_als = None
        self._shm_lid = None
        self._last_accel_total = 0
        self._last_gyro_total = 0
        self._last_als_count = 0
        self._last_lid_count = 0
        self._start_time = 0.0
        self._sample_count = 0

        self._ahrs: Optional[MahonyAHRS] = None
        self._orient_thread: Optional[threading.Thread] = None
        self._orient_stop = threading.Event()
        self._orient_lock = threading.Lock()
        self._last_orient: Optional[Orientation] = None

        self._recording_file = None
        self._recording_writer = None
        self._mock = False
        self._mock_data: list = []
        self._mock_idx = 0

        if orientation and not (accel and gyro):
            raise ValueError("orientation=True requires both accel=True and gyro=True")

    @classmethod
    def available(cls) -> bool:
        """Check if the SPU IMU is present on this machine (no root needed)."""
        return check_available()

    @classmethod
    def device_info(cls) -> dict:
        """Return SPU device metadata (no root needed).

        Returns dict with 'sensors' list and any available properties
        like Product, SerialNumber, Manufacturer, etc.
        """
        return get_device_info()

    @classmethod
    def mock(cls, duration: float = 60.0, rate: int = 100,
             noise: float = 0.01) -> IMU:
        """Create a mock IMU with synthetic data (no root needed).

        Generates sinusoidal acceleration + noise for testing.
        """
        instance = cls.__new__(cls)
        instance.__init__(accel=True, gyro=True)
        instance._mock = True
        samples = []
        dt = 1.0 / rate
        for i in range(int(duration * rate)):
            t = i * dt
            ax = noise * (hash(('ax', i)) % 1000 / 500 - 1)
            ay = noise * (hash(('ay', i)) % 1000 / 500 - 1)
            az = -1.0 + noise * (hash(('az', i)) % 1000 / 500 - 1)
            gx = 0.5 * math.sin(2 * math.pi * 0.2 * t)
            gy = 0.3 * math.cos(2 * math.pi * 0.15 * t)
            gz = noise * (hash(('gz', i)) % 1000 / 500 - 1)
            samples.append((t, ax, ay, az, gx, gy, gz))
        instance._mock_data = samples
        instance._mock_idx = 0
        instance._started = True
        instance._start_time = time.monotonic()
        instance._decimation = max(1, round(_NATIVE_RATE_HZ / rate))
        return instance

    @classmethod
    def from_recording(cls, path: str) -> IMU:
        """Create an IMU that replays data from a CSV file (no root needed).

        CSV format: t,sensor,x,y,z (sensor = 'accel' or 'gyro').
        """
        instance = cls.__new__(cls)
        instance.__init__(accel=True, gyro=True)
        instance._mock = True
        samples = []
        with open(path, newline='') as f:
            reader = csv.reader(f)
            for row in reader:
                if len(row) >= 5 and row[0] != 't':
                    samples.append((
                        float(row[0]), row[1],
                        float(row[2]), float(row[3]), float(row[4])))
        instance._mock_data = samples
        instance._mock_idx = 0
        instance._started = True
        instance._start_time = time.monotonic()
        return instance

    @property
    def is_running(self) -> bool:
        """True if the sensor worker is active."""
        return self._started

    @property
    def effective_sample_rate(self) -> Optional[float]:
        """Measured sample rate in Hz, or None if not enough data."""
        elapsed = time.monotonic() - self._start_time
        if elapsed < 0.5 or self._sample_count < 10:
            return None
        return self._sample_count / elapsed

    def start(self) -> None:
        """Start the sensor worker thread. Requires root."""
        if self._started:
            return
        if self._mock:
            return
        if os.geteuid() != 0:
            raise PermissionError("macimu requires root -- run with sudo")
        if not check_available():
            raise SensorNotFound(
                "no SPU IMU found -- this machine may not have the sensor")

        self._cleanup_stale_shm()

        self._shm_accel = self._create_shm(SHM_NAME, SHM_SIZE)
        kwargs = {'decimation': self._decimation, 'shutdown_event': self._shutdown}

        if self._want_gyro:
            self._shm_gyro = self._create_shm(SHM_NAME_GYRO, SHM_SIZE)
            kwargs['gyro_shm_name'] = SHM_NAME_GYRO
        if self._want_als:
            self._shm_als = self._create_shm(SHM_NAME_ALS, SHM_ALS_SIZE)
            kwargs['als_shm_name'] = SHM_NAME_ALS
        if self._want_lid:
            self._shm_lid = self._create_shm(SHM_NAME_LID, SHM_LID_SIZE)
            kwargs['lid_shm_name'] = SHM_NAME_LID

        self._shutdown.clear()
        self._worker = threading.Thread(
            target=sensor_worker,
            args=(SHM_NAME, 0),
            kwargs=kwargs,
            daemon=True,
        )
        self._worker.start()
        self._started = True
        self._start_time = time.monotonic()

        if self._want_orient:
            self._ahrs = MahonyAHRS()
            self._orient_stop.clear()
            self._orient_thread = threading.Thread(
                target=self._orientation_loop, daemon=True)
            self._orient_thread.start()

        atexit.register(self.stop)

    def stop(self) -> None:
        """Stop the sensor worker and free shared memory."""
        if not self._started:
            return
        self._orient_stop.set()
        if self._orient_thread:
            self._orient_thread.join(timeout=2)
            self._orient_thread = None
        self._shutdown.set()
        if self._worker:
            self._worker.join(timeout=3)
        self._worker = None
        for shm in self._shms:
            try:
                shm.close()
                shm.unlink()
            except Exception:
                pass
        self._shms.clear()
        self._shm_accel = None
        self._shm_gyro = None
        self._shm_als = None
        self._shm_lid = None
        self._started = False
        if self._recording_file:
            self._recording_file.close()
            self._recording_file = None
            self._recording_writer = None

    def record_to(self, path: str) -> None:
        """Start recording samples to a CSV file.

        CSV format: t,sensor,x,y,z. Call stop() to flush and close.
        """
        self._recording_file = open(path, 'w', newline='')
        self._recording_writer = csv.writer(self._recording_file)
        self._recording_writer.writerow(['t', 'sensor', 'x', 'y', 'z'])

    def read_accel(self) -> list[Sample]:
        """Return new accelerometer samples since last call.

        Returns list of Sample(x, y, z) in g. Magnitude at rest is ~1g
        (includes gravity).
        """
        if self._mock:
            return self._mock_read('accel', timed=False)
        if not self._shm_accel:
            return []
        raw, self._last_accel_total = shm_read_new(
            self._shm_accel.buf, self._last_accel_total)
        samples = [Sample(*s) for s in raw]
        self._sample_count += len(samples)
        self._record_samples(samples, 'accel')
        return samples

    def read_gyro(self) -> list[Sample]:
        """Return new gyroscope samples since last call.

        Returns list of Sample(x, y, z) in deg/s.
        """
        if self._mock:
            return self._mock_read('gyro', timed=False)
        if not self._shm_gyro:
            return []
        raw, self._last_gyro_total = shm_read_new_gyro(
            self._shm_gyro.buf, self._last_gyro_total)
        samples = [Sample(*s) for s in raw]
        self._record_samples(samples, 'gyro')
        return samples

    def read_accel_timed(self) -> list[TimedSample]:
        """Return new accelerometer samples with hardware timestamps.

        Returns list of TimedSample(t, x, y, z) where t is seconds
        from mach_absolute_time (IOKit HID report timestamp).
        """
        if self._mock:
            return self._mock_read('accel', timed=True)
        if not self._shm_accel:
            return []
        raw, self._last_accel_total = shm_read_new_accel_timed(
            self._shm_accel.buf, self._last_accel_total)
        samples = [TimedSample(*s) for s in raw]
        self._sample_count += len(samples)
        self._record_timed_samples(samples, 'accel')
        return samples

    def read_gyro_timed(self) -> list[TimedSample]:
        """Return new gyroscope samples with hardware timestamps.

        Returns list of TimedSample(t, x, y, z) where t is seconds
        from mach_absolute_time (IOKit HID report timestamp).
        """
        if self._mock:
            return self._mock_read('gyro', timed=True)
        if not self._shm_gyro:
            return []
        raw, self._last_gyro_total = shm_read_new_gyro_timed(
            self._shm_gyro.buf, self._last_gyro_total)
        samples = [TimedSample(*s) for s in raw]
        self._record_timed_samples(samples, 'gyro')
        return samples

    def latest_accel(self) -> Optional[Sample]:
        """Return most recent accelerometer Sample, or None."""
        samples = self.read_accel()
        return samples[-1] if samples else None

    def latest_gyro(self) -> Optional[Sample]:
        """Return most recent gyroscope Sample, or None."""
        samples = self.read_gyro()
        return samples[-1] if samples else None

    def orientation(self) -> Optional[Orientation]:
        """Return current fused orientation, or None if not ready.

        Requires orientation=True in constructor. Returns
        Orientation(roll, pitch, yaw, qw, qx, qy, qz) with angles in degrees.
        """
        if not self._want_orient:
            raise RuntimeError("orientation not enabled -- use IMU(orientation=True)")
        with self._orient_lock:
            return self._last_orient

    def read_lid(self) -> Optional[float]:
        """Return lid angle in degrees, or None if unavailable."""
        if not self._shm_lid:
            return None
        data, self._last_lid_count = shm_snap_read(
            self._shm_lid.buf, self._last_lid_count, 4)
        if data is None:
            return None
        return struct.unpack('<f', data)[0]

    def read_als(self) -> Optional[ALSReading]:
        """Return ambient light data, or None if unavailable.

        Returns ALSReading(lux, channels) where channels is a list of 4 ints.
        """
        if not self._shm_als:
            return None
        raw, self._last_als_count = shm_snap_read(
            self._shm_als.buf, self._last_als_count, ALS_REPORT_LEN)
        if raw is None or len(raw) < 44:
            return None
        lux = struct.unpack_from('<f', raw, _ALS_LUX_OFF)[0]
        channels = [struct.unpack_from('<I', raw, o)[0] for o in _ALS_CH_OFFSETS]
        return ALSReading(lux=lux, channels=channels)

    def read_all(self) -> dict:
        """Return latest reading from all enabled sensors.

        Returns dict with keys like 'accel', 'gyro', 'lid', 'als',
        'orientation' (only present if enabled and data available).
        """
        result = {}
        a = self.latest_accel()
        if a is not None:
            result['accel'] = a
        g = self.latest_gyro()
        if g is not None:
            result['gyro'] = g
        if self._want_lid:
            lid = self.read_lid()
            if lid is not None:
                result['lid'] = lid
        if self._want_als:
            als = self.read_als()
            if als is not None:
                result['als'] = als
        if self._want_orient:
            o = self.orientation()
            if o is not None:
                result['orientation'] = o
        return result

    def stream_accel(self, interval: float = 0.01) -> Generator[Sample, None, None]:
        """Yield accelerometer samples as they arrive."""
        while self._started:
            for s in self.read_accel():
                yield s
            time.sleep(interval)

    def stream_gyro(self, interval: float = 0.01) -> Generator[Sample, None, None]:
        """Yield gyroscope samples as they arrive."""
        while self._started:
            for s in self.read_gyro():
                yield s
            time.sleep(interval)

    def stream_accel_timed(self, interval: float = 0.01) -> Generator[TimedSample, None, None]:
        """Yield timestamped accelerometer samples as they arrive."""
        while self._started:
            for s in self.read_accel_timed():
                yield s
            time.sleep(interval)

    def stream_gyro_timed(self, interval: float = 0.01) -> Generator[TimedSample, None, None]:
        """Yield timestamped gyroscope samples as they arrive."""
        while self._started:
            for s in self.read_gyro_timed():
                yield s
            time.sleep(interval)

    def on_accel(self, callback: Callable[[Sample], None],
                 interval: float = 0.01) -> Callable[[], None]:
        """Register a callback for new accelerometer samples.

        Returns a stop function -- call it to unregister.
        """
        return self._start_callback_thread(self.read_accel, callback, interval)

    def on_gyro(self, callback: Callable[[Sample], None],
                interval: float = 0.01) -> Callable[[], None]:
        """Register a callback for new gyroscope samples.

        Returns a stop function -- call it to unregister.
        """
        return self._start_callback_thread(self.read_gyro, callback, interval)

    def __enter__(self) -> IMU:
        self.start()
        return self

    def __exit__(self, *exc) -> None:
        self.stop()

    # -- internal --

    def _mock_read(self, sensor: str, timed: bool):
        """Read from synthetic/recorded mock data."""
        elapsed = time.monotonic() - self._start_time
        results = []
        while self._mock_idx < len(self._mock_data):
            row = self._mock_data[self._mock_idx]
            if isinstance(row, tuple) and len(row) == 7:
                t, ax, ay, az, gx, gy, gz = row
                if t > elapsed:
                    break
                self._mock_idx += 1
                if sensor == 'accel':
                    results.append(TimedSample(t, ax, ay, az) if timed else Sample(ax, ay, az))
                else:
                    results.append(TimedSample(t, gx, gy, gz) if timed else Sample(gx, gy, gz))
            elif isinstance(row, tuple) and len(row) == 5:
                t, s_type, x, y, z = row
                if t > elapsed:
                    break
                self._mock_idx += 1
                if s_type == sensor:
                    results.append(TimedSample(t, x, y, z) if timed else Sample(x, y, z))
            else:
                self._mock_idx += 1
        return results

    def _record_samples(self, samples, sensor):
        if self._recording_writer:
            for s in samples:
                self._recording_writer.writerow(['', sensor, f'{s.x:.6f}', f'{s.y:.6f}', f'{s.z:.6f}'])

    def _record_timed_samples(self, samples, sensor):
        if self._recording_writer:
            for s in samples:
                self._recording_writer.writerow([f'{s.t:.6f}', sensor, f'{s.x:.6f}', f'{s.y:.6f}', f'{s.z:.6f}'])

    def _orientation_loop(self) -> None:
        """Background thread: fuses accel+gyro into orientation quaternion."""
        ahrs = self._ahrs
        accel_total = 0
        gyro_total = 0
        last_gyro = (0.0, 0.0, 0.0)

        while not self._orient_stop.is_set():
            if self._shm_gyro:
                gyro_raw, gyro_total = shm_read_new_gyro(
                    self._shm_gyro.buf, gyro_total)
                if gyro_raw:
                    last_gyro = gyro_raw[-1]

            if self._shm_accel:
                accel_timed, accel_total = shm_read_new_accel_timed(
                    self._shm_accel.buf, accel_total)
                if len(accel_timed) >= 2:
                    dt = (accel_timed[-1][0] - accel_timed[0][0]) / len(accel_timed)
                    dt = max(dt, 0.001)
                else:
                    dt = 0.01
                for t, ax, ay, az in accel_timed:
                    ahrs.update(ax, ay, az, last_gyro[0], last_gyro[1], last_gyro[2], dt)

                if accel_timed:
                    r, p, y = ahrs.euler()
                    qw, qx, qy, qz = ahrs.quaternion
                    with self._orient_lock:
                        self._last_orient = Orientation(r, p, y, qw, qx, qy, qz)

            self._orient_stop.wait(0.01)

    def _start_callback_thread(self, read_fn, callback, interval):
        stop = threading.Event()

        def _poll():
            while not stop.is_set():
                for s in read_fn():
                    callback(s)
                stop.wait(interval)

        t = threading.Thread(target=_poll, daemon=True)
        t.start()
        return stop.set

    def _create_shm(self, name, size):
        shm = multiprocessing.shared_memory.SharedMemory(
            name=name, create=True, size=size)
        for i in range(size):
            shm.buf[i] = 0
        self._shms.append(shm)
        return shm

    def _cleanup_stale_shm(self):
        names = [SHM_NAME, SHM_NAME_GYRO, SHM_NAME_ALS, SHM_NAME_LID]
        for name in names:
            try:
                old = multiprocessing.shared_memory.SharedMemory(
                    name=name, create=False)
                old.close()
                old.unlink()
            except FileNotFoundError:
                pass
