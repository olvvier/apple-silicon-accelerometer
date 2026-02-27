"""macimu - read the undocumented IMU on apple silicon macs via iokit hid"""

from __future__ import annotations

__version__ = "0.1.1"

import os
import struct
import time
import threading
import multiprocessing
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
)
from .orientation import MahonyAHRS

_ALS_LUX_OFF = 40
_ALS_CH_OFFSETS = [20, 24, 28, 32]


class SensorNotFound(RuntimeError):
    """Raised when no SPU IMU device is found on this machine."""
    pass


class IMU:
    """High-level interface to the Apple Silicon SPU IMU.

    Requires root privileges (sudo). Spawns a background worker process
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
    decimation : int
        Keep 1 in N raw HID reports. Lower = higher sample rate.
        Default 8 gives ~100 Hz from ~800 Hz native.
    """

    def __init__(self, accel: bool = True, gyro: bool = True,
                 als: bool = False, lid: bool = False,
                 orientation: bool = False, decimation: int = 8) -> None:
        self._want_accel = accel
        self._want_gyro = gyro
        self._want_als = als
        self._want_lid = lid
        self._want_orient = orientation
        self._decimation = decimation
        self._started = False
        self._worker: Optional[multiprocessing.Process] = None
        self._shms: list = []
        self._shm_accel = None
        self._shm_gyro = None
        self._shm_als = None
        self._shm_lid = None
        self._last_accel_total = 0
        self._last_gyro_total = 0
        self._last_als_count = 0
        self._last_lid_count = 0

        self._ahrs: Optional[MahonyAHRS] = None
        self._orient_thread: Optional[threading.Thread] = None
        self._orient_stop = threading.Event()
        self._orient_lock = threading.Lock()
        self._last_orient: Optional[Orientation] = None

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

    def start(self) -> None:
        """Start the sensor worker process. Requires root."""
        if self._started:
            return
        if os.geteuid() != 0:
            raise PermissionError("macimu requires root -- run with sudo")
        if not check_available():
            raise SensorNotFound(
                "no SPU IMU found -- this machine may not have the sensor")

        self._cleanup_stale_shm()

        self._shm_accel = self._create_shm(SHM_NAME, SHM_SIZE)
        kwargs = {'decimation': self._decimation}

        if self._want_gyro:
            self._shm_gyro = self._create_shm(SHM_NAME_GYRO, SHM_SIZE)
            kwargs['gyro_shm_name'] = SHM_NAME_GYRO
        if self._want_als:
            self._shm_als = self._create_shm(SHM_NAME_ALS, SHM_ALS_SIZE)
            kwargs['als_shm_name'] = SHM_NAME_ALS
        if self._want_lid:
            self._shm_lid = self._create_shm(SHM_NAME_LID, SHM_LID_SIZE)
            kwargs['lid_shm_name'] = SHM_NAME_LID

        self._worker = multiprocessing.Process(
            target=sensor_worker,
            args=(SHM_NAME, 0),
            kwargs=kwargs,
            daemon=True,
        )
        self._worker.start()
        self._started = True

        if self._want_orient:
            self._ahrs = MahonyAHRS()
            self._orient_stop.clear()
            self._orient_thread = threading.Thread(
                target=self._orientation_loop, daemon=True)
            self._orient_thread.start()

    def stop(self) -> None:
        """Stop the sensor worker and free shared memory."""
        if not self._started:
            return
        self._orient_stop.set()
        if self._orient_thread:
            self._orient_thread.join(timeout=2)
            self._orient_thread = None
        if self._worker and self._worker.is_alive():
            self._worker.kill()
            self._worker.join(timeout=2)
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

    def read_accel(self) -> list[Sample]:
        """Return new accelerometer samples since last call.

        Returns list of Sample(x, y, z) in g.
        """
        if not self._shm_accel:
            return []
        raw, self._last_accel_total = shm_read_new(
            self._shm_accel.buf, self._last_accel_total)
        return [Sample(*s) for s in raw]

    def read_gyro(self) -> list[Sample]:
        """Return new gyroscope samples since last call.

        Returns list of Sample(x, y, z) in deg/s.
        """
        if not self._shm_gyro:
            return []
        raw, self._last_gyro_total = shm_read_new_gyro(
            self._shm_gyro.buf, self._last_gyro_total)
        return [Sample(*s) for s in raw]

    def read_accel_timed(self) -> list[TimedSample]:
        """Return new accelerometer samples with monotonic timestamps.

        Returns list of TimedSample(t, x, y, z) where t is time.monotonic().
        """
        if not self._shm_accel:
            return []
        raw, self._last_accel_total = shm_read_new_accel_timed(
            self._shm_accel.buf, self._last_accel_total)
        return [TimedSample(*s) for s in raw]

    def read_gyro_timed(self) -> list[TimedSample]:
        """Return new gyroscope samples with monotonic timestamps.

        Returns list of TimedSample(t, x, y, z) where t is time.monotonic().
        """
        if not self._shm_gyro:
            return []
        raw, self._last_gyro_total = shm_read_new_gyro_timed(
            self._shm_gyro.buf, self._last_gyro_total)
        return [TimedSample(*s) for s in raw]

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

    def stream_accel(self, interval: float = 0.01) -> Generator[Sample, None, None]:
        """Yield accelerometer samples as they arrive.

        Blocks between polls. Use in a loop or a dedicated thread.
        """
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
