"""macimu - read the undocumented IMU on apple silicon macs via iokit hid"""

__version__ = "0.1.0"

import os
import struct
import time
import multiprocessing
import multiprocessing.shared_memory
from collections import namedtuple

Sample = namedtuple('Sample', ['x', 'y', 'z'])
ALSReading = namedtuple('ALSReading', ['lux', 'channels'])

from ._spu import (
    sensor_worker,
    shm_read_new, shm_read_new_gyro, shm_snap_read,
    SHM_NAME, SHM_NAME_GYRO, SHM_SIZE,
    SHM_NAME_ALS, SHM_ALS_SIZE, SHM_NAME_LID, SHM_LID_SIZE,
    SHM_SNAP_HDR, ALS_REPORT_LEN,
    ACCEL_SCALE, GYRO_SCALE,
)

_ALS_LUX_OFF = 40
_ALS_CH_OFFSETS = [20, 24, 28, 32]


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
    """

    def __init__(self, accel=True, gyro=True, als=False, lid=False):
        self._want_accel = accel
        self._want_gyro = gyro
        self._want_als = als
        self._want_lid = lid
        self._started = False
        self._worker = None
        self._shms = []
        self._shm_accel = None
        self._shm_gyro = None
        self._shm_als = None
        self._shm_lid = None
        self._last_accel_total = 0
        self._last_gyro_total = 0
        self._last_als_count = 0
        self._last_lid_count = 0

    def start(self):
        """Start the sensor worker process. Requires root."""
        if self._started:
            return
        if os.geteuid() != 0:
            raise PermissionError("macimu requires root -- run with sudo")

        self._cleanup_stale_shm()

        self._shm_accel = self._create_shm(SHM_NAME, SHM_SIZE)
        kwargs = {}

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

    def stop(self):
        """Stop the sensor worker and free shared memory."""
        if not self._started:
            return
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

    def read_accel(self):
        """Return new accelerometer samples since last call.

        Returns list of Sample(x, y, z) in g.
        """
        if not self._shm_accel:
            return []
        raw, self._last_accel_total = shm_read_new(
            self._shm_accel.buf, self._last_accel_total)
        return [Sample(*s) for s in raw]

    def read_gyro(self):
        """Return new gyroscope samples since last call.

        Returns list of Sample(x, y, z) in deg/s.
        """
        if not self._shm_gyro:
            return []
        raw, self._last_gyro_total = shm_read_new_gyro(
            self._shm_gyro.buf, self._last_gyro_total)
        return [Sample(*s) for s in raw]

    def latest_accel(self):
        """Return most recent accelerometer Sample, or None."""
        samples = self.read_accel()
        return samples[-1] if samples else None

    def latest_gyro(self):
        """Return most recent gyroscope Sample, or None."""
        samples = self.read_gyro()
        return samples[-1] if samples else None

    def read_lid(self):
        """Return lid angle in degrees, or None if unavailable."""
        if not self._shm_lid:
            return None
        data, self._last_lid_count = shm_snap_read(
            self._shm_lid.buf, self._last_lid_count, 4)
        if data is None:
            return None
        return struct.unpack('<f', data)[0]

    def read_als(self):
        """Return ambient light data as dict, or None if unavailable.

        Returns dict with keys: lux (float), channels (list of 4 ints).
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

    def __enter__(self):
        self.start()
        return self

    def __exit__(self, *exc):
        self.stop()

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
