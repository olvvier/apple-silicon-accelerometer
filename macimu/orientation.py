"""Mahony AHRS quaternion filter for fusing accelerometer + gyroscope."""

import math


class MahonyAHRS:
    """Attitude and heading reference system using Mahony's complementary filter.

    Fuses accelerometer (gravity reference) with gyroscope (angular velocity)
    to produce a drift-corrected orientation quaternion.
    """

    def __init__(self, kp: float = 1.0, ki: float = 0.05):
        self._q = [1.0, 0.0, 0.0, 0.0]
        self._kp = kp
        self._ki = ki
        self._err_int = [0.0, 0.0, 0.0]
        self._initialized = False

    def update(self, ax: float, ay: float, az: float,
               gx_dps: float, gy_dps: float, gz_dps: float,
               dt: float) -> None:
        """Feed one accel sample (g) and gyro sample (deg/s) with timestep dt."""
        a_norm = math.sqrt(ax * ax + ay * ay + az * az)
        if a_norm < 0.3:
            return

        gx = math.radians(gx_dps)
        gy = math.radians(gy_dps)
        gz = math.radians(gz_dps)

        if not self._initialized:
            ax_n, ay_n, az_n = ax / a_norm, ay / a_norm, az / a_norm
            pitch0 = math.atan2(-ax_n, -az_n)
            roll0 = math.atan2(ay_n, -az_n)
            cp, sp = math.cos(pitch0 * 0.5), math.sin(pitch0 * 0.5)
            cr, sr = math.cos(roll0 * 0.5), math.sin(roll0 * 0.5)
            self._q = [cr * cp, sr * cp, cr * sp, -sr * sp]
            self._initialized = True
            return

        q = self._q
        inv_norm = 1.0 / a_norm
        ax_n, ay_n, az_n = ax * inv_norm, ay * inv_norm, az * inv_norm

        qw, qx, qy, qz = q
        vx = 2.0 * (qx * qz - qw * qy)
        vy = 2.0 * (qw * qx + qy * qz)
        vz = qw * qw - qx * qx - qy * qy + qz * qz

        ex = ay_n * (-vz) - az_n * (-vy)
        ey = az_n * (-vx) - ax_n * (-vz)
        ez = ax_n * (-vy) - ay_n * (-vx)

        self._err_int[0] += self._ki * ex * dt
        self._err_int[1] += self._ki * ey * dt
        self._err_int[2] += self._ki * ez * dt

        gx += self._kp * ex + self._err_int[0]
        gy += self._kp * ey + self._err_int[1]
        gz += self._kp * ez + self._err_int[2]

        hdt = 0.5 * dt
        dw = (-qx * gx - qy * gy - qz * gz) * hdt
        dx = (qw * gx + qy * gz - qz * gy) * hdt
        dy = (qw * gy - qx * gz + qz * gx) * hdt
        dz = (qw * gz + qx * gy - qy * gx) * hdt

        qw += dw; qx += dx; qy += dy; qz += dz

        n = math.sqrt(qw * qw + qx * qx + qy * qy + qz * qz)
        if n > 0:
            inv_n = 1.0 / n
            qw *= inv_n; qx *= inv_n; qy *= inv_n; qz *= inv_n

        self._q = [qw, qx, qy, qz]

    @property
    def quaternion(self) -> tuple:
        """Current orientation as (w, x, y, z) quaternion."""
        return tuple(self._q)

    def euler(self) -> tuple:
        """Current orientation as (roll, pitch, yaw) in degrees."""
        qw, qx, qy, qz = self._q
        sin_r = 2.0 * (qw * qx + qy * qz)
        cos_r = 1.0 - 2.0 * (qx * qx + qy * qy)
        roll = math.degrees(math.atan2(sin_r, cos_r))
        sin_p = max(-1.0, min(1.0, 2.0 * (qw * qy - qz * qx)))
        pitch = math.degrees(math.asin(sin_p))
        sin_y = 2.0 * (qw * qz + qx * qy)
        cos_y = 1.0 - 2.0 * (qy * qy + qz * qz)
        yaw = math.degrees(math.atan2(sin_y, cos_y))
        return (roll, pitch, yaw)

    def reset(self) -> None:
        """Reset filter state."""
        self._q = [1.0, 0.0, 0.0, 0.0]
        self._err_int = [0.0, 0.0, 0.0]
        self._initialized = False
