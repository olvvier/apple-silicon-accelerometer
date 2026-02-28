# apple-silicon-accelerometer

more information: [read the article on Medium](https://medium.com/@oli.bourbonnais/your-macbook-has-an-accelerometer-and-you-can-read-it-in-real-time-in-python-28d9395fb180)

it turns out modern macbook pros have an undocumented mems accelerometer + gyroscope managed by the sensor processing unit (spu).
this project reads both via iokit hid, along with lid angle and ambient light sensors from the same interface

## install

    pip install macimu

if you get `externally-managed-environment` (homebrew python), use a venv:

    python3 -m venv .venv && source .venv/bin/activate && pip install macimu

```python
from macimu import IMU

if __name__ == '__main__':
    with IMU() as imu:
        accel = imu.latest_accel()       # Sample(x, y, z) in g
        gyro = imu.latest_gyro()         # Sample(x, y, z) in deg/s

        for s in imu.read_accel():       # all new samples since last call
            print(s.x, s.y, s.z)
```

requires root (sudo) because iokit hid device access needs elevated privileges.
note: accelerometer reads ~1g at rest (gravity). use `macimu.filters.remove_gravity()` to isolate dynamic acceleration.

### check if sensor exists (no root needed)

```python
from macimu import IMU
print(IMU.available())   # True on macbook pro m2+
```

### real-time orientation (roll / pitch / yaw)

fuses accel + gyro with a mahony quaternion filter, no math needed on your side

```python
from macimu import IMU

if __name__ == '__main__':
    with IMU(orientation=True) as imu:
        o = imu.orientation()
        print(f"{o.roll:.1f}° {o.pitch:.1f}° {o.yaw:.1f}°")
        print(o.qw, o.qx, o.qy, o.qz)  # raw quaternion
```

### timestamped samples (hardware timestamps from iokit)

each sample includes a precise timestamp from the hid report (mach_absolute_time),
not a python-side clock. every report gets its own unique timestamp.

```python
from macimu import IMU

if __name__ == '__main__':
    with IMU() as imu:
        for s in imu.read_accel_timed():
            print(f"t={s.t:.6f}  x={s.x:.3f}  y={s.y:.3f}  z={s.z:.3f}")
```

### streaming with callback

```python
import time
from macimu import IMU

def on_sample(s):
    print(s.x, s.y, s.z)

if __name__ == '__main__':
    with IMU() as imu:
        stop = imu.on_accel(on_sample)  # background thread
        time.sleep(10)
        stop()                          # unregister
```

### sample rate control

```python
IMU(sample_rate=200)  # ~200 hz (preferred way)
IMU(sample_rate=50)   # ~50 hz
IMU(decimation=1)     # ~800 hz (full native rate)
IMU(decimation=8)     # ~100 hz (default)
```

### signal processing

```python
from macimu import IMU
from macimu.filters import magnitude, remove_gravity, high_pass, low_pass

if __name__ == '__main__':
    with IMU() as imu:
        samples = imu.read_accel()
        m = magnitude(samples[0].x, samples[0].y, samples[0].z)
        dynamic = remove_gravity(samples)       # subtract gravity estimate
        smooth = low_pass(samples, 5.0, 100.0)  # 5 hz cutoff, 100 hz rate
        taps = high_pass(samples, 10.0, 100.0)  # isolate sharp impacts
```

### mock mode (no root needed, for development / testing)

```python
from macimu import IMU

imu = IMU.mock(duration=10.0, rate=100)  # synthetic sinusoidal data
for s in imu.stream_accel():
    print(s)
```

### record and replay

```python
from macimu import IMU

if __name__ == '__main__':
    # record
    with IMU() as imu:
        imu.record_to("session.csv")
        time.sleep(10)

    # replay (no root needed)
    imu = IMU.from_recording("session.csv")
    for s in imu.stream_accel_timed():
        print(s)
```

### api reference

**constructor**

    IMU(accel=True, gyro=True, als=False, lid=False, orientation=False, decimation=8, sample_rate=None)

**class methods** (no root needed)

| method | returns | description |
|--------|---------|-------------|
| `IMU.available()` | `bool` | check if sensor exists |
| `IMU.device_info()` | `dict` | sensors list, serial, product name |
| `IMU.mock(duration, rate, noise)` | `IMU` | synthetic data for testing |
| `IMU.from_recording(path)` | `IMU` | replay from csv |

**reading data**

| method | returns | description |
|--------|---------|-------------|
| `imu.read_accel()` | `list[Sample]` | new samples since last call (x, y, z in g) |
| `imu.read_gyro()` | `list[Sample]` | new samples since last call (x, y, z in deg/s) |
| `imu.read_accel_timed()` | `list[TimedSample]` | with hardware timestamp (t, x, y, z) |
| `imu.read_gyro_timed()` | `list[TimedSample]` | same for gyro |
| `imu.latest_accel()` | `Sample \| None` | most recent sample |
| `imu.latest_gyro()` | `Sample \| None` | most recent sample |
| `imu.read_all()` | `dict` | latest from all enabled sensors |

**orientation & sensors**

| method | returns | description |
|--------|---------|-------------|
| `imu.orientation()` | `Orientation \| None` | roll, pitch, yaw (deg) + quaternion |
| `imu.read_lid()` | `float \| None` | lid angle in degrees |
| `imu.read_als()` | `ALSReading \| None` | lux + 4 spectral channels |

**streaming**

| method | returns | description |
|--------|---------|-------------|
| `imu.stream_accel()` | generator | blocking, yields `Sample` |
| `imu.stream_gyro()` | generator | blocking, yields `Sample` |
| `imu.stream_accel_timed()` | generator | blocking, yields `TimedSample` |
| `imu.stream_gyro_timed()` | generator | blocking, yields `TimedSample` |
| `imu.on_accel(callback)` | `stop_fn` | background thread, call `stop()` to end |
| `imu.on_gyro(callback)` | `stop_fn` | background thread, call `stop()` to end |

**lifecycle**

| method / property | description |
|-------------------|-------------|
| `imu.start()` / `imu.stop()` | manual lifecycle (or use `with IMU() as imu:`) |
| `imu.is_running` | `True` if worker is active |
| `imu.effective_sample_rate` | measured hz (or `None` if not enough data) |
| `imu.record_to(path)` | start writing samples to csv |

**filters** (`from macimu.filters import ...`)

| function | description |
|----------|-------------|
| `magnitude(x, y, z)` | euclidean magnitude |
| `remove_gravity(samples, alpha)` | subtract slow-moving gravity estimate |
| `high_pass(samples, cutoff_hz, rate)` | first-order iir high-pass |
| `low_pass(samples, cutoff_hz, rate)` | first-order iir low-pass |
| `bandpass(samples, low, high, rate)` | cascaded high + low pass |
| `rolling_rms(samples, window)` | rolling root-mean-square of magnitude |

**exceptions**: `macimu.SensorNotFound` if no SPU device, `PermissionError` if not root

![demo](https://raw.githubusercontent.com/olvvier/apple-silicon-accelerometer/main/assets/demo.gif)

## what is this

apple silicon chips (M2/M3/M4/M5) have a hard to find mems IMU (accelerometer + gyroscope) managed by the sensor processing unit (SPU).
it's not exposed through any public api or framework.
this project reads raw 3-axis acceleration and angular velocity data at ~800hz via iokit hid callbacks.

only tested on macbook pro m3 pro so far - might work on other apple silicon macs but no guarantees

## how it works

the sensor lives under AppleSPUHIDDevice in the iokit registry, on vendor usage page 0xFF00.
usage 3 is the accelerometer, usage 9 is the gyroscope (same physical IMU, believed to be Bosch BMI286 based on teardowns).
the driver is AppleSPUHIDDriver which is part of the sensor processing unit.
we open it with IOHIDDeviceCreate and register an asynchronous callback via IOHIDDeviceRegisterInputReportWithTimeStampCallback.
data comes as 22-byte hid reports with x/y/z as int32 little-endian at byte offsets 6, 10, 14.
divide by 65536 to get the value in g (accel) or deg/s (gyro).
each report carries a hardware timestamp (mach_absolute_time).
callback rate is ~100hz (decimated from ~800hz native)

orientation is computed by fusing accel + gyro with a Mahony AHRS quaternion filter and displayed as roll/pitch/yaw gauges

you can verify the device exists on your machine with:

    ioreg -l -w0 | grep -A5 AppleSPUHIDDevice

## demo dashboard

    git clone https://github.com/olvvier/apple-silicon-accelerometer
    cd apple-silicon-accelerometer
    python3 -m venv .venv && source .venv/bin/activate
    pip install -e .[demo]
    sudo .venv/bin/python3 motion_live.py

the demo includes vibration detection, orientation gauges, experimental heartbeat (bcg), lid angle, ambient light, and optional keyboard flash

### keyboard flash mode (bundled KBPulse)

`motion_live.py` can flash the keyboard backlight from vibration intensity in near realtime.
the repo now vendors KBPulse, including a prebuilt apple silicon binary at `KBPulse/bin/KBPulse`.

run as usual:

    sudo python3 motion_live.py

optional overrides:

    sudo python3 motion_live.py --no-kbpulse
    sudo python3 motion_live.py --kbpulse-bin /path/to/KBPulse

### with uv

If you have `uv`/`uvx` installed, you can also just

    sudo uvx git+https://github.com/olvvier/apple-silicon-accelerometer.git

## code structure

- macimu/ - python package (`pip install macimu`): high-level IMU class + low-level iokit bindings, shared memory ring buffers
- motion_live.py - demo app: vibration detection, heartbeat bcg, terminal ui
- KBPulse/ - vendored keyboard backlight driver code + binary (`KBPulse/bin/KBPulse`)

## heartbeat demo

place your wrists on the laptop near the trackpad and wait 10-20 seconds for the signal to stabilize.
this uses ballistocardiography - the mechanical vibrations from your heartbeat transmitted through your arms into the chassis.
experimental, not reliable, just a fun use-case to show what the sensor can pick up.
the bcg bandpass is 0.8-3hz and bpm is estimated via autocorrelation on the filtered signal

## notes

- experimental / undocumented AppleSPU hid path
- requires sudo
- may break on future macos updates
- use at your own risk
- not for medical use

## tested on

- macbook pro m3 pro, macos 15.6.1
- python 3.14


## known incompatible

- intel macs (no spu)
- m1 macbook pro (2020)
- mac studio m4 max 


## license

MIT

---

not affiliated with Apple or any employer
