# apple-silicon-accelerometer

more information: [read the article on Medium](https://medium.com/@oli.bourbonnais/your-macbook-has-an-accelerometer-and-you-can-read-it-in-real-time-in-python-28d9395fb180)

it turns out modern macbook pros have an undocumented mems accelerometer + gyroscope managed by the sensor processing unit (spu).
this project reads both via iokit hid, along with lid angle and ambient light sensors from the same interface

![demo](https://raw.githubusercontent.com/olvvier/apple-silicon-accelerometer/main/assets/demo.gif)

## try it

    git clone https://github.com/olvvier/apple-silicon-accelerometer
    cd apple-silicon-accelerometer
    python3 -m venv .venv && source .venv/bin/activate
    pip install -e .[demo]
    sudo .venv/bin/python3 motion_live.py

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
callback rate is ~100hz (decimated from ~800hz native)

orientation is computed by fusing accel + gyro with a Mahony AHRS quaternion filter and displayed as roll/pitch/yaw gauges

you can verify the device exists on your machine with:

    ioreg -l -w0 | grep -A5 AppleSPUHIDDevice

## install

    pip install macimu

if you get `externally-managed-environment` (homebrew python), use a venv:

    python3 -m venv .venv && source .venv/bin/activate && pip install macimu

```python
from macimu import IMU

with IMU() as imu:
    accel = imu.latest_accel()       # Sample(x, y, z) in g
    gyro = imu.latest_gyro()         # Sample(x, y, z) in deg/s

    for s in imu.read_accel():       # all new samples since last call
        print(s.x, s.y, s.z)
```

requires root (sudo) because iokit hid device access needs elevated privileges.
note: accelerometer reads ~1g at rest (gravity). use `macimu.filters.remove_gravity()` to isolate dynamic acceleration.

### one sample at a time (arduino-style)

```python
from macimu import IMU

with IMU(sample_rate=200) as imu:
    while True:
        s = imu.wait_accel()         # blocks until next sample
        print(s.x, s.y, s.z)        # per-axis access
```

### real-time streaming pipeline

gravity removal, filtering, and peak detection -- all sample-by-sample, all stateful

```python
from macimu import IMU
from macimu.filters import GravityKalman, BiquadFilter, StreamingPeakDetector, magnitude

gk = GravityKalman(warmup_samples=200)
hp = BiquadFilter('hp', 10.0, 200.0)
pd = StreamingPeakDetector(threshold=0.15, min_spacing=30)

with IMU(sample_rate=200) as imu:
    for s in imu.stream_accel():
        dx, dy, dz = gk.feed_remove(s.x, s.y, s.z)
        if not gk.warmed_up:
            continue
        fx, fy, fz = hp.feed(dx, dy, dz)
        if pd.feed(magnitude(fx, fy, fz)):
            print(f"tap! z={fz:.3f}")
```

### check if sensor exists (no root needed)

```python
from macimu import IMU
print(IMU.available())   # True on macbook pro m2+
```

### orientation (roll / pitch / yaw)

```python
from macimu import IMU

with IMU(orientation=True) as imu:
    o = imu.orientation()
    print(f"{o.roll:.1f}° {o.pitch:.1f}° {o.yaw:.1f}°")
```

### hardware timestamps

each hid report carries its own mach_absolute_time timestamp

```python
from macimu import IMU

with IMU() as imu:
    for s in imu.read_accel_timed():
        print(f"t={s.t:.6f}  x={s.x:.3f}  y={s.y:.3f}  z={s.z:.3f}")
```

### sample rate control

```python
IMU(sample_rate=200)  # ~200 hz (preferred way)
IMU(sample_rate=50)   # ~50 hz
IMU(decimation=1)     # ~800 hz (full native rate)
IMU(decimation=8)     # ~100 hz (default)
```

### batch signal processing

```python
from macimu import IMU
from macimu.filters import magnitude, remove_gravity, high_pass, peak_detect

with IMU() as imu:
    samples = imu.read_accel()
    dynamic = remove_gravity(samples)               # kalman gravity removal
    taps = high_pass(samples, 10.0, 100.0, order=4) # 4th-order butterworth
    mags = [magnitude(s.x, s.y, s.z) for s in samples]
    hits = peak_detect(mags, threshold=1.2)          # detect impacts
```

all batch filters return `Sample(x, y, z)` namedtuples with per-axis access

### mock mode (no root needed)

```python
from macimu import IMU

imu = IMU.mock(duration=10.0, rate=100)
for s in imu.stream_accel():   # yields 1000 samples then stops
    print(s.x, s.y, s.z)
```

### record and replay

```python
import time
from macimu import IMU

with IMU() as imu:
    imu.record_to("session.csv")
    time.sleep(10)

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
| `imu.wait_accel(timeout)` | `Sample \| None` | block until next sample (arduino-style) |
| `imu.wait_gyro(timeout)` | `Sample \| None` | block until next sample |
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
| `imu.effective_sample_rate` | measured hz |
| `imu.record_to(path)` | start writing samples to csv |

**streaming filter classes** (`from macimu.filters import ...`) -- stateful, one sample at a time

| class | method | description |
|-------|--------|-------------|
| `BiquadFilter(mode, cutoff_hz, rate, order=2)` | `.feed(x,y,z)` -> `(x,y,z)` | butterworth lp/hp, maintains state between calls |
| | `.feed_sample(s)` -> `Sample` | same but takes/returns Sample |
| `GravityKalman(Q, R, warmup_samples=200)` | `.feed_remove(ax,ay,az)` -> `(dx,dy,dz)` | gravity subtraction in one call |
| | `.feed_remove_sample(s)` -> `Sample` | same but takes/returns Sample |
| | `.warmed_up` -> `bool` | True after warmup period |
| `StreamingPeakDetector(threshold, min_spacing)` | `.feed(value)` -> `bool` | True when peak detected |
| `StreamingRMS(window=50)` | `.feed(x,y,z)` -> `float` | rolling rms magnitude |

**batch filters** (`from macimu.filters import ...`) -- all return `list[Sample]` with `.x .y .z` access

| function | description |
|----------|-------------|
| `magnitude(x, y, z)` | euclidean magnitude |
| `remove_gravity(samples, Q, R)` | kalman gravity subtraction |
| `low_pass(samples, cutoff_hz, rate, order=2)` | butterworth low-pass |
| `high_pass(samples, cutoff_hz, rate, order=2)` | butterworth high-pass |
| `bandpass(samples, low, high, rate, order=2)` | cascaded hp + lp |
| `filtfilt_low_pass(samples, cutoff_hz, rate)` | zero-phase lp (offline only) |
| `filtfilt_high_pass(samples, cutoff_hz, rate)` | zero-phase hp (offline only) |
| `median_filter(samples, window=5)` | spike / outlier removal |
| `peak_detect(values, threshold, min_spacing)` | find peaks in 1d signal |
| `rolling_rms(samples, window)` | rolling rms of magnitude |

**exceptions**: `macimu.SensorNotFound` if no SPU device, `PermissionError` if not root

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
