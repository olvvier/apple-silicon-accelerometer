# apple-silicon-accelerometer

More information: [read the article on Medium](https://medium.com/@oli.bourbonnais/your-macbook-has-an-accelerometer-and-you-can-read-it-in-real-time-in-python-28d9395fb180)

reading the undocumented mems accelerometer + gyroscope on apple silicon macbooks via iokit hid

![demo](assets/demo.gif)

## what is this

apple silicon chips (M1/M2/M3/M4/M5) have a hard to find mems IMU (accelerometer + gyroscope) managed by the sensor processing unit (SPU).
it's not exposed through any public api or framework.
this project reads raw 3-axis acceleration and angular velocity data at ~800hz via iokit hid callbacks.

only tested on macbook pro m3 pro so far - might work on other apple silicon macs but no guarantees

## how it works

the sensor lives under AppleSPUHIDDevice in the iokit registry, on vendor usage page 0xFF00.
usage 3 is the accelerometer, usage 9 is the gyroscope (same physical IMU, believed to be Bosch BMI286 based on teardowns).
the driver is AppleSPUHIDDriver which is part of the sensor processing unit.
we open it with IOHIDDeviceCreate and register an asynchronous callback via IOHIDDeviceRegisterInputReportCallback.
data comes as 22-byte hid reports with x/y/z as int32 little-endian at byte offsets 6, 10, 14.
divide by 65536 to get the value in g (accel) or deg/s (gyro).
callback rate is ~100hz (decimated from ~800hz native)

orientation is computed by fusing accel + gyro with a Mahony AHRS quaternion filter and displayed as roll/pitch/yaw gauges

you can verify the device exists on your machine with:

    ioreg -l -w0 | grep -A5 AppleSPUHIDDevice

## quick start

    git clone https://github.com/olvvier/apple-silicon-accelerometer
    cd apple-silicon-accelerometer
    pip install -e .
    sudo python3 motion_live.py

requires root because iokit hid device access on apple silicon needs elevated privileges

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

- spu_sensor.py - the core: iokit bindings, device discovery, accel + gyro hid callbacks, shared memory ring buffers
- motion_live.py - vibration detection pipeline, heartbeat bcg, terminal ui, main loop
- KBPulse/ - vendored keyboard backlight driver code + binary (`KBPulse/bin/KBPulse`)

the sensor reading logic is isolated in spu_sensor.py so you can reuse it independently

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

## license

MIT

---

not affiliated with Apple or any employer
