# Security Audit: apple-silicon-accelerometer

## Why this matters

Tools that interact directly with hardware via low-level system interfaces — such as IOKit HID callbacks, `ctypes` bindings into C frameworks, and shared memory IPC — demand careful security scrutiny. Unlike typical application-layer Python code, these utilities operate with elevated privileges (root), bypass standard OS abstractions, and manipulate raw memory buffers. A vulnerability at this level can have outsized impact: privilege escalation, memory corruption, or silent data exfiltration. Evaluating the security and safety of such tools is crucial before running them on any machine.

---

## Overall verdict

**The repository is safe.** There are no high-severity vulnerabilities. This is a small, focused, local-only hardware utility with a minimal attack surface. The issues documented below are low-severity observations, not blockers.

## What the code does

Two Python files read the undocumented accelerometer on Apple Silicon MacBooks via IOKit HID (`spu_sensor.py`), process the signal for vibration/heartbeat detection, and render a real-time terminal UI (`motion_live.py`). It has one external dependency (`PyWavelets`), no network access, no user input parsing, and no credential handling.

## No major security concerns found

- **No network access** — zero imports of `socket`, `http`, `requests`, `urllib`, etc. All data stays local.
- **No command injection** — no `subprocess`, `os.system`, `eval`, or `exec` calls. No shell commands are constructed.
- **No deserialization of untrusted data** — the only data parsed comes from the hardware sensor via IOKit HID reports.
- **No user input processing** — no CLI arguments are consumed (beyond `sys.argv[0]` in an error message at `motion_live.py:613`), no config files read, no environment variables used.
- **No credentials or secrets** — `.gitignore` correctly excludes `.env` files, and no API keys or tokens exist in the code.
- **Clean resource lifecycle** — shared memory is zeroed on creation (`motion_live.py:624`), cleaned up on startup if stale (`motion_live.py:617-619`), and properly unlinked on shutdown (`motion_live.py:708-709`). The worker process is killed and joined on exit (`motion_live.py:683-685`).

## Low-severity observations

### 1. Entire application runs as root

**Location:** `motion_live.py:612`

Only the sensor worker needs root for IOKit HID access. The main process (UI rendering, JSON writing) doesn't need elevated privileges. Ideally, the main process would drop privileges after creating shared memory and spawning the worker. In practice, this is a minor concern for a local developer tool.

### 2. Predictable shared memory name

**Location:** `spu_sensor.py:21`

The segment is named `vib_detect_shm` — a static, predictable name. Any local process running as the same user could attach to it and read sensor data or inject false samples. This is low-risk since it requires local access and the data is non-sensitive accelerometer readings.

### 3. Silent exception swallowing in HID callback

**Location:** `spu_sensor.py:138`

```python
except Exception:
    pass
```

The `on_report` callback silently discards all exceptions. While this prevents the callback from crashing the sensor worker, it also masks bugs (e.g., malformed reports, shared memory corruption). Logging to stderr or a counter would be better.

### 4. Shared memory index used before validation

**Location:** `spu_sensor.py:27-28`

```python
idx, = struct.unpack_from('<I', buf, 0)
off = SHM_HEADER + idx * RING_ENTRY
```

The index read from shared memory is used to compute an offset before being wrapped with `% RING_CAP` on line 30. If another process corrupted the index, `struct.pack_into` would raise an exception (Python's struct module is memory-safe), which the callback silently swallows. No actual buffer overflow is possible, but the sample would be silently lost.

### 5. Root-owned log files

**Location:** `motion_live.py:703`

JSON logs are written to the current working directory as root. The filename uses `datetime.strftime()` with no user input, so there's no path traversal risk — but the resulting files will be root-owned, which is a minor usability annoyance.

### 6. Unbounded dependency version

**Location:** `requirements.txt`

`PyWavelets>=1.4` has no upper bound. Standard practice for small projects, but pinning (e.g., `>=1.4,<2.0`) would prevent unexpected breaking changes.

## Summary

| Category | Status |
|---|---|
| Remote code execution | None |
| Network exposure | None |
| Command injection | None |
| Untrusted input handling | None |
| Credential leakage | None |
| Memory safety (ctypes) | Safe — Python struct module bounds-checks |
| Privilege management | Runs entirely as root (low severity) |
| Shared memory isolation | Predictable name, world-readable (low severity) |
| Resource cleanup | Proper cleanup on shutdown |
| Dependencies | Single, well-known library |

The code is straightforward, does what it claims, and has no hidden functionality or suspicious behavior.
