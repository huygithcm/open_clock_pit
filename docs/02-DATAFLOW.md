# 🔄 Data Flow & Thread Synchronization

## MSP Reading Thread

### Main Loop (MSP_Read_pi.py:main)

```python
while True:
    now = time.time()
    
    # FAST requests @ 30 Hz
    if now - t_fast >= FAST_DT:
        send_msp_request(ser, MSP_ATTITUDE)
        t_fast = now
    
    # SLOW requests @ 15 Hz
    if now - t_slow >= SLOW_DT:
        send_msp_request(ser, MSP_ALTITUDE)
        send_msp_request(ser, MSP_RAW_GPS)
        send_msp_request(ser, MSP_ANALOG)
        send_msp_request(ser, MSP_CURRENT)
        send_msp_request(ser, MSP_COMP_GPS)
        send_msp_request(ser, MSP_RC)
        t_slow = now
    
    # Read responses
    resp = read_msp_response(ser)
    if resp:
        cmd, payload = resp
        
        # Parse & store (thread-safe)
        with data_lock:
            if cmd == MSP_ATTITUDE:
                data["roll"], data["pitch"], data["yaw"] = parse_attitude(payload)
            elif cmd == MSP_ALTITUDE:
                data["alt"], data["v_speed"] = parse_altitude(payload)
            # ... etc
```

### Data Update Rate

| Request | Frequency | Fields Updated |
|---------|-----------|-----------------|
| MSP_ATTITUDE | 30 Hz | roll, pitch, yaw |
| MSP_ALTITUDE | 15 Hz | alt, v_speed |
| MSP_RAW_GPS | 15 Hz | lat, lon, speed, sats, course |
| MSP_ANALOG | 15 Hz | vbat, current, rssi |
| MSP_COMP_GPS | 15 Hz | home_dist, home_dir |
| MSP_RC | 15 Hz | throttle |

---

## Display Render Thread

### Main Loop (main.py:display_loop)

```python
while True:
    clock.tick(fps)  # Frame rate limiter
    
    # Get thread-safe snapshot
    snap = get_msp_snapshot()
    
    # Extract with fallback values
    pitch = snap["pitch"] or 0.0
    roll = snap["roll"] or 0.0
    # ... extract all fields
    
    # Call module render
    module.render_hud(pitch, roll, yaw, v_speed, ...)
    
    # Composite surfaces
    module.screen.blit(background_surface, (0, 0))
    module.screen.blit(dynamic_surface, (0, 0))
    module.screen.blit(fixed_surface, (0, 0))
    
    # Color conversion
    raw = pygame.image.tostring(module.screen, "RGB")
    buf = rgb888_to_rgb565(raw, width, height)
    
    # Send to hardware
    disp._block(0, 0, width - 1, height - 1, buf)
```

### Framerate Control

```python
# HUD modules (30 Hz)
if "HUD" in mod_key:
    fps = HIGH_FPS  # 30

# MAP/MFD/INFO (15 Hz)
else:
    fps = LOW_FPS  # 15
```

---

## Thread Safety Pattern

### The Snapshot Pattern

```python
# Writer (MSP_Read_pi)
with data_lock:
    data["roll"] = 15.5
    data["pitch"] = -3.2

# Reader (display_loop)
with data_lock:
    snap = dict(data)  # Make a copy

# Use snapshot outside lock (no blocking)
pitch = snap["pitch"]
roll = snap["roll"]
render_hud(pitch, roll)
```

**Why?** 
- Minimizes lock time (copy only, not rendering)
- Prevents display threads from blocking each other
- Prevents stale data between reads

---

## Data Dictionary Structure

```python
data = {
    # Attitude (from FC attitude sensor)
    "roll": None,       # degrees, -180 to 180
    "pitch": None,      # degrees, -90 to 90
    "yaw": None,        # degrees, 0 to 360
    
    # Altitude (from barometer)
    "alt": None,        # meters (relative)
    "v_speed": None,    # m/s (positive = up)
    
    # Position (from GPS)
    "lat": None,        # degrees
    "lon": None,        # degrees
    "sats": None,       # count
    "course": None,     # degrees (heading)
    "speed": None,      # m/s (horizontal)
    
    # Power
    "vbat": None,       # volts
    "current": None,    # amps
    
    # Signal
    "rssi": None,       # 0-1023 (MSP) or 0-255 (MAVLink)
    "throttle": None,   # PWM 1000-2000 or percent
    
    # Home Reference
    "home_dist": None,  # meters
    "home_dir": None,   # degrees (bearing)
    
    # Derived
    "speed_3d": None    # m/s = sqrt(speed^2 + v_speed^2)
}
```

---

## Timing Analysis

### Complete Cycle Timeline

```
t=0ms      MSP_Read_pi checks timer, sends ATTITUDE request
t=1ms      FC receives request
t=2ms      FC processes (attitude calc)
t=5ms      FC transmits response (6 bytes @ 115200 baud = ~520µs)
t=6ms      MSP_Read_pi receives & checksums
t=7ms      Lock data_lock, parse attitude, update data dict
t=8ms      Release data_lock

Parallel (Display Thread):
t=0ms      display_loop checks timer
t=2ms      Lock data_lock, copy snapshot
t=3ms      Release data_lock (fast!)
t=4ms      Call render_hud(pitch, roll, yaw, ...)
t=12ms     Pygame surface rendering complete
t=13ms     RGB888 → RGB565 conversion (NumPy)
t=15ms     SPI write begins (52MHz)
t=21ms     SPI write complete
t=33ms     clock.tick(30) triggers next frame
```

### Latency Budget (Serial → Display)

| Stage | Duration | Notes |
|-------|----------|-------|
| Serial I/O (FC→Pi) | 25-40 ms | FC response time + baud delay |
| MSP Parsing | 1-2 ms | struct.unpack + lock |
| Data Lock Contention | <1 ms | dict copy operation |
| Pygame Rendering | 8-15 ms | Depends on complexity (HUD > MAP) |
| RGB Conversion | 2-3 ms | NumPy vectorized operation |
| SPI Write | 5-10 ms | 52MHz @ 160×80 pixels |
| **Total E2E** | **60-100 ms** | Acceptable for FPV (typical: 80ms) |

### Frame Rate Timeline

```
Display 1 (HUD, 30 Hz):  Frame │ 33.3ms │ 33.3ms │ 33.3ms │
Display 2 (MAP, 15 Hz):  Frame │        66.7ms        │
Display 3 (MFD, 15 Hz):  Frame │        66.7ms        │
Display 4 (INFO, 15 Hz): Frame │        66.7ms        │

MSP Reader (30 Hz):      Attitude every 33.3ms
                         GPS/Alt every 66.7ms
```

**Key:** Different displays run independently; no global frame sync.

---

## Lock Contention Analysis

### Scenario: 4 Display Threads + 1 MSP Reader

```
Display_1.read()  ─────────────────────────────────────
Display_2.read()  ─────────────────────────────────────
Display_3.read()  ─────────────────────────────────────
Display_4.read()  ─────────────────────────────────────
MSP_Read.write()  ─────────────────────────────────────

Lock timeline:
├─ Display_1: Lock at t=10ms (1µs to copy)
├─ Display_2: Lock at t=15ms (1µs to copy)
├─ MSP_Read: Lock at t=8ms (1µs to write)
├─ Display_3: Lock at t=20ms (1µs to copy)
├─ Display_4: Lock at t=25ms (1µs to copy)

Conclusion: Contention is negligible (<1µs total per frame)
```

---

## Error Recovery

### Serial Disconnection

```python
# MSP_Read_pi.py
while True:
    try:
        resp = read_msp_response(ser)
        # ... process
    except (serial.SerialException, OSError):
        ser.close()
        # Reconnect with backoff
        ser, port = _open_msp_serial_blocking("reconnected")
```

**Behavior:** Displays keep rendering with stale data (no crash)

### Display Hardware Failure

```python
# main.py
try:
    disp_hw = init_display(disp_id, module_obj)
    loop = threading.Thread(...)
    loop.start()
except Exception as e:
    print(f"Failed to init {disp_id}: {e}")
    # Continue with other displays
```

**Behavior:** Other displays still render normally

---

## Optimization Opportunities

### Current Performance

- MSP reader: 1 thread, 100% core utilization
- Display threads: 4 threads, parallel (shared SPI)
- Total render time: ~40-50ms for all 4 displays
- Idle time: ~50% (waiting for locks/timers)

### Possible Improvements

1. **DMA SPI writes** - Offload SPI to hardware (reduce CPU)
2. **Multi-core MSP** - Separate requests/responses into different threads
3. **Pygame caching** - Cache static text renders
4. **Reduced polling** - Adaptive refresh rates (lower when flying steady)

---

See also:
- [ARCHITECTURE.md](01-ARCHITECTURE.md) - System design
- [HARDWARE.md](03-HARDWARE.md) - Hardware specs
