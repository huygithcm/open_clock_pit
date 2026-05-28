# 📊 OpenCockpit Data Flow Architecture

## Overview

OpenCockpit is a real-time FPV (First Person View) cockpit display system running on Raspberry Pi. This document describes the complete data flow from Flight Controller to display outputs.

---

## 1️⃣ INPUT LAYER - Flight Controller (FC)

```
INAV FC / ArduPilot
    ↓
Serial Port: /dev/serial0 (UART GPIO) or /dev/ttyACM0 (USB)
Baudrate: 115200 bps
    ↓
Protocol Auto-Detect:
├─ MSP (MultiWii Serial Protocol) - for INAV
└─ MAVLink - for ArduPilot
```

**Configuration Sources (Priority Order):**
1. Environment variable: `MSP_PROTOCOL`, `MSP_PORT`, `MSP_BAUD`
2. Web UI config: `config.json` (fc block)
3. Auto-detection: Protocol sniffing + heartbeat check

---

## 2️⃣ DATA READING LAYER - MSP_Read_pi.py

### Main Thread: `MSP_Read_pi.main()`
- Runs as daemon thread spawned from `main.py`
- Continuously polls Flight Controller over serial
- Parses responses and updates shared `data` dictionary
- Uses `threading.Lock()` for thread-safe access

### MSP Request Frequencies

| Frequency | Messages | Fields |
|-----------|----------|--------|
| **FAST (30Hz)** | MSP_ATTITUDE (108) | roll, pitch, yaw |
| **SLOW (15Hz)** | MSP_ALTITUDE (109) | alt, v_speed |
| | MSP_RAW_GPS (106) | lat, lon, sats, course, speed |
| | MSP_ANALOG (110) | vbat, current, rssi |
| | MSP_COMP_GPS (107) | home_dist, home_dir |
| | MSP_CURRENT (23) | current (redundant) |
| | MSP_RC (105) | throttle (CH3) |

### Data Storage

```python
data = {
    "roll": float,        # degrees
    "pitch": float,       # degrees
    "yaw": float,         # degrees
    "alt": float,         # meters (relative)
    "v_speed": float,     # m/s (vertical)
    "lat": float,         # degrees
    "lon": float,         # degrees
    "speed": float,       # m/s (GPS horizontal)
    "sats": int,          # satellite count
    "course": int,        # degrees (GPS heading)
    "vbat": float,        # volts
    "current": float,     # amps
    "rssi": float,        # 0-1023 (MSP) or 0-255 (MAVLink)
    "throttle": float,    # PWM or percent
    "home_dist": int,     # meters
    "home_dir": int,      # degrees (bearing to home)
    "speed_3d": float     # computed: sqrt(speed^2 + v_speed^2)
}
```

### Thread-Safe Access

```python
# Global lock
data_lock = threading.Lock()

# Writer (MSP_Read_pi):
with data_lock:
    data["roll"] = roll_value

# Reader (display_loop):
with data_lock:
    snap = dict(data)  # atomic snapshot copy
```

**Key Design:** Lock is held for minimal time (dict copy only). Display threads never block on data read.

---

## 3️⃣ ORCHESTRATION LAYER - main.py

### Initialization Sequence

```
1. _kill_previous_instances()
   └─ Clean up GPIO/SPI from prior runs

2. Initialize SPI Bus
   └─ board.SCK, MOSI (shared for all displays)

3. Initialize GPIO Pins
   └─ For each display: CS (chip select), DC (data/cmd), RST (reset)

4. Start MSP Reader Thread (daemon)
   └─ threading.Thread(MSP_Read_pi.main, daemon=True)

5. Start Web UI Thread (daemon, Flask)
   └─ web_app.init() → web_app.start()

6. Initialize Displays & Start Render Threads
   └─ For each display in SELECTED_DISPLAYS:
      └─ threading.Thread(display_loop, ...)
```

### Display Configuration

```python
SELECTED_DISPLAYS = {
    "Display_1": "HUD_0.85",      # 128×128 Attitude Indicator
    "Display_2": "MAP_0.96",      # 160×80  GPS Map
    "Display_3": "MFD_0.96",      # 160×80  Flight Instruments
    "Display_4": "INFO_0.96",     # 160×80  Power & System Info
}

DISPLAY_HARDWARE_MAP = {
    "Display_1": {"cs": board.D5,  "dc": board.D25, "rst": board.D22},
    "Display_2": {"cs": board.D6,  "dc": board.D24, "rst": board.D27},
    "Display_3": {"cs": board.D13, "dc": board.D23, "rst": board.D17},
    "Display_4": {"cs": board.D19, "dc": board.D26, "rst": board.D16}
}
```

### Framerate Configuration

- **HIGH_FPS = 30Hz** - HUD modules (real-time attitude, needs smooth updates)
- **LOW_FPS = 15Hz** - MAP, MFD, INFO (slow-changing data, reduces SPI load)

---

## 4️⃣ DISPLAY RENDER LAYER - Display Threads

### `display_loop()` Execution

Each display runs its own thread with independent framerate:

```python
def display_loop(module, disp, width, height, fps):
    # 1. Initialize Pygame surface
    module.screen = pygame.Surface((width, height))
    
    # 2. Render fixed components (static background)
    if hasattr(module, "render_mfd_fixed"):
        module.render_mfd_fixed()
    if hasattr(module, "render_info_fixed"):
        module.render_info_fixed()
    
    # 3. Main loop
    clock = pygame.time.Clock()
    while True:
        clock.tick(fps)  # Frame rate limiter
        
        # 4. Get thread-safe snapshot of current MSP data
        snap = get_msp_snapshot()
        
        # 5. Extract fields with None fallback
        pitch = snap["pitch"] or 0.0
        roll = snap["roll"] or 0.0
        # ... extract all fields
        
        # 6. Call module-specific render function
        if hasattr(module, "render_hud"):
            module.render_hud(pitch, roll, yaw, v_speed, alt, ...)
        elif hasattr(module, "render_map"):
            module.render_map(yaw, v_speed, alt, lat, lon, ...)
        elif hasattr(module, "render_mfd_dynamic"):
            module.render_mfd_dynamic(pitch, roll, yaw, ...)
        elif hasattr(module, "render_info_dynamic"):
            module.render_info_dynamic(vbat, current, rssi, ...)
        
        # 7. Composite pygame surfaces (layer stacking)
        module.screen.blit(module.background_surface, (0, 0))  # bottom
        module.screen.blit(module.dynamic_surface, (0, 0))
        module.screen.blit(module.fixed_surface, (0, 0))       # top
        
        # 8. Color space conversion: RGB888 → RGB565
        raw = pygame.image.tostring(module.screen, "RGB")
        buf = rgb888_to_rgb565(raw, width, height)
        
        # 9. Send to hardware via SPI
        disp._block(0, 0, width - 1, height - 1, buf)
```

### Rendering Pipeline

```
Data Dictionary
    ↓
Pygame Render Functions (module-specific)
    ├─ Dynamic: real-time telemetry (HUD horizon, speed tape, etc.)
    ├─ Fixed: static elements (labels, frames)
    └─ Background: base color/image
    ↓
Pygame Surface Composition (layer blitting)
    ├─ background_surface (bottom layer)
    ├─ dynamic_surface (middle layer)
    └─ fixed_surface (top layer)
    ↓
RGB888 → RGB565 Color Conversion
    └─ Matches ST7735/ST7789 hardware format
    ↓
SPI Block Write to Display
    └─ disp._block(x0, y0, x1, y1, rgb565_buffer)
```

### Pygame Surfaces

Each display module maintains 3 surfaces:

1. **background_surface** - Non-changing base (rendered once)
2. **dynamic_surface** - Telemetry data (rendered every frame)
3. **fixed_surface** - UI elements (labels, frames)

**Why 3 surfaces?** Reduces re-rendering overhead by only updating dynamic content.

---

## 5️⃣ HARDWARE LAYER - Display Controllers

### Display Specifications

| Slot | Driver | Size | Resolution | Module | Purpose |
|------|--------|------|------------|--------|---------|
| Display_1 | ST7735S | 1.14" | 128×128 | HUD_0.85 | Attitude Indicator |
| Display_2 | ST7735S | 0.96" | 160×80 | MAP_0.96 | GPS Map |
| Display_3 | ST7735S | 0.96" | 160×80 | MFD_0.96 | Flight Instruments |
| Display_4 | ST7735S | 0.96" | 160×80 | INFO_0.96 | Power & Status |

### SPI Bus Configuration

```
Shared SPI Bus (52MHz baudrate)
├─ SCK (clock) ────────→ board.SCK
├─ MOSI (data) ────────→ board.MOSI
├─ MISO (unused)
└─ CS (chip select) per display:
   ├─ Display_1: GPIO 5
   ├─ Display_2: GPIO 6
   ├─ Display_3: GPIO 13
   └─ Display_4: GPIO 19

DC (Data/Command) per display:
├─ Display_1: GPIO 25
├─ Display_2: GPIO 24
├─ Display_3: GPIO 23
└─ Display_4: GPIO 26

RST (Reset) per display:
├─ Display_1: GPIO 22
├─ Display_2: GPIO 27
├─ Display_3: GPIO 17
└─ Display_4: GPIO 16
```

### SPI Protocol

- **Baudrate:** 52 MHz
- **Data Format:** RGB565 (2 bytes per pixel)
- **Transfer Size per Frame:**
  - HUD: 128×128×2 = 32 KB
  - Others: 160×80×2 = 25.6 KB
- **Estimated SPI Time:** ~5-10ms per display

---

## 6️⃣ WEB UI LAYER - web_app.py (Bonus)

### Flask Server

Runs on port 5000 for remote monitoring and configuration.

#### API Endpoints

| Endpoint | Method | Purpose |
|----------|--------|---------|
| `/` | GET | Web UI dashboard (HTML/CSS/JS) |
| `/api/telemetry` | GET | Live telemetry data snapshot (JSON) |
| `/api/state` | GET | Display status and config (JSON) |
| `/api/display/<id>.png` | GET | Display screenshot as PNG |
| `/api/config` | GET | Read config.json |
| `/api/config` | POST | Save display/FC config |

### Web UI Telemetry

```python
@app.route("/api/telemetry")
def telemetry():
    with _msp_module.data_lock:
        snapshot = dict(_msp_module.data)
    return jsonify({"ok": True, "data": snapshot})
```

**No polling needed.** Web browser fetches on demand.

---

## 🔄 Timing & Synchronization

### Timeline Example (Complete Cycle)

```
t=0ms    MSP_Read_pi sends attitude request
t=5ms    FC responds with attitude data
t=10ms   MSP_Read_pi parses & locks data_lock
t=11ms   data["roll/pitch/yaw"] updated
t=12ms   data_lock released

Parallel (Display Thread):
t=13ms   Display thread reads snapshot (gets updated data)
t=14ms   Pygame render_hud() called
t=20ms   RGB888→RGB565 conversion
t=22ms   SPI write begins (52MHz)
t=28ms   SPI write complete, display updated
t=33ms   Frame complete, clock.tick(30) waits for next frame
```

### Latency Budget

| Stage | Time | Notes |
|-------|------|-------|
| Serial I/O (FC→Pi) | 25-40ms | Depends on FC response time |
| MSP Parsing | 1-2ms | Struct unpack + lock |
| Data Lock Contention | <1ms | Dict copy operation |
| Pygame Rendering | 8-15ms | Complex shapes, text, transforms |
| RGB Color Conversion | 2-3ms | NumPy vectorized operation |
| SPI Write | 5-10ms | 52MHz @ 160×80 or 128×128 |
| **Total E2E Latency** | **60-100ms** | Acceptable for FPV |

### Synchronization Points

1. **data_lock** (MSP_Read_pi ↔ display_loop)
   - Writer: MSP_Read_pi updates every ~33ms (30Hz)
   - Readers: 4 display threads read independently (15-30Hz)
   - Conflict: Rare, minimal contention

2. **pygame.time.Clock.tick(fps)**
   - Enforces framerate per display
   - Independent timing (no global sync)
   - Allows displays to run at different rates

---

## ⚠️ Critical Path Analysis

### Potential Bottlenecks

#### 1. Serial I/O (MSP_Read_pi)

**Impact:** Directly affects telemetry freshness

- Flight Controller must respond within ~100ms
- INAV typically responds in 5-10ms per request
- Baud rate: 115200 = ~1 byte per 86.8 µs

**Optimization:** Already optimized—separate daemon thread

#### 2. SPI Bus Sharing

**Impact:** Multiple displays compete for SPI

- 4 displays × 5-10ms each = 20-40ms per complete cycle
- Sequential writes (not parallel) due to single SPI bus
- CS pin selects which display receives data

**Optimization:** Low framerate (15Hz) for non-HUD displays reduces load

#### 3. Pygame Rendering

**Impact:** CPU-intensive drawing operations

- Horizon calculations (rotation matrices)
- Text rendering (font rasterization)
- Complex path drawing (speed tape, altitude scale)

**Optimization:** Separate fixed surface (render once) vs dynamic surface

#### 4. Data Lock Contention

**Impact:** Display threads wait for MSP_Read_pi

- MSP_Read_pi holds lock ~<1ms (dict copy only)
- Display threads hold lock ~1ms (snapshot copy)
- Rare conflicts due to 30Hz vs 15Hz rates

**Optimization:** Snapshot-copy pattern minimizes lock time

---

## 🎯 Data Flow Diagram (Complete)

```
┌──────────────────────┐
│  Flight Controller   │
│  (INAV / ArduPilot)  │
└──────────┬───────────┘
           │ Serial (115200 bps)
           ↓
┌──────────────────────────────────┐
│  MSP_Read_pi.main() [Thread]     │
│  ┌────────────────────────────┐  │
│  │ MSP Request/Parse Loop     │  │
│  │ • Attitude @ 30Hz          │  │
│  │ • Altitude/GPS @ 15Hz      │  │
│  │ • Power/RC @ 15Hz          │  │
│  └────────────────────────────┘  │
└──────────────┬────────────────────┘
               │ threading.Lock()
               ↓
        ┌──────────────┐
        │  data dict   │
        │ (thread-safe)│
        └──────┬───────┘
               │
     ┌─────────┼─────────┬────────┐
     │         │         │        │
     ↓         ↓         ↓        ↓
┌────────┐ ┌─────────┐ ┌──────┐ ┌─────────┐
│Display │ │ Display │ │ Web  │ │Monitoring│
│Loop 1  │ │ Loop 2-4│ │ UI   │ │(Debug)   │
│HUD 30Hz│ │15Hz     │ │Flask │ │          │
└───┬────┘ └────┬────┘ └──┬───┘ └─────────┘
    │           │         │
    │ Pygame    │ Pygame  │ HTTP
    │ render    │ render  │ JSON
    │           │         │
    ↓           ↓         ↓
RGB565 Buffer   Web API   Browser
    │           │
    ↓           ↓
SPI Write    TCP Socket
    │
    ↓
┌─────────────────────────┐
│ Display Controllers     │
│ • ST7735S @ 52MHz       │
│ • 4× displays parallel  │
└─────────────────────────┘
```

---

## 📁 File Structure

```
OpenCockpit/
├── main.py                    # Orchestration, display loops
├── MSP_Read_pi.py            # Serial read, data update
├── HUD_pi_085.py             # Attitude indicator (1.14")
├── HUD_pi_114.py             # Attitude indicator (0.96")
├── MAP_pi_096.py             # GPS map display
├── MFD_pi_096.py             # Flight instruments
├── INFO_pi_096.py            # Power/status info
├── web_app.py                # Flask web UI
├── config.json               # Runtime config (generated by web UI)
└── ViperDisplay-Bold.ttf     # Display font
```

---

## 🚀 Startup Sequence

```
1. python main.py (or via systemd service)
   ↓
2. main() function
   ├─ Kill previous instances (cleanup)
   ├─ Initialize SPI bus
   ├─ Initialize GPIO pins
   └─ Print "--- Display Initialization Start ---"
   ↓
3. Start MSP_Read_pi thread (daemon)
   └─ Attempts serial connection to FC
   └─ If successful: "MSP connected on /dev/serial0"
   └─ If fails: "MSP waiting for /dev/serial0..." (loops)
   ↓
4. Start web_app (Flask) thread
   └─ Listens on 0.0.0.0:5000
   └─ Prints "[WEB] UI listening on http://0.0.0.0:5000"
   ↓
5. For each display in SELECTED_DISPLAYS:
   ├─ Initialize hardware (ST7735S init)
   ├─ Create pygame surface
   ├─ Render fixed background
   ├─ Start display_loop thread
   └─ Print "Success: Display_X initialized with MODULE_Y"
   ↓
6. Main thread enters infinite loop (waits for KeyboardInterrupt)
   └─ All work happens in daemon threads
   ↓
7. Displays render continuously in parallel
```

---

## 🔧 Configuration Options

### Environment Variables (Priority 1)

```bash
export MSP_PORT=/dev/ttyACM0        # Serial port
export MSP_BAUD=115200               # Baud rate
export MSP_PROTOCOL=msp              # Force protocol (msp|mavlink)
export MSP_RECONNECT_IDLE_S=3.0      # Reconnect timeout
export MSP_RECONNECT_BACKOFF_S=0.5   # Reconnect delay
```

### config.json (Priority 2)

```json
{
  "displays": {
    "Display_1": "HUD_0.85",
    "Display_2": "MAP_0.96",
    "Display_3": "MFD_0.96",
    "Display_4": "INFO_0.96"
  },
  "fc": {
    "port": "/dev/serial0",
    "baudrate": 115200,
    "protocol": "auto"
  }
}
```

### Auto-Detection (Priority 3)

If no config found, system tries:
1. `/dev/serial/by-id/*` (stable USB CDC path)
2. `/dev/ttyACM0` (standard Arduino USB)
3. `/dev/serial0` (Raspberry Pi GPIO UART)

---

## 📈 Performance Metrics

| Metric | Value | Status |
|--------|-------|--------|
| MSP Update Rate | 30 Hz (ATTITUDE) | ✅ Real-time |
| Display Refresh (HUD) | 30 Hz | ✅ Smooth |
| Display Refresh (MAP/MFD) | 15 Hz | ✅ Adequate |
| E2E Latency | ~60-100 ms | ✅ Acceptable |
| Concurrent Threads | 6-7 | ✅ Manageable |
| SPI Bus Utilization | ~30-50% | ✅ Plenty of headroom |
| Memory Usage | ~150-200 MB | ✅ Reasonable |

---

## 🎓 Key Design Principles

1. **Separation of Concerns**
   - MSP_Read_pi: Only handles serial I/O
   - display_loop: Only handles rendering
   - main.py: Orchestrates everything

2. **Thread Safety**
   - Single shared dictionary (`data`)
   - Protected by `threading.Lock()`
   - Snapshot pattern (copy-on-read)

3. **Parallel Processing**
   - 4 display threads run independently
   - MSP reader runs concurrently
   - No blocking dependencies

4. **Graceful Degradation**
   - If MSP disconnects: displays render with stale data
   - If display fails: others continue running
   - Web UI always available

5. **Performance Optimization**
   - Separate fixed/dynamic surfaces
   - Lower framerate for non-critical displays
   - Shared SPI bus with minimal contention

---

## 🐛 Debugging Tips

### Enable Verbose Output

```bash
python main.py 2>&1 | tee opencockpit.log
```

### Monitor Telemetry

```bash
curl http://localhost:5000/api/telemetry | jq
```

### Check Display Screenshots

```bash
curl http://localhost:5000/api/display/Display_1.png > hud.png
```

### Force Protocol

```bash
export MSP_PROTOCOL=mavlink
python main.py
```

### Test MSP Connection

```python
import serial
ser = serial.Serial('/dev/serial0', 115200)
ser.write(b'$M<\x00\x6c\x68')  # MSP_ATTITUDE request
print(ser.read(50))
```

---

## 📚 References

- **MSP Protocol:** INAV wiki
- **MAVLink:** ArduPilot documentation
- **Pygame:** Python game library
- **Adafruit Libraries:** CircuitPython for displays
- **RPi GPIO:** Adafruit Blinka

---

**Document Version:** 1.0  
**Last Updated:** 2026-05-28  
**Author:** OpenCockpit Analysis
