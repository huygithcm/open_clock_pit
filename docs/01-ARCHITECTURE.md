# 🏗️ OpenCockpit System Architecture

## Overview

OpenCockpit is a real-time FPV (First Person View) cockpit display system for Raspberry Pi that reads telemetry from drone flight controllers and displays it on multiple LCD screens.

### Key Design Principles

1. **Modular Architecture** - Independent components (MSP reader, display modules, web UI)
2. **Multi-threaded** - Parallel processing of I/O and rendering
3. **Thread-Safe** - Shared data protected by locks
4. **Graceful Degradation** - System continues if one component fails
5. **Configuration-Driven** - Flexible display/hardware mapping

---

## System Components

```
┌─────────────────────────────────────────────────────────────┐
│                  OpenCockpit System                         │
├─────────────────────────────────────────────────────────────┤
│                                                              │
│  ┌──────────┐  ┌──────────┐  ┌──────────┐  ┌──────────┐   │
│  │ MSP Read │  │ Display  │  │ Display  │  │ Web UI   │   │
│  │ Thread   │  │ Thread 1 │  │ Thread 2 │  │ Flask    │   │
│  └────┬─────┘  └────┬─────┘  └────┬─────┘  └────┬─────┘   │
│       │             │             │             │          │
│       └─────────────┴─────────────┴─────────────┘          │
│             (Shared Data Dictionary)                        │
│             (Protected by threading.Lock)                   │
│                     │                                       │
│       ┌─────────────┼─────────────┐                         │
│       │             │             │                         │
│       ↓             ↓             ↓                         │
│  ┌────────┐  ┌────────┐  ┌────────┐  ┌────────┐           │
│  │Display │  │Display │  │Display │  │Display │           │
│  │ #1     │  │ #2     │  │ #3     │  │ #4     │           │
│  │ST7735S │  │ST7735S │  │ST7735S │  │ST7735S │           │
│  └────────┘  └────────┘  └────────┘  └────────┘           │
│                                                              │
└─────────────────────────────────────────────────────────────┘
```

---

## Component Responsibilities

| Component | Role | Thread Model |
|-----------|------|--------------|
| **MSP_Read_pi** | Read FC telemetry via serial | Daemon thread |
| **main.py** | Initialize hardware, start threads | Main thread (waits) |
| **display_loop** | Render & update displays | 4× independent threads |
| **web_app (Flask)** | Remote monitoring & config | Daemon thread |
| **HUD/MAP/MFD/INFO** | Render-specific display content | Called by display_loop |

---

## Data Flow

```
Flight Controller
    ↓ (Serial: MSP/MAVLink)
MSP_Read_pi.main()
    ↓ (threading.Lock)
Shared data dict
    ↓ (Snapshot copy)
display_loop threads
    ↓ (Pygame render)
RGB565 buffer
    ↓ (SPI @ 52MHz)
ST7735 displays
    ↓
User sees live telemetry
```

---

## Thread Synchronization

### Data Protection

```python
# Writer (MSP_Read_pi)
with data_lock:
    data["roll"] = roll_value
    data["pitch"] = pitch_value

# Reader (display_loop)
with data_lock:
    snap = dict(data)  # atomic snapshot
    
# Process outside lock
pitch = snap["pitch"] or 0.0
```

### Framerate Control

- **HUD modules**: 30 Hz (real-time attitude)
- **MAP/MFD/INFO**: 15 Hz (slower telemetry)
- **MSP requests**: 30 Hz (ATTITUDE) + 15 Hz (others)

---

## Hardware Mapping

### SPI Bus (Shared)

- **Clock**: board.SCK (GPIO 11)
- **MOSI**: board.MOSI (GPIO 10)
- **MISO**: Unused
- **Baudrate**: 52 MHz

### Display Pinouts

| Display | Driver | CS | DC | RST | Size | Module |
|---------|--------|----|----|-----|------|--------|
| Display_1 | ST7735S | GPIO 5 | GPIO 25 | GPIO 22 | 1.14" (128×128) | HUD_0.85 |
| Display_2 | ST7735S | GPIO 6 | GPIO 24 | GPIO 27 | 0.96" (160×80) | MAP_0.96 |
| Display_3 | ST7735S | GPIO 13 | GPIO 23 | GPIO 17 | 0.96" (160×80) | MFD_0.96 |
| Display_4 | ST7735S | GPIO 19 | GPIO 26 | GPIO 16 | 0.96" (160×80) | INFO_0.96 |

---

## File Structure

```
OpenCockpit/
├── main.py                    # Entry point, orchestration
├── MSP_Read_pi.py            # Serial telemetry reader (daemon)
├── web_app.py                # Flask web UI
│
├── Display Modules:
├── HUD_pi_085.py             # Attitude indicator 1.14"
├── HUD_pi_114.py             # Attitude indicator 0.96"
├── MAP_pi_096.py             # GPS map 0.96"
├── MFD_pi_096.py             # Flight instruments 0.96"
├── INFO_pi_096.py            # Power/status info 0.96"
│
├── Assets:
├── ViperDisplay-Bold.ttf     # Display font
├── config.json               # Runtime config (generated)
│
└── Templates (web UI):
├── static/
└── templates/
```

---

## Initialization Sequence

```
1. main.py starts
   └─ _kill_previous_instances()  [cleanup GPIO]
   
2. Initialize hardware
   └─ SPI bus
   └─ GPIO pins (CS, DC, RST for each display)
   
3. Load configuration
   └─ From web_app.load_config_from_disk()
   └─ Fallback to SELECTED_DISPLAYS (hardcoded)
   
4. Start daemon threads
   └─ MSP_Read_pi.main()  [serial reader]
   └─ web_app.start()     [Flask server]
   
5. For each display:
   └─ init_display()      [ST7735 init]
   └─ Thread(display_loop) [render thread]
   
6. Main thread waits
   └─ Daemon threads do all work
   └─ Ctrl+C to exit
```

---

## Error Handling

### Graceful Degradation

| Failure | Behavior |
|---------|----------|
| MSP disconnected | Displays continue with stale data |
| Display init fails | Other displays still run |
| Flask fails | Displays still work (web UI offline) |
| SPI conflict | Attempt recovery with retry logic |

### Reconnection Logic

- **MSP**: Auto-reconnect with exponential backoff (0.5s)
- **Flask**: Continuous service (no reconnect needed)
- **Displays**: Restart on serial error (rare)

---

## Configuration Sources (Priority)

1. **Environment Variables** (highest priority)
   ```bash
   MSP_PORT=/dev/ttyACM0
   MSP_BAUD=115200
   MSP_PROTOCOL=msp
   ```

2. **Web UI config** (config.json)
   ```json
   {
     "displays": { "Display_1": "HUD_0.85", ... },
     "fc": { "port": "/dev/serial0", "baudrate": 115200 }
   }
   ```

3. **Hardcoded defaults** (lowest priority)
   ```python
   SELECTED_DISPLAYS = { "Display_1": "HUD_0.85", ... }
   DISPLAY_HARDWARE_MAP = { "Display_1": {"cs": D5, ...}, ... }
   ```

---

## Performance Characteristics

| Metric | Value | Notes |
|--------|-------|-------|
| Telemetry Update Rate | 30 Hz (attitude) | Real-time |
| Display Refresh | 30 Hz (HUD), 15 Hz (others) | Smooth rendering |
| End-to-End Latency | 60-100 ms | Acceptable for FPV |
| SPI Bus Utilization | 30-50% | Plenty of headroom |
| Memory Usage | ~150-200 MB | Python overhead |
| CPU Usage | 40-60% | Multi-core efficient |

---

## Extensibility

### Adding a New Display

1. Create module: `NEW_pi_096.py`
2. Implement: `render_new()` function
3. Add to `MODULE_MAP` in main.py
4. Assign GPIO pins in `DISPLAY_HARDWARE_MAP`
5. Select via `SELECTED_DISPLAYS` or web UI

### Adding a New Data Field

1. Add to MSP request in `MSP_Read_pi.py`
2. Parse response: `parse_*()` function
3. Store in `data` dict
4. Use in display modules: `snap["field_name"]`
5. Update web API output

---

See also:
- [DATAFLOW.md](02-DATAFLOW.md) - Detailed data flow
- [HARDWARE.md](03-HARDWARE.md) - Hardware specifications
- [CONFIGURATION.md](04-CONFIGURATION.md) - Configuration guide
