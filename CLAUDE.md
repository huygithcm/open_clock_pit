# 🚁 OpenCockpit - FPV Cockpit Display System

**OpenCockpit** is a real-time FPV (First Person View) cockpit display system for Raspberry Pi. It reads telemetry from drone flight controllers (INAV or ArduPilot) and displays live flight instruments on multiple LCD screens.

## 🎯 What It Does

- **Real-time telemetry display** - Attitude, altitude, GPS, power status
- **Multiple independent displays** - HUD (attitude), MAP (GPS), MFD (instruments), INFO (power)
- **Flexible hardware mapping** - Configure GPIO pins and display modules via config
- **Network control** - Web UI for monitoring and remote configuration
- **Robust design** - Graceful degradation, auto-reconnection, thread-safe

## 🏗️ Project Structure

```
OpenCockpit/
├── docs/                       # Complete technical documentation
│   ├── INDEX.md               # Documentation home
│   ├── 01-ARCHITECTURE.md     # System design & components
│   ├── 02-DATAFLOW.md         # Telemetry flow & threading
│   ├── 03-HARDWARE.md         # GPIO & display specs
│   └── 04-CONFIGURATION.md    # Settings & hardcoding fixes
│
├── OpenCockpit/               # Main application
│   ├── main.py                # Entry point, orchestration
│   ├── MSP_Read_pi.py         # Serial telemetry reader
│   ├── web_app.py             # Flask web UI
│   ├── HUD_pi_*.py            # Attitude indicator modules
│   ├── MAP_pi_096.py          # GPS map display
│   ├── MFD_pi_096.py          # Flight instruments
│   ├── INFO_pi_096.py         # Power & status info
│   ├── config.json            # Runtime configuration (generated)
│   └── ViperDisplay-Bold.ttf  # Display font
│
├── venv/                      # Python virtual environment
├── requirements.txt           # Dependencies
├── README.md                  # Project overview
├── CLAUDE.md                  # This file (project guide)
└── DATAFLOW.md                # Quick reference (original format)
```

## 📖 Documentation

**Start here:** [docs/INDEX.md](docs/INDEX.md) - Complete documentation home

### Quick Navigation

- **System Overview:** [docs/01-ARCHITECTURE.md](docs/01-ARCHITECTURE.md)
- **How Data Flows:** [docs/02-DATAFLOW.md](docs/02-DATAFLOW.md)
- **Hardware Setup:** [docs/03-HARDWARE.md](docs/03-HARDWARE.md)
- **Configuration & Settings:** [docs/04-CONFIGURATION.md](docs/04-CONFIGURATION.md)

## 🚀 Quick Start

### 1. Install Dependencies

```bash
cd ~/Desktop/open_clock_pit
source venv/bin/activate
pip install -r requirements.txt
```

### 2. Run the Application

```bash
# Standard startup
python OpenCockpit/main.py

# With custom serial port
export MSP_PORT=/dev/ttyACM0
python OpenCockpit/main.py

# With debug logging
python OpenCockpit/main.py 2>&1 | tee opencockpit.log
```

### 3. Access Web UI

```
http://<raspberry-pi-ip>:5000
```

## ⚙️ Configuration

### Option 1: Web UI (Recommended)

1. Open web UI: http://rpi:5000
2. Select displays and FC settings
3. Changes auto-saved to `config.json`

### Option 2: Manual config.json

```json
{
  "displays": {
    "Display_1": "HUD_0.85",
    "Display_2": "MAP_0.96",
    "Display_3": "MFD_0.96",
    "Display_4": "INFO_0.96"
  },
  "hardware": {
    "gpio_pins": {
      "Display_1": {"cs": 5, "dc": 25, "rst": 22},
      "Display_2": {"cs": 6, "dc": 24, "rst": 27},
      "Display_3": {"cs": 13, "dc": 23, "rst": 17},
      "Display_4": {"cs": 19, "dc": 26, "rst": 16}
    }
  },
  "fc": {
    "port": "/dev/serial0",
    "baudrate": 115200,
    "protocol": "auto"
  }
}
```

### Option 3: Environment Variables

```bash
export MSP_PORT=/dev/ttyACM0         # Serial port
export MSP_BAUD=115200               # Baud rate
export MSP_PROTOCOL=msp              # Protocol (msp|mavlink)
export MSP_RECONNECT_IDLE_S=3.0      # Reconnect timeout
export MSP_RECONNECT_BACKOFF_S=0.5   # Reconnect delay
```

## 🔌 Hardware Mapping

### GPIO Pins Used

```
Raspberry Pi GPIO:
├─ SCK (GPIO 11):   SPI Clock (all displays)
├─ MOSI (GPIO 10):  SPI Data (all displays)
│
├─ Display 1 (HUD 1.14"):
│  ├─ CS (GPIO 5), DC (GPIO 25), RST (GPIO 22)
│
├─ Display 2 (MAP 0.96"):
│  ├─ CS (GPIO 6), DC (GPIO 24), RST (GPIO 27)
│
├─ Display 3 (MFD 0.96"):
│  ├─ CS (GPIO 13), DC (GPIO 23), RST (GPIO 17)
│
└─ Display 4 (INFO 0.96"):
   ├─ CS (GPIO 19), DC (GPIO 26), RST (GPIO 16)
```

See [docs/03-HARDWARE.md](docs/03-HARDWARE.md) for complete specifications.

## 📊 System Architecture

```
Flight Controller (INAV/ArduPilot)
    ↓ Serial (MSP/MAVLink)
Raspberry Pi
├─ MSP_Read_pi.py [Daemon Thread]
│  ├─ Read telemetry @ 30Hz
│  ├─ Update shared data dict
│  └─ Thread-safe (threading.Lock)
│
├─ display_loop() [4× Parallel Threads]
│  ├─ HUD: 30Hz refresh
│  ├─ MAP: 15Hz refresh
│  ├─ MFD: 15Hz refresh
│  └─ INFO: 15Hz refresh
│
└─ web_app (Flask) [Daemon Thread]
   ├─ /api/telemetry
   ├─ /api/state
   └─ /api/display/<id>.png
    ↓
4× LCD Displays (ST7735S @ 52MHz SPI)
```

See [docs/01-ARCHITECTURE.md](docs/01-ARCHITECTURE.md) for design details.

## ⏱️ Performance

| Metric | Value | Notes |
|--------|-------|-------|
| Telemetry Rate | 30 Hz (ATTITUDE) | Real-time |
| Display Refresh | 30 Hz (HUD), 15 Hz (MAP/MFD/INFO) | Smooth |
| E2E Latency | 60-100 ms | Acceptable for FPV |
| CPU Usage | 40-60% | Multi-core efficient |
| Memory | 150-200 MB | Python overhead |

See [docs/02-DATAFLOW.md](docs/02-DATAFLOW.md) for detailed timing analysis.

## 🛠️ Development

### Adding Display Modules

1. Create new module: `NEW_MODULE_pi_SIZE.py`
2. Implement render function: `render_hud()` / `render_map()` / etc.
3. Add to `MODULE_MAP` in main.py
4. Assign GPIO pins in config.json
5. Enable via `SELECTED_DISPLAYS`

### Adding Data Fields

1. Add MSP request to `MSP_Read_pi.py`
2. Implement parse function
3. Store in shared `data` dict
4. Use in display modules: `snap["field_name"]`
5. Expose in web API

See [docs/01-ARCHITECTURE.md](docs/01-ARCHITECTURE.md#extensibility) for details.

## ⚠️ Known Hardcoding Issues

Current status: Some settings are hardcoded in Python files.

### Priority Fixes

🔴 **HIGH:** GPIO pin mapping (affects hardware compatibility)
- Currently: Hardcoded in main.py
- Fix: Load from config.json
- See: [docs/04-CONFIGURATION.md](docs/04-CONFIGURATION.md)

🟡 **MEDIUM:** Display selection (needs web UI to change)
- Currently: Hardcoded in main.py, overridden by config.json
- Fix: Load from config.json at startup
- See: [docs/04-CONFIGURATION.md](docs/04-CONFIGURATION.md)

🟡 **MEDIUM:** Framerate settings (can't adjust without editing code)
- Currently: `HIGH_FPS=30`, `LOW_FPS=15`
- Fix: Move to config.json
- See: [docs/04-CONFIGURATION.md](docs/04-CONFIGURATION.md)

🟢 **LOW:** Serial port & baud rate (already well-configured)
- ✅ Supports env vars, config.json, auto-detection

See [docs/04-CONFIGURATION.md](docs/04-CONFIGURATION.md) for complete analysis and action items.

## 🐛 Troubleshooting

### Display Not Showing

1. Check GPIO pins in config.json
2. Verify SPI bus available: `lsof /dev/spidev0.0`
3. Check power supply: 3.3V stable
4. Look for errors: `python OpenCockpit/main.py 2>&1 | head -50`

### No Telemetry

1. Verify serial port: `ls /dev/ttyACM0` or `/dev/serial0`
2. Check baud rate: `stty -F /dev/ttyACM0 speed`
3. Test FC: `screen /dev/ttyACM0 115200` (see data flowing)
4. Verify protocol: Check FC is using MSP or MAVLink

### Flickering

1. Reduce framerate in config.json
2. Try lower SPI speed: change `52000000` to `26000000`
3. Check for GPIO conflicts

Full troubleshooting: [docs/03-HARDWARE.md#troubleshooting](docs/03-HARDWARE.md#troubleshooting)

## 📈 Monitoring

### Check Live Telemetry

```bash
curl http://localhost:5000/api/telemetry | jq
```

### Get Display Screenshot

```bash
curl http://localhost:5000/api/display/Display_1.png > hud.png
```

### View System State

```bash
curl http://localhost:5000/api/state | jq
```

## 🔄 Thread Synchronization

### Safe Data Access

All display threads access shared `data` dict via `threading.Lock()`:

```python
# Writer (MSP_Read_pi)
with data_lock:
    data["roll"] = roll_value

# Reader (display_loop)
with data_lock:
    snap = dict(data)  # atomic snapshot

# Use outside lock (no blocking)
pitch = snap["pitch"]
render_hud(pitch, ...)
```

See [docs/02-DATAFLOW.md#thread-safety-pattern](docs/02-DATAFLOW.md#thread-safety-pattern) for details.

## 📋 Dependencies

```
Adafruit-Blinka==9.0.4              # GPIO abstraction
adafruit-circuitpython-*             # Display drivers
pygame==2.6.1                        # Rendering
pyserial==3.5                        # Serial I/O
numpy==2.4.4                         # Math
Flask==3.1.3                         # Web UI
RPi.GPIO==0.7.1                      # GPIO control
rpi_ws281x==5.0.0                    # NeoPixel (if used)
```

## 🚀 Production Deployment

### Run as System Service

```bash
# Create /etc/systemd/system/opencockpit.service
[Unit]
Description=OpenCockpit FPV Display
After=network.target

[Service]
Type=simple
User=pi
WorkingDirectory=/home/pi/Desktop/open_clock_pit
ExecStart=/home/pi/Desktop/open_clock_pit/venv/bin/python \
          /home/pi/Desktop/open_clock_pit/OpenCockpit/main.py
Restart=on-failure
RestartSec=5

[Install]
WantedBy=multi-user.target

# Enable & start
sudo systemctl enable opencockpit
sudo systemctl start opencockpit
```

### Monitor Logs

```bash
sudo journalctl -u opencockpit -f
```

## 📞 Getting Help

1. **Check documentation:** Start with [docs/INDEX.md](docs/INDEX.md)
2. **Common issues:** [docs/03-HARDWARE.md#troubleshooting](docs/03-HARDWARE.md#troubleshooting)
3. **Configurations:** [docs/04-CONFIGURATION.md](docs/04-CONFIGURATION.md)
4. **Architecture:** [docs/01-ARCHITECTURE.md](docs/01-ARCHITECTURE.md)

## 📝 Project Status

- ✅ Multi-display support (4 displays)
- ✅ Real-time telemetry (30Hz ATTITUDE)
- ✅ Web UI dashboard
- ✅ Thread-safe data access
- ✅ Auto-reconnection
- ⚠️ Configuration (partially hardcoded - see [docs/04-CONFIGURATION.md](docs/04-CONFIGURATION.md))
- 🔄 Performance optimization (room for improvement)

## 🔗 Quick Links

| Document | Purpose |
|----------|---------|
| [docs/INDEX.md](docs/INDEX.md) | Documentation home |
| [docs/01-ARCHITECTURE.md](docs/01-ARCHITECTURE.md) | System design |
| [docs/02-DATAFLOW.md](docs/02-DATAFLOW.md) | Data flow & timing |
| [docs/03-HARDWARE.md](docs/03-HARDWARE.md) | Hardware specs |
| [docs/04-CONFIGURATION.md](docs/04-CONFIGURATION.md) | Settings & hardcoding |
| [DATAFLOW.md](DATAFLOW.md) | Quick reference (original) |
| [README.md](README.md) | Project overview |

---

**Version:** 1.0  
**Last Updated:** 2026-05-28  
**Status:** Production Ready (with configuration recommendations)

For latest updates, see [docs/INDEX.md](docs/INDEX.md).
