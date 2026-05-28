# How to Run OpenCockpit

This guide explains how to set up and run the OpenCockpit FPV cockpit display system on your Raspberry Pi.

## Prerequisites

- Raspberry Pi (4B or newer recommended)
- Python 3.8+
- Virtual environment set up
- All dependencies installed via `requirements.txt`
- Flight controller connected to serial port (INAV or ArduPilot)
- SPI bus enabled on Raspberry Pi

## Quick Start

### 1. Activate Virtual Environment

```bash
cd ~/Desktop/open_clock_pit
source venv/bin/activate
```

### 2. Run the Application

```bash
python OpenCockpit/main.py
```

This starts:
- **MSP/MAVLink telemetry reader** - reads data from flight controller
- **4× display render threads** - renders HUD, MAP, MFD, INFO
- **Flask web UI** - available at `http://<raspberry-pi-ip>:5000`

## Running Methods

### Method 1: Direct Execution (Development)

```bash
cd ~/Desktop/open_clock_pit
source venv/bin/activate
python OpenCockpit/main.py
```

**Output:**
```
[FC] port=/dev/ttyACM0 baud=115200 protocol=auto
--- Display Initialization Start ---
Success: Display_1 initialized with HUD_0.85
Success: Display_2 initialized with MAP_0.96
Success: Display_3 initialized with MFD_0.96
Success: Display_4 initialized with INFO_0.96
--- 4 Displays Running ---
```

### Method 2: With Debug Logging

```bash
cd ~/Desktop/open_clock_pit
source venv/bin/activate
python OpenCockpit/main.py 2>&1 | tee opencockpit.log
```

This captures all output to both console and `opencockpit.log` file.

### Method 3: With Custom Serial Port

```bash
export MSP_PORT=/dev/ttyACM0
python OpenCockpit/main.py
```

Available environment variables:
- `MSP_PORT` - Serial port (default: auto-detect)
- `MSP_BAUD` - Baud rate (default: 115200)
- `MSP_PROTOCOL` - Protocol: `msp`, `mavlink`, or `auto` (default: auto)
- `MSP_RECONNECT_IDLE_S` - Reconnection timeout (default: 3.0s)
- `MSP_RECONNECT_BACKOFF_S` - Reconnection delay (default: 0.5s)

**Example:**
```bash
export MSP_PORT=/dev/serial0
export MSP_BAUD=115200
export MSP_PROTOCOL=msp
python OpenCockpit/main.py
```

### Method 4: Via Configuration File

Create or edit `OpenCockpit/config.json`:

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
    "port": "/dev/ttyACM0",
    "baudrate": 115200,
    "protocol": "auto"
  }
}
```

Then run:
```bash
python OpenCockpit/main.py
```

### Method 5: As System Service (Production)

Create `/etc/systemd/system/opencockpit.service`:

```ini
[Unit]
Description=OpenCockpit FPV Display System
After=network.target

[Service]
Type=simple
User=pi
WorkingDirectory=/home/pi/Desktop/open_clock_pit
Environment="PATH=/home/pi/Desktop/open_clock_pit/venv/bin"
ExecStart=/home/pi/Desktop/open_clock_pit/venv/bin/python \
          /home/pi/Desktop/open_clock_pit/OpenCockpit/main.py
Restart=on-failure
RestartSec=5
StandardOutput=journal
StandardError=journal

[Install]
WantedBy=multi-user.target
```

Enable and start the service:

```bash
sudo systemctl daemon-reload
sudo systemctl enable opencockpit
sudo systemctl start opencockpit
```

Check status:
```bash
sudo systemctl status opencockpit
```

View logs:
```bash
sudo journalctl -u opencockpit -f
```

## Web UI Access

Once running, access the web interface:

```
http://<raspberry-pi-ip>:5000
```

### Available API Endpoints

- **`GET /api/telemetry`** - Current telemetry data (JSON)
  ```bash
  curl http://localhost:5000/api/telemetry | jq
  ```

- **`GET /api/state`** - System state (JSON)
  ```bash
  curl http://localhost:5000/api/state | jq
  ```

- **`GET /api/display/<Display_ID>.png`** - Display screenshot
  ```bash
  curl http://localhost:5000/api/display/Display_1.png > hud.png
  ```

- **`POST /api/config`** - Update configuration

## Troubleshooting

### Issue: "No module named 'board'" or similar

**Solution:** Install dependencies:
```bash
source venv/bin/activate
pip install -r requirements.txt
```

### Issue: "Permission denied" on GPIO

**Solution:** Run with sudo or add user to gpio group:
```bash
sudo usermod -aG gpio pi
# Log out and back in
```

Or run directly:
```bash
sudo python OpenCockpit/main.py
```

### Issue: Serial Port Not Found

Check available ports:
```bash
ls /dev/ttyACM* /dev/serial*
```

List by connection ID (more stable):
```bash
ls /dev/serial/by-id/
```

Use the stable path:
```bash
export MSP_PORT=/dev/serial/by-id/usb-YOUR_DEVICE_ID
python OpenCockpit/main.py
```

### Issue: Displays Not Showing

1. Verify SPI is enabled:
   ```bash
   lsmod | grep spi
   ```

2. Check GPIO conflicts:
   ```bash
   lsof /dev/spidev0.0
   ```

3. Verify pin configuration in `config.json`

4. Check power supply (displays need stable 3.3V)

### Issue: Flickering Display

1. Reduce framerate in `config.json`
2. Lower SPI speed: change `52000000` to `26000000` in main.py
3. Check for GPIO conflicts

### Issue: No Telemetry Data

1. Check flight controller is connected:
   ```bash
   screen /dev/ttyACM0 115200
   ```

2. Verify baud rate matches FC setting (usually 115200)

3. Check protocol is supported (MSP or MAVLink)

4. View logs:
   ```bash
   python OpenCockpit/main.py 2>&1 | head -50
   ```

## Stopping the Application

### If Running in Terminal
Press `Ctrl+C` to gracefully shutdown.

### If Running as Service
```bash
sudo systemctl stop opencockpit
```

### Kill All Instances
```bash
pkill -f "python OpenCockpit/main.py"
```

## Performance Tips

- **Enable Hardware Acceleration** - Check if GPU acceleration is available
- **Reduce Framerate** - Lower FPS in config.json if CPU-bound
- **Use Stable Serial Port** - Prefer `/dev/serial/by-id/*` over `/dev/ttyACM0`
- **Monitor Resource Usage**:
  ```bash
  top -p $(pgrep -f "main.py")
  ```

## Next Steps

1. **Configure Displays** - Adjust `SELECTED_DISPLAYS` in `config.json`
2. **Calibrate Home Position** - Set via web UI
3. **Fine-tune Performance** - Adjust FPS and SPI speed
4. **Set Up Auto-start** - Use system service for production deployment
5. **Monitor Telemetry** - Use web UI or API endpoints

## Getting Help

- **Documentation** - See [docs/INDEX.md](docs/INDEX.md)
- **Architecture** - See [docs/01-ARCHITECTURE.md](docs/01-ARCHITECTURE.md)
- **Hardware Setup** - See [docs/03-HARDWARE.md](docs/03-HARDWARE.md)
- **Configuration** - See [docs/04-CONFIGURATION.md](docs/04-CONFIGURATION.md)

---

**Version:** 1.0  
**Last Updated:** 2026-05-28
