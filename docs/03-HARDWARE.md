# 🖥️ Hardware Specifications

## Display Controllers

### Overview

OpenCockpit uses 4× ST7735S color LCD displays on a shared SPI bus.

| Parameter | Value |
|-----------|-------|
| Total Displays | 4 |
| Driver Chip | ST7735S |
| Interface | SPI |
| SPI Baudrate | 52 MHz |
| Color Depth | RGB565 (16-bit) |
| Refresh Rate | 30 Hz (HUD) / 15 Hz (others) |

---

## Individual Display Configuration

### Display 1: HUD Attitude Indicator

```
╔═══════════════════════════════════╗
║ Display_1 (HUD_0.85)              ║
╠═══════════════════════════════════╣
║ Size:          1.14"              ║
║ Resolution:    128 × 128 pixels   ║
║ Driver:        ST7735S            ║
║ Framerate:     30 Hz (HIGH_FPS)   ║
║ Content:       Attitude indicator ║
║                (roll, pitch, yaw) ║
╠═══════════════════════════════════╣
║ GPIO Pins:                        ║
║   CS  (Chip Select):  GPIO 5      ║
║   DC  (Data/Cmd):     GPIO 25     ║
║   RST (Reset):        GPIO 22     ║
╚═══════════════════════════════════╝
```

**Rendering:**
- Pygame Surface: 128×128 RGB888
- Color Conv: RGB888 → RGB565
- Data Size: 128 × 128 × 2 = 32 KB
- SPI Time @ 52MHz: ~5 ms

---

### Display 2: MAP

```
╔═════════════════════════════════════╗
║ Display_2 (MAP_0.96)                ║
╠═════════════════════════════════════╣
║ Size:          0.96"                ║
║ Resolution:    160 × 80 pixels     ║
║ Driver:        ST7735S              ║
║ Framerate:     15 Hz (LOW_FPS)      ║
║ Content:       GPS map with heading ║
╠═════════════════════════════════════╣
║ GPIO Pins:                          ║
║   CS  (Chip Select):  GPIO 6        ║
║   DC  (Data/Cmd):     GPIO 24       ║
║   RST (Reset):        GPIO 27       ║
╚═════════════════════════════════════╝
```

**Rendering:**
- Pygame Surface: 160×80 RGB888
- Data Size: 160 × 80 × 2 = 25.6 KB
- SPI Time @ 52MHz: ~4 ms

---

### Display 3: MFD (Multi-Function Display)

```
╔═════════════════════════════════════╗
║ Display_3 (MFD_0.96)                ║
╠═════════════════════════════════════╣
║ Size:          0.96"                ║
║ Resolution:    160 × 80 pixels     ║
║ Driver:        ST7735S              ║
║ Framerate:     15 Hz (LOW_FPS)      ║
║ Content:       Flight instruments   ║
║                (alt scale, speed,   ║
║                 vertical speed,     ║
║                 heading indicator)  ║
╠═════════════════════════════════════╣
║ GPIO Pins:                          ║
║   CS  (Chip Select):  GPIO 13       ║
║   DC  (Data/Cmd):     GPIO 23       ║
║   RST (Reset):        GPIO 17       ║
╚═════════════════════════════════════╝
```

---

### Display 4: INFO (Status & Power)

```
╔═════════════════════════════════════╗
║ Display_4 (INFO_0.96)               ║
╠═════════════════════════════════════╣
║ Size:          0.96"                ║
║ Resolution:    160 × 80 pixels     ║
║ Driver:        ST7735S              ║
║ Framerate:     15 Hz (LOW_FPS)      ║
║ Content:       Power & System info  ║
║                (vbat, current, mah, ║
║                 satellites, rssi)   ║
╠═════════════════════════════════════╣
║ GPIO Pins:                          ║
║   CS  (Chip Select):  GPIO 19       ║
║   DC  (Data/Cmd):     GPIO 26       ║
║   RST (Reset):        GPIO 16       ║
╚═════════════════════════════════════╝
```

---

## SPI Bus Configuration

### Shared SPI Bus

All 4 displays share a single SPI bus:

```
Raspberry Pi
│
├─ GPIO 11 (SCK)  ─────┬─────────┬─────────┬───────────┐
│                      │         │         │           │
├─ GPIO 10 (MOSI) ─────┼─────────┼─────────┼───────────┤
│                      │         │         │           │
├─ GPIO 5 (CS_1)  ─────┤         │         │           │
├─ GPIO 6 (CS_2)  ─────┤         │         │           │
├─ GPIO 13 (CS_3) ─────┤         │         │           │
├─ GPIO 19 (CS_4) ─────┤         │         │           │
│                      │         │         │           │
└─────────────────────┼─────────┼─────────┼───────────┘
                      │         │         │
                  ST7735S   ST7735S   ST7735S   ST7735S
                  (Disp1)   (Disp2)   (Disp3)   (Disp4)
```

### SPI Timing

```
Baudrate:      52 MHz
Clock Period:  19.2 ns
Byte Transfer: 154.4 ns

Frame Size:    32 KB (HUD) or 25.6 KB (others)
Transfer Time: ~5 ms per display
```

### Why Shared SPI?

- **Pros:** Saves GPIO pins, simpler PCB layout
- **Cons:** Displays updated sequentially (not in parallel)
- **Result:** ~20-40 ms total SPI time for all 4 displays

---

## GPIO Pin Assignment

### Complete Pinout

```
Raspberry Pi GPIO
├─ SCK (GPIO 11)     → SPI Clock (all displays)
├─ MOSI (GPIO 10)    → SPI Data  (all displays)
├─ MISO (GPIO 9)     → Unused
│
├─ Display 1 (HUD):
│  ├─ CS (GPIO 5)    → Chip Select
│  ├─ DC (GPIO 25)   → Data/Command
│  └─ RST (GPIO 22)  → Reset
│
├─ Display 2 (MAP):
│  ├─ CS (GPIO 6)    → Chip Select
│  ├─ DC (GPIO 24)   → Data/Command
│  └─ RST (GPIO 27)  → Reset
│
├─ Display 3 (MFD):
│  ├─ CS (GPIO 13)   → Chip Select
│  ├─ DC (GPIO 23)   → Data/Command
│  └─ RST (GPIO 17)  → Reset
│
└─ Display 4 (INFO):
   ├─ CS (GPIO 19)   → Chip Select
   ├─ DC (GPIO 26)   → Data/Command
   └─ RST (GPIO 16)  → Reset

Total GPIO Used: 14 pins (SPI: 2, Displays: 12)
```

### GPIO Configuration in Code

```python
# main.py
DISPLAY_HARDWARE_MAP = {
    "Display_1": {"cs": board.D5,  "dc": board.D25, "rst": board.D22},
    "Display_2": {"cs": board.D6,  "dc": board.D24, "rst": board.D27},
    "Display_3": {"cs": board.D13, "dc": board.D23, "rst": board.D17},
    "Display_4": {"cs": board.D19, "dc": board.D26, "rst": board.D16}
}
```

---

## ST7735S Driver Details

### Communication Protocol

```
SPI Protocol:
├─ Mode:        SPI Mode 0 (CPOL=0, CPHA=0)
├─ Endianness:  MSB first
├─ Baudrate:    Up to 52 MHz (typical)
├─ Commands:    1 byte (via DC=0)
├─ Data:        2-byte RGB565 pixels (via DC=1)
│
GPIO Control:
├─ DC (Data/Command):
│  ├─ DC=0: Next byte is command
│  └─ DC=1: Next bytes are data (pixels)
│
├─ CS (Chip Select):
│  ├─ CS=0: Display is active (selected)
│  └─ CS=1: Display is inactive (deselected)
│
└─ RST (Reset):
   ├─ RST=0: Perform hardware reset (hold 50ms)
   └─ RST=1: Normal operation
```

### ST7735S Initialization Sequence

```python
# From main.py:init_display()

1. RST pulse (cold start)
   └─ RST = 0 (50ms) → RST = 1 (50ms)

2. Send initialization commands (via SPI, DC=0)
   ├─ Sleep Out (0x11)
   ├─ Display Mode Set (0x36)
   ├─ Pixel Format Set (0x3A) → RGB565
   ├─ Display On (0x29)
   └─ ...more commands

3. Ready for pixel data writes
```

---

## Power Consumption

### Per Display

```
ST7735S @ 52MHz @ 3.3V:
├─ Idle:      ~30 mW
├─ Active:    ~50-80 mW
└─ Backlight: ~100 mW (if present)

Total per display: ~50-150 mW
```

### System Total

```
Raspberry Pi 5:     ~5-10 W
4× Displays:        ~0.2-0.6 W
Total System:       ~5-11 W @ 5V USB-C
```

---

## Thermal Considerations

- ST7735S operating temperature: 0°C to 60°C
- Raspberry Pi operating: 0°C to 80°C
- No active cooling needed (passive adequate)
- Avoid direct sunlight (LCD temp > 60°C)

---

## Connection Checklist

### Before Powering On

- [ ] SPI bus connected (SCK, MOSI)
- [ ] All CS pins connected
- [ ] All DC pins connected
- [ ] All RST pins connected
- [ ] Power (3.3V) to all displays
- [ ] Ground connected to all displays
- [ ] No shorts between pins
- [ ] Ribbon cables seated firmly

### After Powering On

- [ ] No visible damage (burn marks, smoke)
- [ ] Displays should show initialization test
- [ ] Web UI accessible at http://rpi:5000
- [ ] Terminal shows "Success: Display_X initialized"

---

## Troubleshooting

### Display Not Showing

**Check:**
1. GPIO pins correct in config.json
2. SPI bus initialized (check logs)
3. ST7735 driver selected correctly
4. RST pulse occurred during init
5. Power supply stable at 3.3V

### Display Shows Garbage

**Check:**
1. SPI baudrate too high (try 26MHz in config)
2. Long wires → add capacitors on power lines
3. Crosstalk → route signals away from power
4. RGB565 conversion wrong (check byte swap)

### Flickering

**Check:**
1. Frame rate too high (reduce to 10Hz)
2. Pygame rendering taking too long
3. SPI contention (check lock timings)

---

## Expansion Options

### Adding Display 5+

1. Allocate new CS pin (GPIO 12, 14, 15, etc.)
2. Allocate new DC pin (GPIO 8, 7, etc.)
3. Allocate new RST pin (GPIO 18, 20, 21, etc.)
4. Update DISPLAY_HARDWARE_MAP in config.json
5. Assign module in SELECTED_DISPLAYS

### Using Different Drivers

- **ST7789**: Larger displays (1.3", 1.54")
- **ILI9341**: Even larger (2.8", 3.2")
- **SSD1351**: OLED 128×128

All compatible with same SPI bus, just need new driver module.

---

See also:
- [ARCHITECTURE.md](01-ARCHITECTURE.md) - System design
- [CONFIGURATION.md](04-CONFIGURATION.md) - Configuration options
