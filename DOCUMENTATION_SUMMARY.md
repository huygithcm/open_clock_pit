# 📋 Documentation Package Summary

## ✅ Created Files

### Main Documentation (Modular)

```
docs/
├── INDEX.md                 ← Start here! Documentation home
├── 01-ARCHITECTURE.md       ← System design & components
├── 02-DATAFLOW.md          ← Thread flow & timing analysis
├── 03-HARDWARE.md          ← GPIO pinouts & specs
└── 04-CONFIGURATION.md     ← Settings & hardcoding issues
```

### Project Root Files

```
├── CLAUDE.md               ← Project guide & quick reference
├── DATAFLOW.md             ← Original format (full reference)
└── README.md               ← Project overview (existing)
```

---

## 🔍 Hardcoding Issues Found

### Summary Table

| Issue | Location | Severity | Status | Fix |
|-------|----------|----------|--------|-----|
| GPIO pins | main.py:115-118 | 🔴 CRITICAL | Hardcoded | Move to config.json |
| Display selection | main.py:94-99 | 🟡 MEDIUM | Hardcoded | Needs startup load |
| Framerate (FPS) | main.py:131-132 | 🟡 MEDIUM | Hardcoded | Move to config.json |
| Web UI port | main.py:311 | 🟡 MEDIUM | Hardcoded | Make configurable |
| SPI baudrate | main.py:205 | 🟡 MEDIUM | Hardcoded | Move to config.json |
| Serial port | MSP_Read_pi.py | ✅ GOOD | Auto-detect | No change needed |
| Baud rate | MSP_Read_pi.py | ✅ GOOD | Multi-source | No change needed |
| Protocol | MSP_Read_pi.py | ✅ GOOD | Auto-detect | No change needed |

### Detailed Issues

#### 🔴 1. GPIO Pin Mapping (CRITICAL)

**Problem:** GPIO pins hardcoded in 3 places:
- `main.py` line 115-118: DISPLAY_HARDWARE_MAP
- `HUD_pi_085.py` line 504-506: Test code
- Other display modules: Similar test code

**Impact:** Cannot use different GPIO configuration without editing Python

**Current Code:**
```python
DISPLAY_HARDWARE_MAP = {
    "Display_1": {"cs": board.D5, "dc": board.D25, "rst": board.D22},
    "Display_2": {"cs": board.D6, "dc": board.D24, "rst": board.D27},
    "Display_3": {"cs": board.D13, "dc": board.D23, "rst": board.D17},
    "Display_4": {"cs": board.D19, "dc": board.D26, "rst": board.D16}
}
```

**Recommendation:** Load from config.json at startup
```python
# Proposed fix
config = load_config()
gpio_config = config.get("hardware", {}).get("gpio_pins", {...defaults...})
# Convert to board.Dx format
```

---

#### 🟡 2. Display Module Selection (MEDIUM)

**Problem:** SELECTED_DISPLAYS hardcoded, config.json only checked if web_app runs

**Current Code:**
```python
SELECTED_DISPLAYS = {
    "Display_1": "HUD_0.85",
    "Display_2": "MAP_0.96",
    "Display_3": "MFD_0.96",
    "Display_4": "INFO_0.96",
}
# Override from config only if web UI ran
try:
    _saved = web_app.load_config_from_disk()
    if _saved and _saved.get("displays"):
        SELECTED_DISPLAYS = _saved["displays"]
except:
    pass
```

**Impact:** Must run web UI first to use custom config, or edit code

**Recommendation:** Always load config.json at startup
```python
# Proposed fix
config = load_config_at_startup()
SELECTED_DISPLAYS = config.get("displays", {...defaults...})
```

---

#### 🟡 3. Framerate Settings (MEDIUM)

**Problem:** HIGH_FPS & LOW_FPS hardcoded, cannot adjust without editing

**Current Code:**
```python
HIGH_FPS = 30   # HUD
LOW_FPS = 15    # MAP/MFD/INFO
```

**Use in Code:**
```python
if "HUD" in mod_key:
    fps = HIGH_FPS
else:
    fps = LOW_FPS
```

**Recommendation:** Load from config.json
```json
{
  "hardware": {
    "fps": {
      "hud": 30,
      "map_mfd_info": 15
    }
  }
}
```

---

#### 🟡 4. Web UI Port Binding (MEDIUM)

**Problem:** Hardcoded to 0.0.0.0:5000

**Current Code:**
```python
web_app.start(host="0.0.0.0", port=5000)
```

**Impact:** Cannot run multiple instances or change port

**Recommendation:**
```python
config = load_config()
web_config = config.get("web_ui", {})
host = web_config.get("host", "0.0.0.0")
port = web_config.get("port", 5000)
web_app.start(host=host, port=port)
```

---

#### 🟡 5. SPI Baudrate (MEDIUM)

**Problem:** Hardcoded to 52 MHz, cannot slow down if displays unstable

**Current Code:**
```python
display_kwargs = {
    ...
    "baudrate": 52000000  # 52 MHz - HARDCODED
}
```

**Recommendation:**
```json
{
  "hardware": {
    "spi_baudrate": 52000000
  }
}
```

---

#### ✅ 6. Serial Port (GOOD - No change needed)

**Current Code:**
```python
# Multiple fallback sources (priority order):
1. Environment: MSP_PORT
2. config.json: fc.port
3. Auto-detect: /dev/serial/by-id/*, /dev/ttyACM0, /dev/serial0

PORT = _pick_default_port()  # Smart detection
```

**Status:** ✅ Well-handled

---

#### ✅ 7. Baud Rate (GOOD - No change needed)

**Current Code:**
```python
BAUDRATE = int(_FC_CFG.get("baudrate") or 
               os.environ.get("MSP_BAUD") or 115200)
```

**Status:** ✅ Well-handled (env var, config.json, default)

---

#### ✅ 8. Protocol Selection (GOOD - No change needed)

**Current Code:**
```python
if forced_protocol in {"msp", "mavlink"}:
    protocol = forced_protocol
else:
    if _should_try_mavlink_first(PORT) and _mavlink_heartbeat_present(PORT, BAUDRATE):
        protocol = "mavlink"
    else:
        protocol = _detect_protocol(PORT, BAUDRATE)
```

**Status:** ✅ Well-handled (auto-detection + env var)

---

## 📊 Config.json Schema (Recommended)

```json
{
  "version": "1.0",
  
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
    },
    "spi_baudrate": 52000000,
    "fps": {
      "hud": 30,
      "map_mfd_info": 15
    }
  },
  
  "fc": {
    "port": "/dev/serial0",
    "baudrate": 115200,
    "protocol": "auto"
  },
  
  "web_ui": {
    "host": "0.0.0.0",
    "port": 5000
  },
  
  "msp": {
    "reconnect_idle_s": 3.0,
    "reconnect_backoff_s": 0.5
  }
}
```

---

## 🎯 Priority Action Items

### Priority 1: GPIO Pin Mapping 🔴

**Why:** Cannot use different hardware without editing code

**File:** `main.py` line 115-118

**Action:**
```python
def load_gpio_config(config):
    pins_config = config.get("hardware", {}).get("gpio_pins", {})
    result = {}
    for disp_id, pins in pins_config.items():
        result[disp_id] = {
            "cs": board.__dict__[f"D{pins['cs']}"],
            "dc": board.__dict__[f"D{pins['dc']}"],
            "rst": board.__dict__[f"D{pins['rst']}"]
        }
    return result

DISPLAY_HARDWARE_MAP = load_gpio_config(config)
```

---

### Priority 2: Display Selection & Framerate 🟡

**Why:** Need config.json to change without editing code

**Files:** `main.py` lines 94-99, 131-132

**Action:**
```python
config = load_config()
SELECTED_DISPLAYS = config.get("displays", {...defaults...})
fps_config = config.get("hardware", {}).get("fps", {})
HIGH_FPS = fps_config.get("hud", 30)
LOW_FPS = fps_config.get("map_mfd_info", 15)
```

---

### Priority 3: Web UI Port & SPI Baudrate 🟡

**Why:** Need flexibility for edge cases

**Files:** `main.py` lines 205, 311

**Action:**
```python
# SPI Baudrate
config = load_config()
spi_baud = config.get("hardware", {}).get("spi_baudrate", 52000000)
display_kwargs["baudrate"] = spi_baud

# Web UI
web_config = config.get("web_ui", {})
web_app.start(host=web_config.get("host", "0.0.0.0"),
              port=web_config.get("port", 5000))
```

---

## 📈 Recommended Implementation Path

### Phase 1: Add config loader (1 file)
- Create `config_loader.py`
- Handles config.json loading
- Fallback to hardcoded defaults

### Phase 2: Update main.py (1 file)
- Import config loader
- Replace hardcoded values with config
- Keep hardcoded defaults for backward compatibility

### Phase 3: Create config.json template (1 file)
- User can copy & modify
- Web UI can generate automatically

### Phase 4: Clean up display modules (5 files)
- Remove test GPIO code from HUD/MAP/MFD/INFO
- GPIO pins handled only in main.py

### Phase 5: Documentation (already done!)
- ✅ Architecture guide
- ✅ Configuration guide
- ✅ Hardcoding analysis

---

## 📚 How to Use This Documentation

### I want to...

**Understand the system**
→ Read [docs/01-ARCHITECTURE.md](docs/01-ARCHITECTURE.md)

**Debug telemetry flow**
→ Read [docs/02-DATAFLOW.md](docs/02-DATAFLOW.md)

**Set up hardware**
→ Read [docs/03-HARDWARE.md](docs/03-HARDWARE.md)

**Configure settings**
→ Read [docs/04-CONFIGURATION.md](docs/04-CONFIGURATION.md)

**Quick reference**
→ Read [CLAUDE.md](CLAUDE.md)

**Full details (original format)**
→ Read [DATAFLOW.md](DATAFLOW.md)

---

## ✨ What's Different Now

### Before
- ❌ Single large DATAFLOW.md
- ❌ No architecture guide
- ❌ No hardware guide
- ❌ Hardcoding issues not documented

### After  
- ✅ Modular documentation (5 files)
- ✅ Architecture guide (design patterns)
- ✅ Hardware guide (GPIO specs)
- ✅ Configuration guide (hardcoding analysis + fixes)
- ✅ CLAUDE.md (project guide)
- ✅ All hardcoding issues cataloged
- ✅ Priority fixes identified

---

## 📞 Next Steps

1. **Review documentation:** Start with [docs/INDEX.md](docs/INDEX.md)
2. **Read configuration issues:** [docs/04-CONFIGURATION.md](docs/04-CONFIGURATION.md)
3. **Decide on implementation:** Pick priority level & timeline
4. **Create config.json loader:** Small, high-impact change
5. **Update main.py:** Replace hardcoded values
6. **Test:** Verify backward compatibility

---

**Status:** Documentation Complete ✅  
**Hardcoding Analysis:** Complete ✅  
**Action Items:** Identified (see Priority 1-3 above)  
**Ready for:** Implementation or code review

See [docs/INDEX.md](docs/INDEX.md) for complete documentation.
