# ⚙️ Configuration & Hardcoding Analysis

## Hardcoded Values Found

### 1️⃣ GPIO Pin Mapping (CRITICAL)

**Status:** ⚠️ **Hardcoded in multiple files**

**Locations:**
- `main.py`: lines 115-118 (DISPLAY_HARDWARE_MAP)
- `HUD_pi_085.py`: lines 504-506 (test code)
- `HUD_pi_114.py`: lines 505-507 (test code)
- `MAP_pi_096.py`: lines 284-286 (test code)
- `MFD_pi_096.py`: lines 640-642 (test code)
- `INFO_pi_096.py`: lines 545-547 (test code)

**Current Code:**
```python
DISPLAY_HARDWARE_MAP = {
    "Display_1": {"cs": board.D5,  "dc": board.D25, "rst": board.D22},
    "Display_2": {"cs": board.D6,  "dc": board.D24, "rst": board.D27},
    "Display_3": {"cs": board.D13, "dc": board.D23, "rst": board.D17},
    "Display_4": {"cs": board.D19, "dc": board.D26, "rst": board.D16}
}
```

**Impact:** ❌ Cannot use different GPIO pins without editing code

**Recommendation:** Move to `config.json`

---

### 2️⃣ Display Modules Selection

**Status:** ⚠️ **Partially configurable**

**Locations:**
- `main.py`: lines 94-99 (SELECTED_DISPLAYS hardcoded)
- `main.py`: lines 103-110 (reads from config.json)

**Current Code:**
```python
SELECTED_DISPLAYS = {
    "Display_1": "HUD_0.85",
    "Display_2": "MAP_0.96",
    "Display_3": "MFD_0.96",
    "Display_4": "INFO_0.96",
}
# Override from config.json if present
try:
    _saved = web_app.load_config_from_disk()
    if _saved and isinstance(_saved.get("displays"), dict):
        SELECTED_DISPLAYS = _saved["displays"]
except Exception:
    pass
```

**Impact:** ✅ Can be configured via web UI, but needs web_app.py running

**Recommendation:** Load from config.json at startup (no web UI needed)

---

### 3️⃣ Framerate Settings

**Status:** ⚠️ **Hardcoded constants**

**Location:** `main.py`, lines 131-132

**Current Code:**
```python
HIGH_FPS = 30   # HUD modules
LOW_FPS = 15    # MAP/MFD/INFO
```

**Impact:** ❌ Cannot adjust refresh rates without editing code

**Recommendation:** Move to config.json or environment variables

---

### 4️⃣ Serial Port & Baud Rate

**Status:** ✅ **Good - multiple fallbacks**

**Current Priority:**
1. Environment: `MSP_PORT`, `MSP_BAUD`
2. config.json: `fc.port`, `fc.baudrate`
3. Auto-detection (glob patterns)
4. Defaults: `/dev/serial0`, `115200`

**Current Code:**
```python
PORT = _pick_default_port()  # Smart detection

BAUDRATE = int(_FC_CFG.get("baudrate") or 
               os.environ.get("MSP_BAUD") or 115200)
```

**Impact:** ✅ Well-handled, flexible

---

### 5️⃣ Protocol Selection (MSP vs MAVLink)

**Status:** ✅ **Good - auto-detection**

**Priority:**
1. Environment: `MSP_PROTOCOL`
2. config.json: `fc.protocol`
3. Auto-detection (heartbeat check)
4. Fallback: MSP

**Current Code:**
```python
_cfg_proto = (_FC_CFG.get("protocol") or "").lower()
if _cfg_proto in {"msp", "mavlink"} and "MSP_PROTOCOL" not in os.environ:
    os.environ["MSP_PROTOCOL"] = _cfg_proto
```

**Impact:** ✅ Flexible and auto-detects

---

### 6️⃣ Web UI Binding

**Status:** ⚠️ **Hardcoded address & port**

**Location:** `main.py`, line 311

**Current Code:**
```python
web_app.start(host="0.0.0.0", port=5000)
```

**Impact:** ❌ Always binds to 0.0.0.0:5000, cannot change port

**Recommendation:** Make configurable via environment or config.json

---

### 7️⃣ SPI Baudrate

**Status:** ⚠️ **Hardcoded**

**Location:** `main.py`, line 205

**Current Code:**
```python
display_kwargs = {
    ...
    "baudrate": 52000000  # 52 MHz
}
```

**Impact:** ❌ Cannot slow down SPI if displays are unreliable

**Recommendation:** Make configurable in config.json

---

### 8️⃣ Reconnection Timeouts

**Status:** ✅ **Environment variables available**

**Location:** `MSP_Read_pi.py`, lines 262-263

**Current Code:**
```python
RECONNECT_IDLE_S = float(os.environ.get("MSP_RECONNECT_IDLE_S", "3.0"))
RECONNECT_BACKOFF_S = float(os.environ.get("MSP_RECONNECT_BACKOFF_S", "0.5"))
```

**Impact:** ✅ Already configurable via environment

---

### 9️⃣ Font Path

**Status:** ✅ **Relative to module**

**Current Code:**
```python
_ASSET_DIR = os.path.dirname(__file__)
_FONT_BOLD = os.path.join(_ASSET_DIR, "ViperDisplay-Bold.ttf")
```

**Impact:** ✅ No hardcoding (relative path)

---

## Configuration File Format

### Recommended: config.json Schema

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

## Action Items to De-hardcode

### HIGH PRIORITY 🔴

#### 1. GPIO Pin Mapping
```python
# BEFORE (main.py)
DISPLAY_HARDWARE_MAP = {
    "Display_1": {"cs": board.D5, ...}
}

# AFTER
def load_gpio_pins(config):
    mapping = config.get("hardware", {}).get("gpio_pins", {})
    return {
        disp_id: {
            "cs": board.__dict__[f"D{pins['cs']}"],
            "dc": board.__dict__[f"D{pins['dc']}"],
            "rst": board.__dict__[f"D{pins['rst']}"]
        }
        for disp_id, pins in mapping.items()
    }
```

#### 2. Framerate Settings
```python
# BEFORE (main.py)
HIGH_FPS = 30
LOW_FPS = 15

# AFTER
config = load_config()
HIGH_FPS = config.get("hardware", {}).get("fps", {}).get("hud", 30)
LOW_FPS = config.get("hardware", {}).get("fps", {}).get("map_mfd_info", 15)
```

### MEDIUM PRIORITY 🟡

#### 3. Web UI Binding
```python
# BEFORE (main.py)
web_app.start(host="0.0.0.0", port=5000)

# AFTER
config = load_config()
web_config = config.get("web_ui", {})
host = web_config.get("host", "0.0.0.0")
port = web_config.get("port", 5000)
web_app.start(host=host, port=port)
```

#### 4. SPI Baudrate
```python
# BEFORE (main.py)
"baudrate": 52000000

# AFTER
config = load_config()
spi_baud = config.get("hardware", {}).get("spi_baudrate", 52000000)
display_kwargs["baudrate"] = spi_baud
```

### LOW PRIORITY 🟢

#### 5. Remove Test Code from Modules
- `HUD_pi_085.py`, `HUD_pi_114.py`, etc.
- Lines with board.D5/D6/D13/D19 are test code
- These should not hardcode GPIO (main.py handles it)

---

## Environment Variables (Already Supported)

```bash
# MSP Configuration
export MSP_PORT=/dev/ttyACM0
export MSP_BAUD=115200
export MSP_PROTOCOL=msp

# MSP Reconnection
export MSP_RECONNECT_IDLE_S=3.0
export MSP_RECONNECT_BACKOFF_S=0.5
```

**Note:** These are respected but incomplete. GPIO pins still hardcoded in code.

---

## Recommended Implementation Path

### Step 1: Create config.json loader
```python
# config_loader.py
import json
import os

def load_config():
    """Load config.json from OpenCockpit directory."""
    path = os.path.join(os.path.dirname(__file__), "config.json")
    if os.path.exists(path):
        with open(path) as f:
            return json.load(f)
    return {}
```

### Step 2: Update main.py
```python
# main.py
from config_loader import load_config

config = load_config()

# Replace hardcoded values
SELECTED_DISPLAYS = config.get("displays", {...default...})
DISPLAY_HARDWARE_MAP = config.get("hardware", {}).get("gpio_pins", {...default...})
HIGH_FPS = config.get("hardware", {}).get("fps", {}).get("hud", 30)
```

### Step 3: Create default config.json
```json
{
  "displays": { ... },
  "hardware": { ... },
  "fc": { ... }
}
```

### Step 4: Document in CLAUDE.md
- User can modify config.json without touching Python code
- Web UI generates config.json on changes
- CLI support via environment variables

---

## Current State Summary

| Component | Status | Notes |
|-----------|--------|-------|
| Serial Port | ✅ Configurable | Env vars, config.json, auto-detect |
| Baud Rate | ✅ Configurable | Env vars, config.json, default 115200 |
| Protocol | ✅ Configurable | Env vars, config.json, auto-detect |
| GPIO Pins | ❌ Hardcoded | **NEEDS FIX** |
| Display Modules | ⚠️ Partial | Works via web UI only |
| Framerate | ❌ Hardcoded | **NEEDS FIX** |
| Web UI Port | ❌ Hardcoded | **NEEDS FIX** |
| SPI Baudrate | ❌ Hardcoded | **NEEDS FIX** |
| Reconnect Timeouts | ✅ Configurable | Env vars only |

---

See also:
- [ARCHITECTURE.md](01-ARCHITECTURE.md) - System design
