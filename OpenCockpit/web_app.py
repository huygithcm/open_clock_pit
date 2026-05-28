"""Flask web UI for OpenCockpit — live telemetry and runtime config."""
import glob
import io
import json
import os
import subprocess
import sys
import threading
import time

from flask import Flask, Response, jsonify, render_template, request

_BASE = os.path.dirname(os.path.abspath(__file__))
_CONFIG_PATH = os.path.join(_BASE, "config.json")

# Injected by main.py
_msp_module = None
_display_state = {
    "selected": {},          # disp_id -> module key
    "running": [],           # list of disp_id that started successfully
    "available_modules": [], # known module keys
}

app = Flask(__name__, static_folder="static", template_folder="templates")


# ---------------- public API used by main.py ----------------

def init(msp_module, selected, available_modules):
    """Called from main.py before start()."""
    global _msp_module
    _msp_module = msp_module
    _display_state["selected"] = dict(selected)
    _display_state["available_modules"] = list(available_modules)


def mark_running(disp_id):
    if disp_id not in _display_state["running"]:
        _display_state["running"].append(disp_id)


# Live references to the display modules (so we can snapshot their pygame surfaces).
_display_modules = {}     # slot_id -> module
_display_lock = threading.Lock()


def register_display(slot, module):
    with _display_lock:
        _display_modules[slot] = module


def start(host="0.0.0.0", port=5000):
    t = threading.Thread(
        target=lambda: app.run(
            host=host, port=port, debug=False, use_reloader=False, threaded=True
        ),
        daemon=True,
    )
    t.start()
    print(f"[WEB] UI listening on http://0.0.0.0:{port}")
1

# ---------------- routes ----------------

@app.route("/")
def index():
    return render_template("index.html")


@app.route("/api/telemetry")
def telemetry():
    if _msp_module is None:
        return jsonify({"ok": False, "data": {}})
    with _msp_module.data_lock:
        snapshot = dict(_msp_module.data)
    return jsonify({"ok": True, "data": snapshot})


@app.route("/api/state")
def state():
    # Include each running module's pixel size + key so the UI can lay out previews.
    sizes = {}
    with _display_lock:
        for slot, mod in _display_modules.items():
            cfg = getattr(mod, "DISPLAY_CONFIG", {}) or {}
            sizes[slot] = {
                "module": _display_state["selected"].get(slot),
                "width": cfg.get("width"),
                "height": cfg.get("height"),
            }
    return jsonify({
        "selected": _display_state["selected"],
        "running": _display_state["running"],
        "available_modules": _display_state["available_modules"],
        "sizes": sizes,
    })


@app.route("/api/display/<slot>.png")
def display_png(slot):
    """Snapshot the pygame surface of a running display as PNG."""
    with _display_lock:
        mod = _display_modules.get(slot)
    if mod is None or not hasattr(mod, "screen") or mod.screen is None:
        return Response(status=404)

    # If module has composed surfaces (MFD/INFO), reuse the already-blitted .screen.
    surface = mod.screen
    try:
        import pygame
        # Convert surface to raw RGB bytes -> PIL -> PNG
        w, h = surface.get_size()
        raw = pygame.image.tostring(surface, "RGB")
        from PIL import Image
        img = Image.frombytes("RGB", (w, h), raw)
        buf = io.BytesIO()
        img.save(buf, format="PNG", optimize=False, compress_level=1)
        return Response(buf.getvalue(), mimetype="image/png",
                        headers={"Cache-Control": "no-store"})
    except Exception as e:
        return Response(f"err: {e}", status=500)


@app.route("/api/config", methods=["GET"])
def get_config():
    if os.path.exists(_CONFIG_PATH):
        try:
            with open(_CONFIG_PATH) as f:
                return jsonify(json.load(f))
        except Exception as e:
            return jsonify({"error": str(e)}), 500
    return jsonify({"displays": _display_state["selected"]})


@app.route("/api/config", methods=["POST"])
def set_config():
    payload = request.get_json(silent=True) or {}
    displays = payload.get("displays", {})
    try:
        merged = _merge_cfg({"displays": displays})
    except Exception as e:
        return jsonify({"ok": False, "error": str(e)}), 500
    return jsonify({"ok": True, "saved": merged, "note": "Restart main.py to apply"})


def load_config_from_disk():
    """Helper for main.py to read the saved config (if any)."""
    return _load_full_config()


# ---------------- shared config helpers ----------------

_DEFAULT_FC = {
    "port": "/dev/serial0",
    "baudrate": 115200,
    "protocol": "auto",          # auto | msp | mavlink
}

def _load_full_config():
    if not os.path.exists(_CONFIG_PATH):
        return None
    try:
        with open(_CONFIG_PATH) as f:
            return json.load(f)
    except Exception:
        return None


def _save_full_config(cfg):
    with open(_CONFIG_PATH, "w") as f:
        json.dump(cfg, f, indent=2, sort_keys=True)


def _merge_cfg(updates):
    """Merge into the current config.json file."""
    cur = _load_full_config() or {}
    for k, v in updates.items():
        if isinstance(v, dict) and isinstance(cur.get(k), dict):
            cur[k].update(v)
        else:
            cur[k] = v
    _save_full_config(cur)
    return cur


# ---------------- port scanning ----------------

def _scan_ports():
    """Discover candidate serial ports on the system."""
    found = []
    seen = set()

    def add(path, kind):
        if not path or path in seen:
            return
        seen.add(path)
        info = {"path": path, "kind": kind, "exists": os.path.exists(path)}
        try:
            info["real"] = os.path.realpath(path)
        except Exception:
            info["real"] = path
        # Stable id usually contains vendor/product names
        if "by-id" in path:
            info["label"] = os.path.basename(path)
        else:
            info["label"] = path
        found.append(info)

    # Stable USB CDC paths (best)
    for p in sorted(glob.glob("/dev/serial/by-id/*")):
        add(p, "usb-stable")

    # Generic USB CDC / FTDI
    for p in sorted(glob.glob("/dev/ttyACM*")):
        add(p, "usb-cdc")
    for p in sorted(glob.glob("/dev/ttyUSB*")):
        add(p, "usb-ftdi")

    # On-board UART
    for p in ("/dev/serial0", "/dev/serial1", "/dev/ttyAMA0", "/dev/ttyS0"):
        if os.path.exists(p):
            add(p, "uart")

    return found


# ---------------- new API routes ----------------

@app.route("/api/ports")
def api_ports():
    return jsonify({"ports": _scan_ports()})


@app.route("/api/port/test", methods=["POST"])
def api_port_test():
    """Open a port briefly and report whether data is flowing."""
    payload = request.get_json(silent=True) or {}
    port = payload.get("port")
    try:
        baud = int(payload.get("baudrate") or 115200)
    except (TypeError, ValueError):
        baud = 115200

    if not port:
        return jsonify({"ok": False, "error": "port required"}), 400
    if not os.path.exists(port):
        return jsonify({"ok": False, "error": f"port not found: {port}"}), 200

    try:
        import serial as pyserial
    except Exception as e:
        return jsonify({"ok": False, "error": f"pyserial missing: {e}"}), 200

    try:
        s = pyserial.Serial(port, baud, timeout=0.4)
        time.sleep(0.4)
        data = s.read(400)
        s.close()
    except Exception as e:
        return jsonify({"ok": False, "error": str(e)}), 200

    head = data[:48]
    # Sniff protocol
    proto = "unknown"
    if data.count(b"\xFD") >= 2:
        proto = "mavlink2"
    elif data.count(b"\xFE") >= 2:
        proto = "mavlink1"
    elif data.count(b"$M") >= 1:
        proto = "msp"

    return jsonify({
        "ok": True,
        "port": port,
        "baudrate": baud,
        "bytes": len(data),
        "head_hex": head.hex(),
        "protocol_hint": proto,
    })


@app.route("/api/fc-config", methods=["GET"])
def api_fc_get():
    cfg = _load_full_config() or {}
    fc = dict(_DEFAULT_FC)
    fc.update(cfg.get("fc") or {})
    return jsonify(fc)


def _schedule_restart(delay=1.0):
    """Spawn a fresh copy of the current main script then exit; the new
    process will auto-kill us via its own startup hook."""
    def _runner():
        try:
            time.sleep(delay)
            cwd = os.path.dirname(os.path.abspath(__file__))
            cmd = [sys.executable, "-u"] + sys.argv
            log_path = os.path.join(cwd, "restart.log")
            log_f = open(log_path, "ab", buffering=0)
            log_f.write(b"\n=== restart by web at " + time.strftime("%F %T").encode() + b" ===\n")
            subprocess.Popen(
                cmd,
                cwd=cwd,
                start_new_session=True,
                stdin=subprocess.DEVNULL,
                stdout=log_f,
                stderr=subprocess.STDOUT,
                close_fds=True,
            )
        except Exception as e:
            print(f"[WEB] restart failed: {e}")

    threading.Thread(target=_runner, daemon=True).start()


@app.route("/api/restart", methods=["POST"])
def api_restart():
    _schedule_restart()
    return jsonify({"ok": True, "restarting": True, "delay_ms": 1000})


@app.route("/api/fc-config", methods=["POST"])
def api_fc_set():
    payload = request.get_json(silent=True) or {}
    fc = dict(_DEFAULT_FC)
    fc.update(payload)
    # Coerce types
    try:
        fc["baudrate"] = int(fc.get("baudrate") or 115200)
    except (TypeError, ValueError):
        fc["baudrate"] = 115200
    if fc.get("protocol") not in {"auto", "msp", "mavlink"}:
        fc["protocol"] = "auto"

    try:
        merged = _merge_cfg({"fc": fc})
    except Exception as e:
        return jsonify({"ok": False, "error": str(e)}), 500

    return jsonify({"ok": True, "fc": merged["fc"], "note": "Restart main.py to apply"})
