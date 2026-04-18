import serial
import struct
import time
import math
import threading
import os
import glob
from typing import Optional, Tuple

data_lock = threading.Lock()

# ---------------------------- Base Configuration ----------------------------------------

# Set serial port and baudrate
def _pick_default_port() -> str:
    # 1) Allow explicit override
    env_port = os.environ.get("MSP_PORT")
    if env_port:
        return env_port

    # 2) Prefer stable USB CDC path when present (e.g., Flight Controller on USB)
    by_id = sorted(glob.glob("/dev/serial/by-id/*"))
    if by_id:
        return by_id[0]

    # 3) Common USB CDC node
    if os.path.exists("/dev/ttyACM0"):
        return "/dev/ttyACM0"

    # 4) Fallback to primary UART on GPIO
    return "/dev/serial0"


PORT = _pick_default_port()
BAUDRATE = 115200


def _resolve_port(preferred_port: str) -> str:
    """Resolve a usable serial device path.

    USB FCs can enumerate as /dev/ttyACM0, /dev/ttyACM1, ... depending on timing.
    Prefer stable /dev/serial/by-id/* when available.
    """
    if preferred_port and os.path.exists(preferred_port):
        return preferred_port

    # If the preferred port is a USB-like path, fall back to stable by-id.
    if preferred_port.startswith("/dev/ttyACM") or preferred_port.startswith("/dev/serial/by-id/"):
        by_id = sorted(glob.glob("/dev/serial/by-id/*"))
        if by_id:
            return by_id[0]
        acm = sorted(glob.glob("/dev/ttyACM*"))
        if acm:
            return acm[0]

    return preferred_port

# Frequencies of MSP requests and Display output
FAST_HZ = 30.0   # ATTITUDE MSP Frequency
SLOW_HZ = 15.0   # Others MSP Frequency
OUT_HZ  = 30.0   # Print Frequency

FAST_DT = 1.0 / FAST_HZ
SLOW_DT = 1.0 / SLOW_HZ
OUT_DT  = 1.0 / OUT_HZ

# MSP IDs (INAV)
MSP_ATTITUDE   = 108
MSP_ALTITUDE   = 109
MSP_RAW_GPS    = 106
MSP_ANALOG     = 110
MSP_COMP_GPS   = 107
MSP_CURRENT    = 23
MSP_RC = 105


# ---------------------------- MSP Communication Functions ----------------------------------------

# MSP checksum function
def msp_checksum(data):
    c = 0
    for b in data:
        c ^= b
    return c

# MSP request function
def send_msp_request(ser, cmd):
    frame = bytearray(b"$M<")
    frame.append(0)
    frame.append(cmd)
    frame.append(msp_checksum(frame[3:5]))
    ser.write(frame)

# MSP response reading function
def read_msp_response(ser):
    # Header
    if ser.read(1) != b"$":
        return None
    if ser.read(1) != b"M":
        return None
    if ser.read(1) != b">":
        return None

    size_b = ser.read(1)
    if len(size_b) != 1:
        return None
    size = size_b[0]

    cmd_b = ser.read(1)
    if len(cmd_b) != 1:
        return None
    cmd = cmd_b[0]

    payload = ser.read(size)
    if len(payload) != size:
        return None

    chk_b = ser.read(1)
    if len(chk_b) != 1:
        return None
    checksum = chk_b[0]

    if msp_checksum(bytes([size, cmd]) + payload) != checksum:
        return None

    return cmd, payload


# ---------------------------------------- MAVLink Support ----------------------------------------

def _detect_protocol(port: str, baudrate: int) -> str:
    """Best-effort protocol detection.

    - MAVLink2 frames start with 0xFD (often frequent on ArduPilot USB serial)
    - MAVLink1 frames start with 0xFE
    - MSP v1 frames start with '$' (0x24)
    """
    try:
        ser = serial.Serial(_resolve_port(port), baudrate, timeout=0.2)
        try:
            time.sleep(0.15)
            sample = ser.read(400)
        finally:
            ser.close()
    except Exception:
        return "msp"

    if not sample:
        return "msp"

    # Prefer MAVLink when we see repeated 0xFD/0xFE
    if sample.count(b"\xFD") >= 2 or sample.count(b"\xFE") >= 2:
        return "mavlink"
    if b"$" in sample:
        return "msp"
    return "msp"


def _should_try_mavlink_first(port: str) -> bool:
    return (
        port.startswith("/dev/ttyACM")
        or port.startswith("/dev/serial/by-id/")
        or "ArduPilot" in port
        or "ardupilot" in port.lower()
    )


def _mavlink_heartbeat_present(port: str, baudrate: int, timeout_s: float = 1.2) -> bool:
    try:
        from pymavlink import mavutil
    except Exception:
        return False

    mav = None
    try:
        mav = mavutil.mavlink_connection(_resolve_port(port), baud=baudrate, dialect="ardupilotmega")
        hb = mav.wait_heartbeat(timeout=timeout_s)
        return hb is not None
    except Exception:
        return False
    finally:
        try:
            if mav is not None:
                mav.close()
        except Exception:
            pass


def _haversine_m(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    r = 6371000.0
    phi1, phi2 = math.radians(lat1), math.radians(lat2)
    dphi = math.radians(lat2 - lat1)
    dlambda = math.radians(lon2 - lon1)
    a = math.sin(dphi / 2) ** 2 + math.cos(phi1) * math.cos(phi2) * math.sin(dlambda / 2) ** 2
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
    return r * c


def _bearing_deg(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    phi1 = math.radians(lat1)
    phi2 = math.radians(lat2)
    dlambda = math.radians(lon2 - lon1)
    y = math.sin(dlambda) * math.cos(phi2)
    x = math.cos(phi1) * math.sin(phi2) - math.sin(phi1) * math.cos(phi2) * math.cos(dlambda)
    brng = math.degrees(math.atan2(y, x))
    return (brng + 360.0) % 360.0


def mavlink_main():
    """Read telemetry using MAVLink (ArduPilot typical)."""
    try:
        from pymavlink import mavutil
    except Exception as e:
        raise RuntimeError("pymavlink is required for MAVLink mode") from e

    def _request_streams(m) -> None:
        try:
            ts = getattr(m, "target_system", 1) or 1
            tc = getattr(m, "target_component", 1) or 1
            m.mav.request_data_stream_send(ts, tc, mavutil.mavlink.MAV_DATA_STREAM_ALL, 10, 1)
            m.mav.request_data_stream_send(ts, tc, mavutil.mavlink.MAV_DATA_STREAM_EXTENDED_STATUS, 2, 1)
            m.mav.request_data_stream_send(ts, tc, mavutil.mavlink.MAV_DATA_STREAM_POSITION, 5, 1)
            m.mav.request_data_stream_send(ts, tc, mavutil.mavlink.MAV_DATA_STREAM_EXTRA1, 10, 1)
            m.mav.request_data_stream_send(ts, tc, mavutil.mavlink.MAV_DATA_STREAM_EXTRA2, 10, 1)
            m.mav.request_data_stream_send(ts, tc, mavutil.mavlink.MAV_DATA_STREAM_EXTRA3, 2, 1)
        except Exception:
            # Not all firmwares support data stream requests; we still read what is sent.
            pass

    # Auto-reconnect if the port stops delivering data.
    RECONNECT_IDLE_S = float(os.environ.get("MSP_RECONNECT_IDLE_S", "3.0"))
    RECONNECT_BACKOFF_S = float(os.environ.get("MSP_RECONNECT_BACKOFF_S", "0.5"))

    def _candidate_ports() -> list[str]:
        env_port = os.environ.get("MSP_PORT")

        ports: list[str] = []
        if env_port:
            # If the user pinned a port and it exists, respect it.
            # If it doesn't exist (enumeration changed), try to find a replacement.
            if os.path.exists(env_port):
                return [env_port]
            ports.append(env_port)

        ports.extend(sorted(glob.glob("/dev/serial/by-id/*")))
        ports.extend(sorted(glob.glob("/dev/ttyACM*")))
        if PORT and PORT not in ports:
            ports.append(PORT)

        # De-dupe preserving order
        out: list[str] = []
        for p in ports:
            if p not in out:
                out.append(p)
        return out

    def _open_link_blocking(tag: str) -> Tuple["mavutil.mavfile", str]:
        next_log = 0.0
        while True:
            for candidate in _candidate_ports():
                try:
                    m = mavutil.mavlink_connection(candidate, baud=BAUDRATE, dialect="ardupilotmega")
                    hb = m.wait_heartbeat(timeout=3)
                    if hb is None:
                        raise RuntimeError("no heartbeat")
                    _request_streams(m)
                    if tag:
                        print(f"MAVLink {tag} on {candidate}.")
                    return m, candidate
                except Exception:
                    try:
                        m.close()  # type: ignore[name-defined]
                    except Exception:
                        pass

            now = time.time()
            if now >= next_log:
                print(f"MAVLink waiting for {PORT}...")
                next_log = now + 2.0
            time.sleep(RECONNECT_BACKOFF_S)

    # Dialect ardupilotmega is the common superset for ArduPilot.
    mav, active_port = _open_link_blocking("connected")

    # Track home position if available
    home_lat: Optional[float] = None
    home_lon: Optional[float] = None

    print(f"MAVLink read started on {active_port}.")
    last_rx = time.time()

    t_out = time.time()

    while True:
        try:
            msg = mav.recv_match(blocking=True, timeout=0.2)
        except Exception:
            try:
                mav.close()
            except Exception:
                pass
            mav, active_port = _open_link_blocking("reconnected")
            last_rx = time.time()
            continue

        if msg is None:
            now = time.time()
            if now - last_rx >= RECONNECT_IDLE_S:
                try:
                    mav.close()
                except Exception:
                    pass
                mav, active_port = _open_link_blocking("reconnected")
                last_rx = time.time()

            # Periodic output even when nothing received
            if now - t_out >= OUT_DT:
                t_out = now
                with data_lock:
                    print(
                        f"ROLL:{data['roll']} deg "
                        f"PITCH:{data['pitch']} deg "
                        f"YAW:{data['yaw']} deg | "
                        f"ALT:{data['alt']} m  "
                        f"V_SPD:{data['v_speed']} m/s | "
                        f"3D_SPD:{data['speed_3d']} m/s | "
                        f"LAT:{data['lat']}  "
                        f"LON:{data['lon']}  "
                        f"GPS_SPD:{data['speed']} m/s  "
                        f"SATS:{data['sats']}  "
                        f"COURSE:{data['course']}° | "
                        f"VBAT:{data['vbat']} V  "
                        f"CURRENT:{data['current']} A | "
                        f"RSSI:{data['rssi']} | "
                        f"THR:{data['throttle']} | "
                        f"HOME:{data['home_dist']} m  "
                        f"{data['home_dir']}°"
                    )
            continue

        last_rx = time.time()

        mtype = msg.get_type()

        # Home position messages (preferred)
        if mtype == "HOME_POSITION":
            # lat/lon are 1e7
            home_lat = msg.latitude / 1e7
            home_lon = msg.longitude / 1e7

        elif mtype == "GPS_RAW_INT":
            # lat/lon are 1e7, vel cm/s, cog cdeg
            if msg.fix_type and msg.fix_type >= 2:
                lat = msg.lat / 1e7
                lon = msg.lon / 1e7
                speed = (msg.vel or 0) / 100.0
                course = (msg.cog or 0) / 100.0
                sats = msg.satellites_visible

                with data_lock:
                    data["lat"] = lat
                    data["lon"] = lon
                    data["speed"] = speed
                    data["course"] = int(course)
                    data["sats"] = sats

                # Set home on first valid fix if HOME_POSITION not provided
                if home_lat is None or home_lon is None:
                    home_lat, home_lon = lat, lon

        elif mtype == "ATTITUDE":
            # roll/pitch/yaw in rad
            roll = math.degrees(msg.roll)
            pitch = math.degrees(msg.pitch)
            yaw = math.degrees(msg.yaw)
            with data_lock:
                data["roll"] = roll
                data["pitch"] = pitch
                data["yaw"] = yaw

        elif mtype == "VFR_HUD":
            # groundspeed m/s, climb m/s, heading deg, throttle %
            with data_lock:
                data["speed"] = float(msg.groundspeed)
                data["v_speed"] = float(msg.climb)
                data["course"] = int(msg.heading)
                data["throttle"] = float(msg.throttle)

        elif mtype == "GLOBAL_POSITION_INT":
            # relative_alt in mm, vz in cm/s (down is positive in NED)
            rel_alt_m = (msg.relative_alt or 0) / 1000.0
            vz_m_s = -float(msg.vz or 0) / 100.0
            with data_lock:
                data["alt"] = rel_alt_m
                # Prefer VFR_HUD climb when present, but fill if missing
                if data["v_speed"] is None:
                    data["v_speed"] = vz_m_s

        elif mtype == "SYS_STATUS":
            # voltage_battery mV, current_battery cA
            vbat = (msg.voltage_battery or 0) / 1000.0
            current = (msg.current_battery or 0) / 100.0
            with data_lock:
                data["vbat"] = vbat
                data["current"] = current

        elif mtype == "RADIO_STATUS":
            # rssi 0..255 (not the same scale as MSP)
            with data_lock:
                data["rssi"] = float(getattr(msg, "rssi", 0))

        elif mtype == "RC_CHANNELS":
            # chan3_raw typically throttle 1000..2000
            thr = getattr(msg, "chan3_raw", None)
            rssi = getattr(msg, "rssi", None)
            with data_lock:
                if thr is not None:
                    data["throttle"] = float(thr)
                if rssi is not None:
                    data["rssi"] = float(rssi)

        # Derived values
        with data_lock:
            if data["speed"] is not None and data["v_speed"] is not None:
                data["speed_3d"] = math.sqrt(data["speed"] ** 2 + data["v_speed"] ** 2)

            if home_lat is not None and home_lon is not None and data["lat"] is not None and data["lon"] is not None:
                data["home_dist"] = int(_haversine_m(home_lat, home_lon, data["lat"], data["lon"]))
                data["home_dir"] = int(_bearing_deg(home_lat, home_lon, data["lat"], data["lon"]))

        # Data Output (same format as MSP)
        now = time.time()
        if now - t_out >= OUT_DT:
            t_out = now
            with data_lock:
                print(
                    f"ROLL:{data['roll']} deg "
                    f"PITCH:{data['pitch']} deg "
                    f"YAW:{data['yaw']} deg | "
                    f"ALT:{data['alt']} m  "
                    f"V_SPD:{data['v_speed']} m/s | "
                    f"3D_SPD:{data['speed_3d']} m/s | "
                    f"LAT:{data['lat']}  "
                    f"LON:{data['lon']}  "
                    f"GPS_SPD:{data['speed']} m/s  "
                    f"SATS:{data['sats']}  "
                    f"COURSE:{data['course']}° | "
                    f"VBAT:{data['vbat']} V  "
                    f"CURRENT:{data['current']} A | "
                    f"RSSI:{data['rssi']} | "
                    f"THR:{data['throttle']} | "
                    f"HOME:{data['home_dist']} m  "
                    f"{data['home_dir']}°"
                )


# ---------------------------------------- MSP Data Parsers ----------------------------------------

def parse_attitude(p):
    roll, pitch, yaw = struct.unpack("<hhh", p[:6])
    return roll/10.0, pitch/10.0, yaw

def parse_altitude(p):
    alt_cm, vario_cm_s, _ = struct.unpack("<ihh", p[:8])
    return alt_cm/100.0, vario_cm_s/100.0

def parse_gps(p):
    fix, sats, lat, lon, alt, speed, course = struct.unpack("<BBiiiHH", p[:18])
    if fix == 0:
        return {
            "lat": None, "lon": None,
            "speed": None, "sats": sats,
            "course": None
        }
    return {
        "lat": lat / 1e7 if fix >= 2 else None,
        "lon": lon / 1e7 if fix >= 2 else None,
        "speed": speed / 100.0 if fix >= 2 else None,
        "sats": sats, "course": course
    }

def parse_analog(p):
    if len(p) < 5:
        return None, None, None
    vbat_raw, current_raw, rssi_raw = struct.unpack("<BHH", p[:5])
    vbat = vbat_raw / 10.0          # V
    current = current_raw / 100.0   # A
    rssi = rssi_raw                 # 0 ~ 1023
    return vbat, current, rssi

def parse_home(p):
    dist, direction = struct.unpack("<Hh", p[:4])
    return dist, direction

def parse_rc(p):
    if len(p) < 32:
        return None
    return struct.unpack("<16H", p[:32])


# ---------------------------------------- Main ----------------------------------------

data = {
        "roll": None, "pitch": None, "yaw": None,
        "alt": None, "v_speed": None,
        "lat": None, "lon": None,
        "speed": None, "sats": None, "course": None,
        "vbat": None, "current": None,
        "rssi": None, "throttle": None,
        "home_dist": None, "home_dir": None,
        "speed_3d": None
    }

def main():
    forced_protocol = os.environ.get("MSP_PROTOCOL")
    if forced_protocol in {"msp", "mavlink"}:
        protocol = forced_protocol
    else:
        # Robust auto-detect: try MAVLink heartbeat first when likely.
        if _should_try_mavlink_first(PORT) and _mavlink_heartbeat_present(PORT, BAUDRATE):
            protocol = "mavlink"
        else:
            protocol = _detect_protocol(PORT, BAUDRATE)

    if protocol == "mavlink":
        mavlink_main()
        return

    RECONNECT_BACKOFF_S = float(os.environ.get("MSP_RECONNECT_BACKOFF_S", "0.5"))

    def _open_msp_serial_blocking(tag: str) -> Tuple[serial.Serial, str]:
        next_log = 0.0
        while True:
            resolved = _resolve_port(PORT)
            try:
                s = serial.Serial(resolved, BAUDRATE, timeout=0.01)
                time.sleep(0.5)
                if tag:
                    print(f"MSP {tag} on {resolved}.")
                return s, resolved
            except Exception:
                now = time.time()
                if now >= next_log:
                    print(f"MSP waiting for {PORT}...")
                    next_log = now + 2.0
                time.sleep(RECONNECT_BACKOFF_S)

    ser, active_port = _open_msp_serial_blocking("connected")

    print(f"MSP read and output started on {active_port}.")

    t_fast = t_slow = t_out = time.time()

    while True:
        try:
            now = time.time()

            # MSP Requests_Fast Frequency
            if now - t_fast >= FAST_DT:
                send_msp_request(ser, MSP_ATTITUDE)
                t_fast = now

            # MSP Requests_Slow Frequency
            if now - t_slow >= SLOW_DT:
                send_msp_request(ser, MSP_ALTITUDE)
                send_msp_request(ser, MSP_RAW_GPS)
                send_msp_request(ser, MSP_ANALOG)
                send_msp_request(ser, MSP_CURRENT)
                send_msp_request(ser, MSP_COMP_GPS)
                send_msp_request(ser, MSP_RC)
                t_slow = now

            # Read Responses
            read_until = now + 0.005
            while time.time() < read_until:
                resp = read_msp_response(ser)
                if not resp:
                    continue

                cmd, p = resp

                if cmd == MSP_ATTITUDE:
                    roll, pitch, yaw = parse_attitude(p)
                    with data_lock:
                        data["roll"] = roll
                        data["pitch"] = pitch
                        data["yaw"] = yaw

                elif cmd == MSP_ALTITUDE:
                    with data_lock:
                        data["alt"], data["v_speed"] = parse_altitude(p)

                elif cmd == MSP_RAW_GPS:
                    with data_lock:
                        data.update(parse_gps(p))

                elif cmd == MSP_ANALOG:
                    with data_lock:
                        data["vbat"], data["current"], data["rssi"] = parse_analog(p)

                elif cmd == MSP_COMP_GPS:
                    with data_lock:
                        data["home_dist"], data["home_dir"] = parse_home(p)

                elif cmd == MSP_RC:
                    rc = parse_rc(p)
                    if rc:
                        with data_lock:
                            data["throttle"] = rc[2]  # CH3 = Throttle / 1000 ~ 2000

            # Data Output
            if now - t_out >= OUT_DT:
                t_out = now

                # 3D Speed Calculation
                if data["speed"] is not None and data["v_speed"] is not None:
                    spd_3d = math.sqrt(data["speed"]**2 + data["v_speed"]**2)
                else:
                    spd_3d = None

                with data_lock:
                    data["speed_3d"] = spd_3d

                print(
                    #f"t_fast:{t_fast:.2f}, t_slow:{t_slow:.2f}, t_out:{t_out:.2f} | "
                    f"ROLL:{data['roll']} deg "
                    f"PITCH:{data['pitch']} deg "
                    f"YAW:{data['yaw']} deg | "
                    f"ALT:{data['alt']} m  "
                    f"V_SPD:{data['v_speed']} m/s | "
                    f"3D_SPD:{data['speed_3d']} m/s | "
                    f"LAT:{data['lat']}  "
                    f"LON:{data['lon']}  "
                    f"GPS_SPD:{data['speed']} m/s  "
                    f"SATS:{data['sats']}  "
                    f"COURSE:{data['course']}° | "
                    f"VBAT:{data['vbat']} V  "
                    f"CURRENT:{data['current']} A | "
                    f"RSSI:{data['rssi']} | "
                    f"THR:{data['throttle']} | "
                    f"HOME:{data['home_dist']} m  "
                    f"{data['home_dir']}°"
                )

            time.sleep(0.005)

        except (serial.SerialException, OSError):
            try:
                ser.close()
            except Exception:
                pass
            ser, active_port = _open_msp_serial_blocking("reconnected")


# Execute at develop environment
if __name__ == "__main__":
    main()