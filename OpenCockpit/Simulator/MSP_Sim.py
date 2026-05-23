"""Simulated MSP data source.

Drop-in replacement for MSP_Read_pi: exposes `data`, `data_lock`, `main()`.
Generates animated telemetry so displays can be tested without a flight controller.
"""

import math
import random
import threading
import time

data_lock = threading.Lock()

data = {
    "roll": None, "pitch": None, "yaw": None,
    "alt": None, "v_speed": None,
    "lat": None, "lon": None,
    "speed": None, "sats": None, "course": None,
    "vbat": None, "current": None,
    "rssi": None, "throttle": None,
    "home_dist": None, "home_dir": None,
    "speed_3d": None,
}

# Home reference (matches default in main.py)
HOME_LAT = 10.7769     # HCMC center
HOME_LON = 106.7009

# Loop frequency
OUT_HZ = 30.0
OUT_DT = 1.0 / OUT_HZ


def _haversine_m(lat1, lon1, lat2, lon2):
    r = 6371000.0
    phi1, phi2 = math.radians(lat1), math.radians(lat2)
    dphi = math.radians(lat2 - lat1)
    dlambda = math.radians(lon2 - lon1)
    a = math.sin(dphi / 2) ** 2 + math.cos(phi1) * math.cos(phi2) * math.sin(dlambda / 2) ** 2
    return r * 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))


def _bearing_deg(lat1, lon1, lat2, lon2):
    phi1, phi2 = math.radians(lat1), math.radians(lat2)
    dlambda = math.radians(lon2 - lon1)
    y = math.sin(dlambda) * math.cos(phi2)
    x = math.cos(phi1) * math.sin(phi2) - math.sin(phi1) * math.cos(phi2) * math.cos(dlambda)
    return (math.degrees(math.atan2(y, x)) + 360.0) % 360.0


def main():
    print("MSP Simulator started (no real FC required).")
    t0 = time.time()
    t_print = t0

    while True:
        t = time.time() - t0

        # Attitude — slow sinusoidal motion
        pitch = 25.0 * math.sin(t * 0.4)
        roll = 35.0 * math.sin(t * 0.3 + 1.0)
        yaw = (t * 20.0) % 360.0  # slow continuous yaw rotation

        # Altitude — climb to 80m then descend, repeat
        alt = 50.0 + 40.0 * math.sin(t * 0.15)
        v_speed = 40.0 * 0.15 * math.cos(t * 0.15)  # derivative

        # Ground speed and 3D speed
        ground_speed = 12.0 + 6.0 * math.sin(t * 0.25)
        speed_3d = math.sqrt(ground_speed ** 2 + v_speed ** 2)

        # GPS — fly a circle around HOME with radius ~150 m
        radius_deg = 0.0015
        lat = HOME_LAT + radius_deg * math.sin(t * 0.1)
        lon = HOME_LON + radius_deg * math.cos(t * 0.1)
        course = int((math.degrees(t * 0.1) + 90.0) % 360.0)
        sats = random.randint(10, 15)

        # Battery — slow drain
        vbat = max(14.0, 16.8 - (t / 600.0))
        current = 10.0 + 4.0 * math.sin(t * 0.5)

        # Other
        rssi = random.randint(850, 1023)
        throttle = 1500 + int(400 * math.sin(t * 0.4))

        # Home distance / direction
        home_dist = int(_haversine_m(HOME_LAT, HOME_LON, lat, lon))
        home_dir = int(_bearing_deg(HOME_LAT, HOME_LON, lat, lon))

        with data_lock:
            data["pitch"] = pitch
            data["roll"] = roll
            data["yaw"] = yaw
            data["alt"] = alt
            data["v_speed"] = v_speed
            data["lat"] = lat
            data["lon"] = lon
            data["speed"] = ground_speed
            data["speed_3d"] = speed_3d
            data["sats"] = sats
            data["course"] = course
            data["vbat"] = vbat
            data["current"] = current
            data["rssi"] = rssi
            data["throttle"] = throttle
            data["home_dist"] = home_dist
            data["home_dir"] = home_dir

        now = time.time()
        if now - t_print >= 1.0:
            t_print = now
            with data_lock:
                print(
                    f"[SIM] ROLL:{data['roll']:.1f} PITCH:{data['pitch']:.1f} YAW:{data['yaw']:.1f} | "
                    f"ALT:{data['alt']:.1f}m V:{data['v_speed']:.2f}m/s SPD:{data['speed_3d']:.1f} | "
                    f"VBAT:{data['vbat']:.2f}V THR:{data['throttle']} HOME:{data['home_dist']}m"
                )

        time.sleep(OUT_DT)


if __name__ == "__main__":
    main()
