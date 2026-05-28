#!/usr/bin/env python3
"""Simple MAVLink serial tester using pymavlink.

Usage:
  python test_mavlink_serial.py --port /dev/ttyACM0 --baud 115200
"""
from pymavlink import mavutil
import argparse
import time


def main():
    p = argparse.ArgumentParser(description="MAVLink serial test tool")
    p.add_argument("--port", required=True, help="Serial port (e.g. /dev/ttyACM0)")
    p.add_argument("--baud", type=int, default=115200, help="Baud rate")
    args = p.parse_args()

    print(f"Connecting to {args.port} @ {args.baud}...")
    m = mavutil.mavlink_connection(args.port, baud=args.baud)

    # send a GCS heartbeat so some devices respond
    try:
        m.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_GCS,
                             mavutil.mavlink.MAV_AUTOPILOT_INVALID,
                             0, 0, 0)
    except Exception:
        pass

    print("Waiting for heartbeat (10s timeout)...")
    try:
        hb = m.wait_heartbeat(timeout=10)
    except Exception as e:
        hb = None

    if hb is None:
        print("No heartbeat received. Check wiring/port/baud and that Arduino is running MAVLink.")
    else:
        print(f"Heartbeat received from system {m.target_system} component {m.target_component}")

    print("Listening for messages (Ctrl-C to stop)...")
    try:
        while True:
            msg = m.recv_match(blocking=True, timeout=5)
            if msg is None:
                continue
            print(msg)
    except KeyboardInterrupt:
        print("Stopped by user")


if __name__ == '__main__':
    main()
