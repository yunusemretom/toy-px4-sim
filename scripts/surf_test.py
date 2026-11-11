#!/usr/bin/env python3
"""Arm etmeden MAV_CMD_ACTUATOR_TEST ile servolari sirayla +/- sonuna kadar saptirir.
Kullanim: surf_test.py [sure_s]"""
import sys
import threading
import time

import os
os.environ.setdefault("MAVLINK20", "1")
os.environ.setdefault("MAVLINK_DIALECT", "common")
from pymavlink import mavutil

HOLD = float(sys.argv[1]) if len(sys.argv) > 1 else 2.0
m = mavutil.mavlink_connection("udp:127.0.0.1:14540", source_system=255)
m.wait_heartbeat(timeout=30)


def hb():
    while True:
        m.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_GCS, mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)
        time.sleep(1)


threading.Thread(target=hb, daemon=True).start()
SERVO1 = 33  # ACTUATOR_OUTPUT_FUNCTION_SERVO1
names = ["sol aileron", "sag aileron", "elevator", "rudder"]
for i, n in enumerate(names):
    for v in (1.0, -1.0):
        m.mav.command_long_send(m.target_system, m.target_component,
                                310, 0,  # MAV_CMD_ACTUATOR_TEST
                                v, HOLD + 1, 0, 0, SERVO1 + i, 0, 0)
        ack = m.recv_match(type="COMMAND_ACK", blocking=True, timeout=3)
        print(f"{n} {v:+.0f}: ack={ack.result if ack else None}", flush=True)
        time.sleep(HOLD)
