#!/usr/bin/env python3
"""pymavlink ile sabit kanat SITL testi: baglan, arm, takeoff, irtifa/hiz izle, RTL."""
import sys
import time
from pymavlink import mavutil

URL = sys.argv[1] if len(sys.argv) > 1 else "udp:127.0.0.1:14540"
DURATION = float(sys.argv[2]) if len(sys.argv) > 2 else 60

import threading

m = mavutil.mavlink_connection(URL, source_system=255)
m.wait_heartbeat(timeout=60)


def gcs_heartbeat():
    # PX4 GCS baglantisi olmadan arm etmez; QGC yerine biz GCS heartbeat'i gonderiyoruz
    while True:
        m.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_GCS, mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)
        time.sleep(1)


threading.Thread(target=gcs_heartbeat, daemon=True).start()
print(f"heartbeat sys={m.target_system} comp={m.target_component}")

# EKF hazir olana kadar bekle (GPS fix + global position)
t0 = time.time()
while time.time() - t0 < 90:
    msg = m.recv_match(type="GLOBAL_POSITION_INT", blocking=True, timeout=2)
    if msg and msg.lat != 0:
        break
print("global position ok")
time.sleep(8)  # pre-arm kontrollerinin oturmasi icin


def cmd(command, *params):
    p = list(params) + [0] * (7 - len(params))
    m.mav.command_long_send(m.target_system, m.target_component, command, 0, *p)
    ack = m.recv_match(type="COMMAND_ACK", blocking=True, timeout=5)
    print(f"cmd {command} -> {ack.result if ack else 'no ack'}")
    return ack


# Takeoff modu (PX4 custom main mode AUTO=4, sub TAKEOFF=2), sonra arm
cmd(mavutil.mavlink.MAV_CMD_DO_SET_MODE, mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, 4, 2)
for _ in range(5):
    ack = cmd(mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 1)
    if ack and ack.result == 0:
        break
    time.sleep(3)

max_alt = 0.0
t0 = time.time()
while time.time() - t0 < DURATION:
    msg = m.recv_match(type=["VFR_HUD", "STATUSTEXT"], blocking=True, timeout=2)
    if msg is None:
        continue
    if msg.get_type() == "STATUSTEXT":
        print("  [px4]", msg.text)
    else:
        max_alt = max(max_alt, msg.alt)
        if int(time.time() - t0) % 5 == 0:
            print(f"  t={time.time()-t0:5.1f}s alt={msg.alt:6.1f} m  airspeed={msg.airspeed:5.1f} gs={msg.groundspeed:5.1f} m/s  thr={msg.throttle}%")
            time.sleep(1)

print(f"MAX ALT (AMSL) = {max_alt:.1f} m")
cmd(mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH)
