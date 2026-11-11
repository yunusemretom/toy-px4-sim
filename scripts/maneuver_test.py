#!/usr/bin/env python3
"""PX4'un kontrol yuzeylerini nasil kullandigini gosteren manevra testi.

Ucagi kaldirir, sonra sert donusler (DO_REPOSITION ile geriye/yana nokta) ve irtifa
degisiklikleri yaptirir. Bu sirada PX4'un gz_bridge'inin Gazebo'ya gonderdigi servo
acilarini (/model/toy_0/servo_N) okur ve her manevra icin min/max sapmayi yazdirir.

Kullanim:  python3 maneuver_test.py [udp:127.0.0.1:14540]
Gereken:   PX4 SITL (make px4_sitl gz_toy) calisiyor ve ucak yerde/disarm.
"""
import atexit
import math
import os
import subprocess
import sys
import threading
import time

from pymavlink import mavutil

URL = sys.argv[1] if len(sys.argv) > 1 else "udp:127.0.0.1:14540"
MODEL = os.environ.get("GZ_MODEL", "toy_0")
NAMES = ["sol aileron", "sag aileron", "elevator", "rudder"]
deflection = [0.0] * 4  # derece, PX4'un son komutu
DEBUG = False  # True: her saniye irtifa/yatis/yunuslama yazdir


READERS = []


def stop_readers():
    for proc in READERS:
        proc.terminate()


atexit.register(stop_readers)  # betik bitince gz topic surecleri acik kalmasin


def servo_reader(i):
    # gz topic -e akisi: her mesajda "data: <rad>" satiri (servo yoksa sessizce bekler)
    p = subprocess.Popen(["gz", "topic", "-e", "-t", f"/model/{MODEL}/servo_{i}"],
                         stdout=subprocess.PIPE, stderr=subprocess.DEVNULL, text=True)
    READERS.append(p)
    for line in p.stdout:
        if line.startswith("data:"):
            deflection[i] = math.degrees(float(line.split()[1]))


for i in range(4):
    threading.Thread(target=servo_reader, args=(i,), daemon=True).start()

m = mavutil.mavlink_connection(URL, source_system=255)
m.wait_heartbeat(timeout=60)


def gcs_heartbeat():
    while True:
        m.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_GCS, mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)
        time.sleep(1)


threading.Thread(target=gcs_heartbeat, daemon=True).start()

state = {"lat": 0, "lon": 0, "alt": 0.0, "home_amsl": 0.0, "hdg": 0.0, "roll": 0.0, "pitch": 0.0}


def pump(dt):
    """dt saniye boyunca MAVLink mesajlarini oku, durumu guncelle."""
    t_end = time.time() + dt
    while time.time() < t_end:
        msg = m.recv_match(type=["GLOBAL_POSITION_INT", "ATTITUDE", "STATUSTEXT"], blocking=True, timeout=0.2)
        if msg is None or msg.get_srcComponent() != 1:  # sadece otopilot
            continue
        t = msg.get_type()
        if t == "GLOBAL_POSITION_INT":
            state.update(lat=msg.lat, lon=msg.lon, alt=msg.relative_alt / 1000.0, hdg=msg.hdg / 100.0,
                         home_amsl=(msg.alt - msg.relative_alt) / 1000.0)
        elif t == "ATTITUDE":
            state.update(roll=math.degrees(msg.roll), pitch=math.degrees(msg.pitch))
        elif t == "STATUSTEXT":
            print("  [px4]", msg.text)


def cmd(command, *params):
    p = list(params) + [0] * (7 - len(params))
    m.mav.command_long_send(m.target_system, m.target_component, command, 0, *p)


def reposition(north_m, east_m, alt_rel):
    """Mevcut konuma gore north/east metre otede, home'a gore alt_rel metre irtifada nokta.
    PX4 global cercevedeki COMMAND_INT irtifasini AMSL olarak kullanir, bu yuzden
    home AMSL irtifasi eklenir."""
    lat = state["lat"] / 1e7 + north_m / 111320.0
    lon = state["lon"] / 1e7 + east_m / (111320.0 * math.cos(math.radians(state["lat"] / 1e7)))
    m.mav.command_int_send(m.target_system, m.target_component,
                           mavutil.mavlink.MAV_FRAME_GLOBAL_INT,
                           mavutil.mavlink.MAV_CMD_DO_REPOSITION, 0, 0,
                           -1, 1, 0, float("nan"), int(lat * 1e7), int(lon * 1e7),
                           state["home_amsl"] + alt_rel)


def watch(label, seconds):
    lo = [1e9] * 4
    hi = [-1e9] * 4
    max_roll = 0.0
    min_alt = 1e9
    pmin, pmax = 1e9, -1e9
    t_end = time.time() + seconds
    next_print = time.time()
    while time.time() < t_end:
        pump(0.1)
        if DEBUG and time.time() >= next_print:
            next_print += 1
            print(f"   t-{t_end-time.time():4.1f} alt={state['alt']:6.1f} roll={state['roll']:+5.0f} pitch={state['pitch']:+4.0f}")
        for i in range(4):
            lo[i] = min(lo[i], deflection[i])
            hi[i] = max(hi[i], deflection[i])
        max_roll = max(max_roll, abs(state["roll"]))
        min_alt = min(min_alt, state["alt"])
        pmin, pmax = min(pmin, state["pitch"]), max(pmax, state["pitch"])
    print(f"\n== {label} ({seconds:.0f} s)  irtifa={state['alt']:.0f} m (min {min_alt:.0f})  "
          f"max yatis={max_roll:.0f} deg  yunuslama {pmin:+.0f}..{pmax:+.0f} deg")
    for i, n in enumerate(NAMES):
        print(f"   {n:12s} PX4 komutu: {lo[i]:+6.1f} .. {hi[i]:+6.1f} deg")


pump(5)
hb = m.recv_match(type="HEARTBEAT", condition="HEARTBEAT.get_srcComponent()==1", blocking=True, timeout=5)
airborne = hb is not None and bool(hb.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED) and state["alt"] > 10
if airborne:
    print(f"Ucak zaten havada ({state['alt']:.0f} m), kalkis atlaniyor.")
    watch("mevcut loiter", 5)
else:
    print("Konum kestirimi bekleniyor...")
    while state["lat"] == 0:
        pump(1)
    pump(8)  # pre-arm kontrollerinin oturmasi icin
    print("Kalkis...")
    cmd(mavutil.mavlink.MAV_CMD_DO_SET_MODE, mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, 4, 2)
    for _ in range(10):
        cmd(mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 1)
        ack = m.recv_match(type="COMMAND_ACK", blocking=True, timeout=3)
        if ack and ack.command == mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM and ack.result == 0:
            break
        pump(3)
    watch("kalkis + tirmanis", 25)

hdg = math.radians(state["hdg"])
reposition(-150 * math.cos(hdg), -150 * math.sin(hdg), 40)  # arkaya: 180 deg donus + tirmanis
watch("180 deg donus + 40 m'ye tirmanis", 20)

hdg = math.radians(state["hdg"])
reposition(120 * math.cos(hdg + 1.57), 120 * math.sin(hdg + 1.57), 20)  # saga: sert donus + alcalis
watch("sert sag donus + 20 m'ye alcalis", 20)

hdg = math.radians(state["hdg"])
reposition(120 * math.cos(hdg - 1.57), 120 * math.sin(hdg - 1.57), 35)  # sola
watch("sert sol donus + 35 m'ye tirmanis", 20)

print("\nRTL")
cmd(mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH)
