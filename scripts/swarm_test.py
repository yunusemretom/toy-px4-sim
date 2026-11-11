#!/usr/bin/env python3
"""multi.sh ile baslatilan N araci birlikte kaldirir ve izler.

Her araca (ornek i = 0..N-1, udp:127.0.0.1:14540+i) baglanir, kalkis irtifasini kademeli ayarlar
(havada carpismasinlar diye 20, 28, 36 ... m), Takeoff modunda arm eder ve
belirtilen sure boyunca her aracin irtifa/hiz durumunu tablo olarak yazdirir.

Kullanim:  python3 swarm_test.py [arac_sayisi=10] [izleme_suresi_s=90]
"""
import sys
import threading
import time

from pymavlink import mavutil

N = int(sys.argv[1]) if len(sys.argv) > 1 else 10
DURATION = float(sys.argv[2]) if len(sys.argv) > 2 else 90
BASE_ALT, STEP_ALT = 20.0, 8.0


class Vehicle:
    def __init__(self, i):
        self.i = i
        self.m = mavutil.mavlink_connection(f"udp:127.0.0.1:{14540 + i}", source_system=255)
        self.name = "?"
        self.alt = 0.0
        self.airspeed = 0.0
        self.armed = False
        self.home_set = False
        self.ok = False
        self.msgs = []

    def connect(self):
        hb = self.m.wait_heartbeat(timeout=30)
        if hb is None:
            raise RuntimeError(f"arac {self.i}: heartbeat yok")
        threading.Thread(target=self._gcs_hb, daemon=True).start()
        threading.Thread(target=self._reader, daemon=True).start()

    def _gcs_hb(self):
        while True:
            self.m.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_GCS, mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)
            time.sleep(1)

    def _reader(self):
        while True:
            msg = self.m.recv_match(blocking=True, timeout=1)
            if msg is None or msg.get_srcComponent() != 1:
                continue
            t = msg.get_type()
            if t == "GLOBAL_POSITION_INT":
                self.alt = msg.relative_alt / 1000.0
                self.home_set = self.home_set or msg.lat != 0
            elif t == "VFR_HUD":
                self.airspeed = msg.airspeed
            elif t == "HEARTBEAT":
                self.armed = bool(msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
            elif t == "COMMAND_ACK":
                self.msgs.append(("ack", msg.command, msg.result))
            elif t == "STATUSTEXT" and ("denied" in msg.text.lower() or "fail" in msg.text.lower()):
                self.msgs.append(("txt", msg.text))

    def cmd(self, command, *params):
        p = list(params) + [0] * (7 - len(params))
        self.m.mav.command_long_send(self.m.target_system, self.m.target_component, command, 0, *p)

    def set_param(self, name, value):
        self.m.mav.param_set_send(self.m.target_system, self.m.target_component, name.encode(), value,
                                  mavutil.mavlink.MAV_PARAM_TYPE_REAL32)


vehicles = [Vehicle(i) for i in range(N)]  # PX4 ornek 0..N-1
print(f"{N} araca baglaniliyor...")
for v in vehicles:
    v.connect()
    print(f"  arac {v.i}: sistem id {v.m.target_system}")

print("Konum kestirimi bekleniyor...")
t0 = time.time()
while not all(v.home_set for v in vehicles) and time.time() - t0 < 120:
    time.sleep(1)
time.sleep(8)

for k, v in enumerate(vehicles):
    v.set_param("MIS_TAKEOFF_ALT", BASE_ALT + STEP_ALT * k)
time.sleep(1)

print("Kalkis (Takeoff modu + arm)...")
for v in vehicles:
    v.cmd(mavutil.mavlink.MAV_CMD_DO_SET_MODE, mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, 4, 2)
time.sleep(1)
for attempt in range(8):
    for v in vehicles:
        if not v.armed:
            v.cmd(mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 1)
    time.sleep(3)
    if all(v.armed for v in vehicles):
        break

print(f"\n{'t(s)':>5} " + " ".join(f"{'#'+str(v.i):>6}" for v in vehicles) + "   havada")
t0 = time.time()
next_print = 0
while time.time() - t0 < DURATION:
    t = time.time() - t0
    if t >= next_print:
        up = sum(v.alt > 5 for v in vehicles)
        print(f"{t:5.0f} " + " ".join(f"{v.alt:6.1f}" for v in vehicles) + f"   {up}/{N}")
        next_print += 10
    time.sleep(0.5)

print("\nOzet (irtifa m / hava hizi m/s / hedef irtifa):")
for k, v in enumerate(vehicles):
    hedef = BASE_ALT + STEP_ALT * k
    durum = "TAMAM" if abs(v.alt - hedef) < 6 else "SORUN"
    print(f"  arac {v.i:2d}: {v.alt:5.1f} m  {v.airspeed:4.1f} m/s  hedef {hedef:4.0f} m  {durum}"
          + ("" if v.armed else "  (arm degil)"))
    for m in v.msgs[-3:]:
        if m[0] == "txt":
            print(f"           px4: {m[1]}")

print("\nHepsi RTL")
for v in vehicles:
    v.cmd(mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH)
