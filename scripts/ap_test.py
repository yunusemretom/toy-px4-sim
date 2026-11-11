#!/usr/bin/env python3
"""ArduPlane (ardupilot/run.sh) icin kalkis + manevra / suru testi.

    python3 scripts/ap_test.py maneuver [ornek=0]      tek arac: kalkis, sert donusler, irtifa degisimi
    python3 scripts/ap_test.py swarm <n> [sure_s=90]   n arac: kademeli irtifalara birlikte kalkis

Arac i: tcp:127.0.0.1:(5760+10*i). Kalkis ArduPlane TAKEOFF moduyla (TKOFF_ALT; surude arac basina
20, 28, 36 ... m), manevralar GUIDED modda MAV_CMD_DO_REPOSITION ile.
"""
import math
import sys
import threading
import time

from pymavlink import mavutil


class Plane:
    def __init__(self, i):
        self.i = i
        self.m = mavutil.mavlink_connection(f"tcp:127.0.0.1:{5760 + 10 * i}", source_system=255,
                                            source_component=190)
        self.m.wait_heartbeat(timeout=90)
        self.m.mav.request_data_stream_send(self.m.target_system, self.m.target_component,
                                            mavutil.mavlink.MAV_DATA_STREAM_ALL, 10, 1)
        self.s = dict(lat=0, lon=0, alt=0.0, home_amsl=0.0, hdg=0.0, roll=0.0, pitch=0.0, aspd=0.0,
                      armed=False, mode="", ekf_ok=False)
        self.texts = []
        threading.Thread(target=self._reader, daemon=True).start()

    def _reader(self):
        while True:
            msg = self.m.recv_match(blocking=True, timeout=1)
            if msg is None:
                continue
            t = msg.get_type()
            if t == "GLOBAL_POSITION_INT":
                self.s.update(lat=msg.lat, lon=msg.lon, alt=msg.relative_alt / 1000.0, hdg=msg.hdg / 100.0,
                              home_amsl=(msg.alt - msg.relative_alt) / 1000.0)
            elif t == "ATTITUDE":
                self.s.update(roll=math.degrees(msg.roll), pitch=math.degrees(msg.pitch))
            elif t == "VFR_HUD":
                self.s["aspd"] = msg.airspeed
            elif t == "HEARTBEAT" and msg.get_srcComponent() == 1:
                self.s["armed"] = bool(msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
                self.s["mode"] = mavutil.mode_string_v10(msg)
            elif t == "EKF_STATUS_REPORT":
                self.s["ekf_ok"] = bool(msg.flags & 0x10)  # EKF_POS_HORIZ_ABS
            elif t == "STATUSTEXT":
                self.texts.append(msg.text)

    def wait_ready(self, timeout=120):
        t0 = time.time()
        while time.time() - t0 < timeout:
            if self.s["lat"] != 0 and self.s["ekf_ok"]:
                return True
            time.sleep(1)
        return False

    def mode(self, name):
        self.m.set_mode(name)

    def param(self, name, value):
        self.m.mav.param_set_send(self.m.target_system, self.m.target_component, name.encode(), value,
                                  mavutil.mavlink.MAV_PARAM_TYPE_REAL32)

    def arm(self):
        for _ in range(10):
            # 21196: on kontrolleri atla (SITL)
            self.m.mav.command_long_send(self.m.target_system, self.m.target_component,
                                         mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 1, 21196, 0, 0, 0, 0, 0)
            time.sleep(2)
            if self.s["armed"]:
                return True
        return False

    def reposition(self, north, east, alt_rel):
        lat = self.s["lat"] / 1e7 + north / 111320.0
        lon = self.s["lon"] / 1e7 + east / (111320.0 * math.cos(math.radians(self.s["lat"] / 1e7)))
        self.m.mav.command_int_send(self.m.target_system, self.m.target_component,
                                    mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
                                    mavutil.mavlink.MAV_CMD_DO_REPOSITION, 0, 0, -1,
                                    mavutil.mavlink.MAV_DO_REPOSITION_FLAGS_CHANGE_MODE, 0, float("nan"),
                                    int(lat * 1e7), int(lon * 1e7), alt_rel)


def watch(p, label, seconds):
    min_alt, max_roll, pmin, pmax = 1e9, 0.0, 1e9, -1e9
    t_end = time.time() + seconds
    while time.time() < t_end:
        min_alt = min(min_alt, p.s["alt"])
        max_roll = max(max_roll, abs(p.s["roll"]))
        pmin, pmax = min(pmin, p.s["pitch"]), max(pmax, p.s["pitch"])
        time.sleep(0.1)
    print(f"== {label} ({seconds:.0f} s)  irtifa={p.s['alt']:.0f} m (min {min_alt:.0f})  hava hizi={p.s['aspd']:.1f}  "
          f"max yatis={max_roll:.0f} deg  yunuslama {pmin:+.0f}..{pmax:+.0f} deg  mod={p.s['mode']}")


def maneuver(i):
    p = Plane(i)
    print("EKF bekleniyor...")
    if not p.wait_ready():
        sys.exit("EKF konum kestirimi hazir degil")
    time.sleep(5)
    p.mode("TAKEOFF")
    time.sleep(1)
    print("arm:", p.arm())
    watch(p, "kalkis (TAKEOFF)", 30)
    hdg = math.radians(p.s["hdg"])
    p.reposition(-150 * math.cos(hdg), -150 * math.sin(hdg), 40)
    watch(p, "180 deg donus + 40 m'ye tirmanis", 25)
    hdg = math.radians(p.s["hdg"])
    p.reposition(120 * math.cos(hdg + 1.57), 120 * math.sin(hdg + 1.57), 20)
    watch(p, "sert sag donus + 20 m'ye alcalis", 25)
    hdg = math.radians(p.s["hdg"])
    p.reposition(120 * math.cos(hdg - 1.57), 120 * math.sin(hdg - 1.57), 35)
    watch(p, "sert sol donus + 35 m'ye tirmanis", 25)
    for t in p.texts[-6:]:
        print("   ap:", t)
    p.mode("RTL")


def swarm(n, duration):
    planes = [Plane(i) for i in range(n)]
    print(f"{n} arac bagli. EKF bekleniyor...")
    for p in planes:
        p.wait_ready()
    time.sleep(5)
    for k, p in enumerate(planes):
        p.param("TKOFF_ALT", 20 + 8 * k)
    time.sleep(1)
    for p in planes:
        p.mode("TAKEOFF")
    time.sleep(1)
    for p in planes:
        p.arm()
    t0 = time.time()
    print(f"{'t(s)':>5} " + " ".join(f"{'#' + str(p.i):>6}" for p in planes) + "   havada")
    nxt = 0
    while time.time() - t0 < duration:
        if time.time() - t0 >= nxt:
            up = sum(p.s["alt"] > 5 for p in planes)
            print(f"{time.time() - t0:5.0f} " + " ".join(f"{p.s['alt']:6.1f}" for p in planes) + f"   {up}/{n}")
            nxt += 10
        time.sleep(0.5)
    print("\nOzet:")
    for k, p in enumerate(planes):
        target = 20 + 8 * k
        ok = "TAMAM" if abs(p.s["alt"] - target) < 8 else "SORUN"
        print(f"  arac {p.i}: {p.s['alt']:5.1f} m  hava hizi {p.s['aspd']:4.1f} m/s  hedef {target} m  {ok}  mod={p.s['mode']}")
    for p in planes:
        p.mode("RTL")


if __name__ == "__main__":
    what = sys.argv[1] if len(sys.argv) > 1 else "maneuver"
    if what == "maneuver":
        maneuver(int(sys.argv[2]) if len(sys.argv) > 2 else 0)
    else:
        swarm(int(sys.argv[2]), float(sys.argv[3]) if len(sys.argv) > 3 else 90)
