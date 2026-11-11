#!/usr/bin/env python3
"""ArduPlane servo fonksiyonlarinin PWM yonlerini olcer -> tools/ardupilot_signs.json.

ArduPlane'i kendi ic ucak modeliyle (Gazebo'suz) calistirir; servo kanallarina aileron (4),
elevator (19), rudder (21), elevon sol/sag (77/78), V-kuyruk sol/sag (79/80) atar. MANUAL modda
RC override ile saga yatis, burun yukari (RC2 yuksek) ve saga yon komutu verip her kanalin PWM'inin
artip azaldigini kaydeder. gen_ardupilot.py bu yonleri modelin AVL turevleriyle birlestirerek
eklem carpaninin isaretini secer.

    python3 tools/ap_calibrate.py ~/Documents/Projeler/ardupilot
"""
import json
import subprocess
import sys
import tempfile
import time
from pathlib import Path

from pymavlink import mavutil

AP = Path(sys.argv[1] if len(sys.argv) > 1 else Path.home() / "ardupilot").expanduser()
BIN = AP / "build" / "sitl" / "bin" / "arduplane"
FUNCS = [4, 19, 21, 77, 78, 79, 80]
OUT = Path(__file__).resolve().parent / "ardupilot_signs.json"

parm = tempfile.NamedTemporaryFile("w", suffix=".parm", delete=False)
for ch, f in enumerate(FUNCS + [70], start=1):
    parm.write(f"SERVO{ch}_FUNCTION {f}\nSERVO{ch}_MIN 1000\nSERVO{ch}_MAX 2000\n"
               f"SERVO{ch}_TRIM {1000 if f == 70 else 1500}\nSERVO{ch}_REVERSED 0\n")
parm.write("ARMING_CHECK 0\n")
parm.close()

work = tempfile.mkdtemp()
proc = subprocess.Popen([str(BIN), "--model", "plane", "--speedup", "1", "--defaults", parm.name,
                         "-I", "0", "--home", "-35.36,149.16,584,0"], cwd=work,
                        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
try:
    time.sleep(3)
    m = mavutil.mavlink_connection("tcp:127.0.0.1:5760", source_system=255)
    m.wait_heartbeat(timeout=60)
    m.mav.request_data_stream_send(m.target_system, m.target_component, mavutil.mavlink.MAV_DATA_STREAM_ALL, 20, 1)
    m.set_mode("MANUAL")

    def outputs(rc):
        """RC override uygula, SERVO_OUTPUT_RAW ortalamasini dondur."""
        vals = [65535] * 8
        for k, v in rc.items():
            vals[k - 1] = v
        t_end = time.time() + 2.5
        samples = []
        while time.time() < t_end:
            m.mav.rc_channels_override_send(m.target_system, m.target_component, *vals)
            msg = m.recv_match(type="SERVO_OUTPUT_RAW", blocking=True, timeout=0.3)
            if msg and time.time() > t_end - 1.0:
                samples.append([getattr(msg, f"servo{i}_raw") for i in range(1, 9)])
        return [sum(c) / len(c) for c in zip(*samples)]

    neutral = outputs({1: 1500, 2: 1500, 3: 1000, 4: 1500})
    # ArduPlane: RC2 yuksek = burun yukari (kalkista elevatoru 2000 us yapip burun kaldirmaya calisir)
    tests = dict(roll={1: 1900}, pitch={2: 1900}, yaw={4: 1900})
    result = {str(f): {} for f in FUNCS}
    for axis, rc in tests.items():
        cmd = {1: 1500, 2: 1500, 3: 1000, 4: 1500}
        cmd.update(rc)
        out = outputs(cmd)
        for ch, f in enumerate(FUNCS):
            delta = out[ch] - neutral[ch]
            result[str(f)][axis] = 0 if abs(delta) < 20 else (1 if delta > 0 else -1)
    OUT.write_text(json.dumps(result, indent=2) + "\n")
    print(json.dumps(result, indent=2))
finally:
    proc.terminate()
