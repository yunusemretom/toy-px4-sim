#!/usr/bin/env python3
"""PX4 modellerinden ArduPilot (ardupilot_gazebo) surumlerini uretir.

    python3 tools/gen_ardupilot.py --all        # ya da: talon x8 ...

Her model icin:
  models/<ad>_ap/model.sdf, model.config      ArduPilot Gazebo modeli
  ardupilot/params/<ad>.parm                  ArduPlane parametreleri

Donusum (aerodinamik, mesh'ler, eklemler PX4 modeliyle ayni kalir):
  - PX4'e ozgu parcalar cikarilir: barometre / manyetometre / GPS sensorleri (ArduPilot SITL
    bunlari kendi uretir), PX4 hava hizi eklentisi, PX4 motor eklentisi.
  - Motor: gz-sim Thruster sistemi; ArduPilot gaz kanali itkiyi (N) dogrudan komut eder.
  - ArduPilotPlugin: her kontrol yuzeyi kanali COMMAND tipiyle mevcut eklem konum
    kontrolcusunun konusuna (/model/<ad>_ap/servo_k) aci (rad) olarak yazar.
    aci = carpan * ((PWM - 1000) / 1000 - 0.5),  carpan = +-2 * 25 deg.
  - Carpanin isareti: tools/ardupilot_signs.json (tools/ap_calibrate.py ile ArduPilot'un kendi
    ic ucak modelinde olculen PWM yonleri) ve modelin AVL turevlerinden; ornegin "saga yatis"
    komutu fiziksel olarak saga yatiran sapmayi uretir.
"""
import argparse
import json
import math
import re
import sys
from pathlib import Path

REPO = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO / "tools"))
from plane_specs import PLANES  # noqa: E402

SERVO_RAD = math.radians(25.0)
# Kontrolun asil ekseni ve "yuksek PWM" ile istenen etki: ArduPilot'ta roll+ = saga yatis,
# pitch+ = burun yukari, yaw+ = burun saga (AVL stabilite eksenleriyle ayni isaretler).
AXIS = dict(roll="Cl", pitch="Cm", yaw="Cn")


def parse_controls(sdf):
    """PX4 model.sdf'den kontrol yuzeyleri: [(servo_index, link_adi, {CL..Cn})]."""
    out = []
    for m in re.finditer(r"<control_surface>\s*<name>servo_(\d+)</name>.*?</control_surface>", sdf, re.S):
        blk = m.group(0)
        d = {k: float(re.search(rf"<{k}_ctrl>([^<]+)</{k}_ctrl>", blk).group(1))
             for k in ("CD", "CY", "CL", "Cell", "Cem", "Cen")}
        k = int(m.group(1))
        # eklem pozitif donusunun etkisi = direction * katsayi (toy'da elevator direction = -1)
        dr = float(re.search(r"<direction>([^<]+)</direction>", blk).group(1))
        child = re.search(rf"<joint name='servo_{k}' type='revolute'>\s*<parent>base_link</parent>\s*"
                          r"<child>([^<]+)</child>", sdf).group(1)
        out.append((k, child, dict(Cl=dr * d["Cell"], Cm=dr * d["Cem"], Cn=dr * d["Cen"])))
    return out


CANDIDATES = dict(ail_l=[4], ail_r=[4], elevator=[19], rudder=[21], el_l=[77, 78], el_r=[78, 77],
                  rv_l=[79, 80], rv_r=[80, 79])


def choose_function(kind, derivs, signs):
    """Yuzey icin (ArduPilot fonksiyonu, carpan isareti) secer.

    signs[f][eksen] = stick + yonunde PWM'in yonu (+1/-1/0). Eklem + yonunun eksen etkisi derivs.
    Istenen: her onemli eksende  carpan * pwm_yonu * etki > 0. Elevon/V-kuyrukta iki fonksiyon da
    denenir (ArduPilot'un sol/sag kariminin isaretleri yuzeyin geometrisine uymayabilir)."""
    mags = {ax: abs(derivs[c]) for ax, c in AXIS.items()}
    top = max(mags.values())
    for func in CANDIDATES[kind]:
        sg = signs[str(func)]
        votes = {1 if sg[ax] * derivs[AXIS[ax]] > 0 else -1
                 for ax in AXIS if sg[ax] != 0 and mags[ax] >= 0.25 * top}
        if len(votes) == 1:
            return func, votes.pop()
    raise RuntimeError(f"{kind}: tutarli ArduPilot fonksiyonu bulunamadi ({derivs})")


def convert(name, signs):
    src = (REPO / "models" / name / "model.sdf").read_text()
    ap = f"{name}_ap"
    sdf = src
    sdf = sdf.replace(f'<model name="{name}">', f'<model name="{ap}">', 1)
    # PX4'e ozgu sensorler ve eklentiler
    for sensor in ("air_pressure_sensor", "magnetometer_sensor", "navsat_sensor"):
        sdf = re.sub(rf'\s*<sensor name="{sensor}".*?</sensor>', "", sdf, flags=re.S)
    sdf = re.sub(r'\s*<plugin filename="libAirSpeedPlugin.so".*?</plugin>', "", sdf, flags=re.S)
    # ArduPilotPlugin IMU olcumunu donusturmeden gonderir ve sensor ekseninin ArduPilot govde
    # ekseni (x ileri, y saga, z asagi) oldugunu varsayar. Govdemiz x ileri / y sola / z yukari:
    # sensore x etrafinda 180 deg poz verilir (ardupilot_gazebo zephyr modeli de boyle yapar).
    # Guncelleme hizi dunyanin 4 ms fizik adimina uygun 250 Hz.
    sdf = sdf.replace('<sensor name="imu_sensor" type="imu">',
                      '<sensor name="imu_sensor" type="imu">\n        <pose degrees="true">0 0 0 180 0 0</pose>', 1)
    sdf = re.sub(r'(<sensor name="imu_sensor" type="imu">.*?<update_rate>)\d+(</update_rate>)', r"\g<1>250\2",
                 sdf, count=1, flags=re.S)
    m = re.search(r'\s*<plugin filename="gz-sim-multicopter-motor-model-system".*?</plugin>', sdf, re.S)
    motor_blk = m.group(0)
    sdf = sdf.replace(motor_blk, "")
    # itki: motorConstant * ESC_MAX^2 (PX4 tarafinda ESC_MAX = 1000 rad/s)
    kmot = float(re.search(r"<motorConstant>([^<]+)</motorConstant>", motor_blk).group(1))
    joint = re.search(r"<jointName>([^<]+)</jointName>", motor_blk).group(1)
    tmax = kmot * 1000.0**2
    prop_r = float(re.search(r"<link name=\"rotor\">.*?<radius>([^<]+)</radius>", sdf, re.S).group(1)) \
        if '<link name="rotor">' in sdf else 0.12
    # PX4 modeli toy icin farkli rotor link adi kullaniyor
    if '<link name="rotor">' not in sdf:
        prop_r = float(re.search(r"<link name='rotor_pusher'>.*?<radius>([^<]+)</radius>", sdf, re.S).group(1))

    controls = parse_controls(src)
    ctrl_xml = []
    channel_funcs = []
    for k, child, d in controls:
        kind = child if child in CANDIDATES else {"left_aileron": "ail_l", "right_aileron": "ail_r"}.get(child, child)
        func, sgn = choose_function(kind, d, signs)
        channel_funcs.append(func)
        ctrl_xml.append(f"""      <control channel="{k}">
        <jointName>servo_{k}</jointName>
        <type>COMMAND</type>
        <cmd_topic>/model/{ap}/servo_{k}</cmd_topic>
        <multiplier>{sgn * 2 * SERVO_RAD:.4f}</multiplier>
        <offset>-0.5</offset>
        <servo_min>1000</servo_min>
        <servo_max>2000</servo_max>
      </control>""")
    thr_ch = len(controls)
    channel_funcs.append(70)
    ctrl_xml.append(f"""      <control channel="{thr_ch}">
        <jointName>{joint}</jointName>
        <type>COMMAND</type>
        <cmd_topic>/model/{ap}/joint/{joint}/cmd_thrust</cmd_topic>
        <multiplier>{tmax:.2f}</multiplier>
        <offset>0</offset>
        <servo_min>1000</servo_min>
        <servo_max>2000</servo_max>
      </control>""")

    plugins = f"""
    <!-- Motor: ArduPilot gaz kanali itkiyi (N, 0..{tmax:.0f}) dogrudan komut eder. -->
    <plugin filename="gz-sim-thruster-system" name="gz::sim::systems::Thruster">
      <namespace>{ap}</namespace>
      <joint_name>{joint}</joint_name>
      <thrust_coefficient>0.1</thrust_coefficient>
      <fluid_density>1.2041</fluid_density>
      <propeller_diameter>{2 * prop_r:.3f}</propeller_diameter>
      <max_thrust_cmd>{tmax:.2f}</max_thrust_cmd>
      <min_thrust_cmd>0</min_thrust_cmd>
    </plugin>
    <!-- ArduPilot JSON koprusu. Govde ekseni x ileri / y sola / z yukari -> FRD icin x etrafinda
         180 deg; dunya ENU -> NED. fdm_port_in cok aracli calistirmada ornek basina degistirilir. -->
    <plugin name="ArduPilotPlugin" filename="ArduPilotPlugin">
      <fdm_addr>127.0.0.1</fdm_addr>
      <fdm_port_in>9002</fdm_port_in>
      <connectionTimeoutMaxCount>5</connectionTimeoutMaxCount>
      <lock_step>1</lock_step>
      <modelXYZToAirplaneXForwardZDown degrees="true">0 0 0 180 0 0</modelXYZToAirplaneXForwardZDown>
      <gazeboXYZToNED degrees="true">0 0 0 180 0 90</gazeboXYZToNED>
      <imuName>imu_sensor</imuName>
{chr(10).join(ctrl_xml)}
    </plugin>
"""
    i = sdf.rindex("  </model>")
    sdf = sdf[:i] + plugins + sdf[i:]
    sdf = sdf.replace("<!--", "<!-- ArduPilot surumu (tools/gen_ardupilot.py). ", 1)

    out = REPO / "models" / ap
    out.mkdir(parents=True, exist_ok=True)
    (out / "model.sdf").write_text(sdf)
    title = PLANES[name]["title"] if name in PLANES else name.upper()
    (out / "model.config").write_text(f"""<?xml version="1.0"?>
<model>
  <name>{title} (ArduPilot)</name>
  <version>1.0</version>
  <sdf version="1.9">model.sdf</sdf>
  <description>{title}, ArduPilot SITL + ardupilot_gazebo icin (tools/gen_ardupilot.py).</description>
</model>
""")
    write_params(name, src, channel_funcs, tmax)
    return controls, channel_funcs, tmax


# Model ozel ArduPlane ayarlari. Uretilen modeller (Talon, Mini Talon, X8, Skywalker 1900)
# ArduPlane'in varsayilan acisal hiz kazanclariyla ucuyor. TOY varsayilan yunuslama kazanclariyla
# +-60 deg salinim yapti; denemeler: 2.4x -> kalkamadi, 0.5x -> salinim +-30-50 deg,
# 0.25x -> manevralar temiz (yunuslama -16..+19 deg). Statik marji ~%1 ve trim hizi yuksek (18 m/s).
AP_EXTRA = dict(toy=["PTCH_RATE_P 0.01", "PTCH_RATE_I 0.04", "PTCH_RATE_FF 0.09"])


def px4_param(name, key, default=None):
    af = next((REPO / "airframes").glob(f"*_gz_{name}"))
    m = re.search(rf"param set-default {key} ([-\d.]+)", af.read_text())
    return float(m.group(1)) if m else default


def write_params(name, sdf, funcs, tmax):
    """ArduPlane parametreleri: servo fonksiyonlari + PX4 airframe'deki hiz/limitler."""
    trim = px4_param(name, "FW_AIRSPD_TRIM", 16)
    vmin = px4_param(name, "FW_AIRSPD_MIN", 12)
    vmax = px4_param(name, "FW_AIRSPD_MAX", 25)
    rlim = px4_param(name, "FW_R_LIM", 40)
    thr_trim = px4_param(name, "FW_THR_TRIM", 0.3)
    lines = [f"# {name}: tools/gen_ardupilot.py ile uretildi. Elle duzenlemeyin.",
             "# Temel: ArduPilot Tools/autotest/default_params/gazebo-zephyr.parm benzeri SITL ayarlari.",
             "AHRS_EKF_TYPE 3", "ARSPD_TYPE 100", "ARSPD_USE 1",
             f"AIRSPEED_CRUISE {trim:.0f}", f"AIRSPEED_MIN {vmin:.0f}", f"AIRSPEED_MAX {vmax:.0f}",
             f"ROLL_LIMIT_DEG {rlim:.0f}", "PTCH_LIM_MAX_DEG 30", "PTCH_LIM_MIN_DEG -20",
             f"TRIM_THROTTLE {thr_trim * 100:.0f}", "THR_MAX 90",
             "TKOFF_ALT 25", "TKOFF_THR_MINSPD 0", "TKOFF_THR_MINACC 0", "TKOFF_ROTATE_SPD 0",
             "TKOFF_PLIM_SEC 2", "TKOFF_THR_MAX 100",
             "ARMING_CHECK 0", "RTL_AUTOLAND 0", "FS_SHORT_ACTN 0", "FS_LONG_ACTN 0", "THR_FAILSAFE 0",
             "SIM_GPS1_TYPE 1", "BATT_MONITOR 4",
             # INS'in kalibre gorunmesi icin kucuk ofsetler (ArduPilot gazebo-zephyr.parm ile ayni)
             "INS_ACCOFFS_X 0.001", "INS_ACCOFFS_Y 0.001", "INS_ACCOFFS_Z 0.001",
             "INS_ACCSCAL_X 1.001", "INS_ACCSCAL_Y 1.001", "INS_ACCSCAL_Z 1.001",
             "INS_ACC2OFFS_X 0.001", "INS_ACC2OFFS_Y 0.001", "INS_ACC2OFFS_Z 0.001",
             "INS_ACC2SCAL_X 1.001", "INS_ACC2SCAL_Y 1.001", "INS_ACC2SCAL_Z 1.001",
             "INS_GYR_CAL 0"]
    lines += AP_EXTRA.get(name, [])
    for ch, f in enumerate(funcs, start=1):
        lines += [f"SERVO{ch}_FUNCTION {f}", f"SERVO{ch}_MIN 1000", f"SERVO{ch}_MAX 2000",
                  f"SERVO{ch}_TRIM {1000 if f == 70 else 1500}", f"SERVO{ch}_REVERSED 0"]
    out = REPO / "ardupilot" / "params"
    out.mkdir(parents=True, exist_ok=True)
    (out / f"{name}.parm").write_text("\n".join(lines) + "\n")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("names", nargs="*")
    ap.add_argument("--all", action="store_true")
    a = ap.parse_args()
    names = (["toy"] + list(PLANES)) if a.all else a.names
    signs = json.loads((REPO / "tools" / "ardupilot_signs.json").read_text())
    for n in names:
        controls, funcs, tmax = convert(n, signs)
        print(f"{n}_ap: kanallar {funcs} (son kanal gaz), itki 0..{tmax:.0f} N")


if __name__ == "__main__":
    main()
