#!/usr/bin/env python3
"""tools/plane_specs.py'deki tanimdan bir ucagin tum simulasyon dosyalarini uretir.

    python3 tools/gen_plane.py talon [--avl ~/Avl/bin/avl]
    python3 tools/gen_plane.py --all

Uretilenler:
  models/<ad>/model.sdf, model.config, meshes/*.obj   Gazebo modeli
  models/<ad>/avl/<ad>.avl, *_st.txt, *_sb.txt        AVL girdisi ve ciktilari
  airframes/<id>_gz_<ad>                              PX4 airframe

Adimlar:
  1. Kanat/kuyruk/govde tanimindan mesh (kontrol yuzeyleri ayri parca) ve AVL geometrisi.
  2. AVL: notr nokta -> agirlik merkezi = Xnp - statik_marj * MAC.
  3. AVL: seyir CL'sinde Cm = 0 olacak sekilde kuyruk acisi (ucan kanatta uc bukumu) ayari.
  4. AVL: alpha = 0'da turevler -> AdvancedLiftDrag katsayilari (her kontrol yuzeyi ayri).
  5. PX4 airframe: kontrol tahsisi AVL turevlerinden, hizlar kanat yukunden, acisal hiz
     kazanclari ayarli toy modeline gore kontrol gucu oranindan.

Eksen kurallari: AVL x geriye / y saga / z yukari, burun orijin. Govde (SDF): x ileri / y sola /
z yukari, orijin agirlik merkezi. Donusum govde = (xcg - x, -y, z - zcg) (z ekseni etrafinda 180
derece, yani donme yonleri korunur): bir eklemin pozitif donusu AVL'deki pozitif kontrol sapmasi
ile ayni yondedir; plugin katsayilari AVL turevleri ile birebir (direction = 1).
"""
import argparse
import math
import re
import subprocess
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
from plane_specs import PLANES  # noqa: E402

REPO = Path(__file__).resolve().parent.parent
RHO, G = 1.2041, 9.81
SERVO_MAX_DEG = 25.0
ESC_MAX = 1000.0  # rad/s, SIM_GZ_EC_MAX

# Ayarlanmis referans: toy (airframes/22000_gz_toy). Kontrol gucu = sum|C_delta| (1/deg) * 25 deg
# * q_trim * S * L / I. Yeni ucagin kazanclari toy'un kazanclari * (P_toy / P_yeni).
TOY = dict(S=0.308, b=1.40, c=0.222, Ixx=0.15, Iyy=0.14, Izz=0.27, v_trim=18.0,
           cl=2 * 0.00272, cm=0.01115, cn=0.00113,
           gains=dict(RR=(0.05, 0.25, 0.09), PR=(0.35, 0.5, 0.40), YR=(0.25, 0.4, 0.4)))


# =============================================================== geometri
def naca4(code, n=14, f0=0.0, f1=1.0):
    """NACA 4 haneli profil: [f0, f1] kiris araliginda kapali halka (ust f0->f1, alt f1->f0)."""
    m, p, t = int(code[0]) / 100, int(code[1]) / 10, int(code[2:]) / 100
    beta = np.linspace(0, math.pi, n)
    x = f0 + (f1 - f0) * (1 - np.cos(beta)) / 2
    yt = 5 * t * (0.2969 * np.sqrt(x) - 0.1260 * x - 0.3516 * x**2 + 0.2843 * x**3 - 0.1036 * x**4)
    if m > 0:
        yc = np.where(x < p, m / p**2 * (2 * p * x - x**2), m / (1 - p)**2 * (1 - 2 * p + 2 * p * x - x**2))
    else:
        yc = np.zeros_like(x)
    upper = np.stack([x, yc + yt], 1)
    lower = np.stack([x, yc - yt], 1)[::-1]
    return np.vstack([upper, lower[1:-1]])


class Mesh:
    def __init__(self):
        self.tris = []  # (3,3) dizileri

    def add(self, tri, inside):
        a, b, c = tri
        n = np.cross(b - a, c - a)
        if np.dot(n, (a + b + c) / 3 - inside) < 0:
            tri = (a, c, b)
        self.tris.append(np.array(tri))

    def loft(self, rings, caps=True):
        rings = [np.asarray(r, float) for r in rings]
        for r0, r1 in zip(rings[:-1], rings[1:]):
            ctr = (r0.mean(0) + r1.mean(0)) / 2
            m = len(r0)
            for k in range(m):
                k2 = (k + 1) % m
                self.add((r0[k], r0[k2], r1[k2]), ctr)
                self.add((r0[k], r1[k2], r1[k]), ctr)
        if caps:
            for r, other in ((rings[0], rings[1]), (rings[-1], rings[-2])):
                c = r.mean(0)
                inside = c + (other.mean(0) - c) * 0.5
                for k in range(len(r)):
                    self.add((c, r[k], r[(k + 1) % len(r)]), inside)

    def write(self, path, to_body):
        lines = ["# tools/gen_plane.py ile uretildi\n"]
        verts, norms = [], []
        for t in self.tris:
            tb = np.array([to_body(p) for p in t])
            n = np.cross(tb[1] - tb[0], tb[2] - tb[0])
            ln = np.linalg.norm(n)
            norms.append(n / ln if ln > 1e-12 else np.array([0, 0, 1.0]))
            verts.extend(tb)
        lines += [f"v {v[0]:.4f} {v[1]:.4f} {v[2]:.4f}\n" for v in verts]
        lines += [f"vn {n[0]:.3f} {n[1]:.3f} {n[2]:.3f}\n" for n in norms]
        lines += [f"f {3*i+1}//{i+1} {3*i+2}//{i+1} {3*i+3}//{i+1}\n" for i in range(len(self.tris))]
        path.write_text("".join(lines))


def section_ring(le, chord, inc_deg, s, airfoil, f0=0.0, f1=1.0):
    """Kesit halkasi. le: hucum kenari noktasi, s: aciklik yonu (bilesen y artan yonde),
    inc_deg: hucum acisi (burun yukari +)."""
    i = math.radians(inc_deg)
    d = np.array([math.cos(i), 0.0, -math.sin(i)])  # LE -> TE
    s = np.asarray(s, float) / np.linalg.norm(s)
    n = np.cross(d, s)
    n /= np.linalg.norm(n)
    pts = naca4(airfoil, f0=f0, f1=f1)
    return np.array([le + x * chord * d + y * chord * n for x, y in pts])


class Surface:
    """Bir kaldirma yuzeyi: istasyonlar y artan yonde sirali; kontroller istasyon araliklarinda."""

    def __init__(self, name, airfoil, ydup=False):
        self.name, self.airfoil, self.ydup = name, airfoil, ydup
        self.stations = []  # dict(le, chord, inc, controls={ad: hinge_frac})

    def station(self, le, chord, inc=0.0, controls=None):
        self.stations.append(dict(le=np.asarray(le, float), chord=chord, inc=inc, controls=controls or {}))

    def span_dir(self, k):
        a, b = self.stations[k]["le"], self.stations[k + 1]["le"]
        return b - a

    def build_meshes(self, fixed, ctrl_meshes):
        """Sabit kisim fixed Mesh'ine, kontrol yuzeyleri ctrl_meshes[ad] Mesh'lerine."""
        for k in range(len(self.stations) - 1):
            a, b = self.stations[k], self.stations[k + 1]
            s = self.span_dir(k)
            common = set(a["controls"]) & set(b["controls"])
            if common:
                cname = sorted(common)[0]
                h = a["controls"][cname]
                fixed.loft([section_ring(st["le"], st["chord"], st["inc"], s, self.airfoil, 0, h)
                            for st in (a, b)])
                ctrl_meshes.setdefault(cname, Mesh()).loft(
                    [section_ring(st["le"], st["chord"], st["inc"], s, self.airfoil, h, 1) for st in (a, b)])
            else:
                fixed.loft([section_ring(st["le"], st["chord"], st["inc"], s, self.airfoil) for st in (a, b)])

    def hinge(self, cname):
        """(nokta, eksen) AVL cercevesinde: kontrolun ilk ve son istasyonundaki mentese noktalari."""
        pts = []
        for st in self.stations:
            if cname in st["controls"]:
                h = st["controls"][cname]
                i = math.radians(st["inc"])
                pts.append(st["le"] + h * st["chord"] * np.array([math.cos(i), 0, -math.sin(i)]))
        p0, p1 = pts[0], pts[-1]
        return (p0 + p1) / 2, (p1 - p0) / np.linalg.norm(p1 - p0)

    def avl(self, nchord=10):
        nspan = 8 * (len(self.stations) - 1)  # kesit araligi basina 8 girdap
        out = [f"#{'=' * 60}\nSURFACE\n{self.name}\n{nchord} 1.0 {nspan} 1.0\n"]
        if self.ydup:
            out.append("YDUPLICATE\n0.0\n")
        for st in self.stations:
            x, y, z = st["le"]
            out.append(f"SECTION\n{x:.4f} {y:.4f} {z:.4f} {st['chord']:.4f} {st['inc']:.3f}\n")
            out.append(f"NACA\n{self.airfoil}\n")
            for cname, h in st["controls"].items():
                out.append(f"CONTROL\n{cname} 1.0 {h:.3f} 0. 0. 0. 1\n")
        return "".join(out)


def build_geometry(p):
    """Ucak tanimindan yuzeyleri (AVL cercevesi) ve govde halkalarini kurar."""
    surfaces = []
    w = p["wing"]
    half = w["span"] / 2
    gam = math.radians(w["dihedral"])
    ctrl = w["control"]  # aileron | elevon
    cl, cr = ("ail_l", "ail_r") if ctrl == "aileron" else ("el_l", "el_r")
    hfrac = 1 - w["ctrl_chord"]

    def wing_station(eta):
        y = eta * half
        return dict(le=np.array([w["x_le"] + w["tip_le_offset"] * eta, y, w["z"] + y * math.tan(gam)]),
                    chord=w["root"] + (w["tip"] - w["root"]) * eta,
                    inc=w["incidence"] + w["twist"] * eta)

    wing = Surface("Kanat", w["airfoil"])
    e0, e1 = w["ctrl_span"]
    # sol uc -> kok -> sag uc; kontrol yalnizca e0..e1 araligindaki panellerde
    for eta, ctl in ((1.0, {}), (e1, {cl: hfrac}), (e0, {cl: hfrac})):
        st = wing_station(eta)
        wing.station(st["le"] * np.array([1, -1, 1]), st["chord"], st["inc"], ctl)
    st = wing_station(0.0)
    wing.station(st["le"], st["chord"], st["inc"])
    for eta, ctl in ((e0, {cr: hfrac}), (e1, {cr: hfrac}), (1.0, {})):
        st = wing_station(eta)
        wing.station(st["le"], st["chord"], st["inc"], ctl)
    surfaces.append(wing)

    if "winglet" in p:
        wl = p["winglet"]
        tip = wing_station(1.0)
        for sgn in (-1, 1):
            s = Surface(f"Winglet{'L' if sgn < 0 else 'R'}", wl["airfoil"])
            base = tip["le"] * np.array([1, sgn, 1]) + np.array([tip["chord"] - wl["root"], 0, 0])
            s.station(base, wl["root"])
            s.station(base + np.array([wl["tip_le_offset"], 0, wl["height"]]), wl["tip"])
            surfaces.append(s)

    if "vtail" in p:
        v = p["vtail"]
        dl = math.radians(v["dihedral"])
        hf = 1 - v["ctrl_chord"]
        s = Surface("V-Kuyruk", v["airfoil"])
        tipL = np.array([v["x_le"] + v["tip_le_offset"], -v["span"] * math.cos(dl), v["z"] + v["span"] * math.sin(dl)])
        root = np.array([v["x_le"], 0.0, v["z"]])
        tipR = tipL * np.array([1, -1, 1])
        inc = v.get("incidence", 0.0)
        s.station(tipL, v["tip"], inc, {"rv_l": hf})
        s.station(root, v["root"], inc, {"rv_l": hf, "rv_r": hf})
        s.station(tipR, v["tip"], inc, {"rv_r": hf})
        surfaces.append(s)

    if "htail" in p:
        h = p["htail"]
        hf = 1 - h["ctrl_chord"]
        inc = h.get("incidence", 0.0)
        s = Surface("Yatay Kuyruk", h["airfoil"])
        for y in (-h["span"] / 2, 0.0, h["span"] / 2):
            eta = abs(y) / (h["span"] / 2)
            s.station([h["x_le"] + h["tip_le_offset"] * eta, y, h["z"]],
                      h["root"] + (h["tip"] - h["root"]) * eta, inc, {"elevator": hf})
        surfaces.append(s)

    if "fin" in p:
        f = p["fin"]
        hf = 1 - f["ctrl_chord"]
        s = Surface("Dikey Kuyruk", f["airfoil"])
        s.station([f["x_le"], 0, f["z"]], f["root"], 0, {"rudder": hf})
        s.station([f["x_le"] + f["tip_le_offset"], 0, f["z"] + f["height"]], f["tip"], 0, {"rudder": hf})
        surfaces.append(s)

    fus_rings = []
    for x, wdt, hgt, zc in p["fuselage"]:
        th = np.linspace(0, 2 * math.pi, 16, endpoint=False)
        # super-elips kesit (kosesi yumusak kutu)
        cy = np.sign(np.cos(th)) * np.abs(np.cos(th)) ** 0.6 * wdt / 2
        cz = np.sign(np.sin(th)) * np.abs(np.sin(th)) ** 0.6 * hgt / 2
        fus_rings.append(np.stack([np.full_like(th, x), cy, zc + cz], 1))
    return surfaces, fus_rings


def tail_incidence_key(p):
    if "vtail" in p:
        return ("vtail", "incidence")
    if "htail" in p:
        return ("htail", "incidence")
    return ("wing", "twist")  # ucan kanat: uc bukumu ile trim


# =============================================================== AVL
def write_avl(p, name, surfaces, xref, path, cdp):
    w = p["wing"]
    S = w["span"] * (w["root"] + w["tip"]) / 2
    lam = w["tip"] / w["root"]
    mac = 2 / 3 * w["root"] * (1 + lam + lam**2) / (1 + lam)
    txt = (f"{p['title']} (tools/gen_plane.py ile uretildi)\n0.0\n0 0 0.0\n"
           f"{S:.4f} {mac:.4f} {w['span']:.4f}\n{xref:.4f} 0.0 0.0\n{cdp:.4f}\n")
    for s in surfaces:
        txt += s.avl()
    path.write_text(txt)
    return S, mac


NUM = r"[-+]?(?:\d+\.\d*|\.\d+|\d+)(?:[EeDd][-+]?\d+)?"
PAIR = re.compile(r"([A-Za-z][A-Za-z0-9']*)\s*=\s*(" + NUM + ")")


def parse(text):
    vals = {}
    for line in text.splitlines():
        for k, v in PAIR.findall(line):
            vals.setdefault(k, float(v))
    return vals


def run_avl(avl_bin, avl_file, commands, cwd):
    script = f"load {avl_file.name}\noper\n{commands}\n\nquit\n"
    r = subprocess.run([avl_bin], input=script, capture_output=True, text=True, cwd=cwd, timeout=60)
    return r.stdout


def avl_point(avl_bin, avl_file, cwd, alpha=None, cl=None, tag="pt"):
    st = cwd / f"{tag}_st.txt"
    sb = cwd / f"{tag}_sb.txt"
    for f in (st, sb):
        f.unlink(missing_ok=True)
    set_a = f"a a {alpha}" if alpha is not None else f"a c {cl}"
    run_avl(avl_bin, avl_file, f"{set_a}\nx\nst {st.name}\nsb {sb.name}", cwd)
    if not st.exists():
        raise RuntimeError(f"AVL cikti uretmedi ({avl_file})")
    return parse(st.read_text()), parse(sb.read_text())


# =============================================================== kutle / atalet
def inertia(p, xcg, zcg, S):
    m = p["mass"]
    w = p["wing"]
    L = max(x for x, *_ in p["fuselage"])
    mac_x = w["x_le"] + w["tip_le_offset"] * 0.45 + 0.25 * (w["root"] + w["tip"]) / 2
    flying = p.get("flying_wing", False)
    parts = [  # (kutle, x, z, Ixx_kendi, Iyy_kendi)
        (m * (0.40 if flying else 0.25), mac_x, w["z"], m * (0.40 if flying else 0.25) * w["span"]**2 / 12,
         m * (0.40 if flying else 0.25) * ((w["root"] + w["tip"]) / 2)**2 / 12),
        (m * (0.30 if flying else 0.30), L * 0.40, 0.0, 0.0, m * 0.30 * (L * 0.8)**2 / 12),
        (m * 0.10, p["motor"]["x"], p["motor"]["z"], 0.0, 0.0),
    ]
    if not flying:
        tail_x = p.get("vtail", p.get("htail", {})).get("x_le", L * 0.9)
        parts.append((m * 0.06, tail_x, 0.05, 0.0, 0.0))
    used = sum(q[0] for q in parts)
    parts.append((m - used, xcg, 0.0, 0.0, 0.0))  # batarya + aviyonik: CG'yi tutturacak yerde
    Ixx = sum(q[3] + q[0] * (q[2] - zcg)**2 for q in parts) + 0.02 * m * w["span"]**2 / 12
    Iyy = sum(q[4] + q[0] * ((q[1] - xcg)**2 + (q[2] - zcg)**2) for q in parts)
    Izz = Ixx + Iyy * 0.9
    return Ixx, Iyy, Izz


# =============================================================== uretim
def generate(name, avl_bin):
    p = PLANES[name]
    out = REPO / "models" / name
    (out / "meshes").mkdir(parents=True, exist_ok=True)
    avl_dir = out / "avl"
    avl_dir.mkdir(exist_ok=True)
    w = p["wing"]
    flying = p.get("flying_wing", False)
    cdp = 0.030 if not flying else 0.022
    clmax = 0.95 if flying else 1.2
    avl_file = avl_dir / f"{name}.avl"

    # 1) notr nokta
    surfaces, fus = build_geometry(p)
    S, mac = write_avl(p, name, surfaces, 0.3, avl_file, cdp)
    st, _ = avl_point(avl_bin, avl_file, avl_dir, alpha=0, tag="np")
    xnp = st["Xnp"]
    xcg = xnp - p["static_margin"] * mac
    zcg = 0.3 * w["z"]

    # 2) trim: seyir CL'sinde Cm = 0 (secant). Stall hizi, Gazebo modelinin kendi CLmax'indan
    #    (CL0 + CLa * alpha_stall) hesaplanir ki airframe hizlari modelle tutarli olsun. Trim
    #    acisi CL0'i, CL0 da stall hizini degistirdigi icin ikisi birlikte yinelenir.
    alpha_stall = math.radians(13.0 - w["incidence"])
    clmax_cfg = clmax
    grp, key = tail_incidence_key(p)

    def cm_at(val, cl_target):
        p[grp][key] = val
        surfs, _ = build_geometry(p)
        write_avl(p, name, surfs, xcg, avl_file, cdp)
        s, _ = avl_point(avl_bin, avl_file, avl_dir, cl=cl_target, tag="trim")
        return s["Cmtot"], s["Alpha"]

    v_trim = None
    for _outer in range(4):
        surfs, _ = build_geometry(p)
        write_avl(p, name, surfs, xcg, avl_file, cdp)
        st0, _ = avl_point(avl_bin, avl_file, avl_dir, alpha=0, tag="np")
        clmax = min(clmax_cfg, st0["CLtot"] + st0["CLa"] * alpha_stall)
        v_stall = math.sqrt(2 * p["mass"] * G / (RHO * S * clmax))
        new_trim = round(p.get("trim_factor", 1.7) * v_stall)
        if new_trim == v_trim:
            break
        v_trim = new_trim
        cl_cruise = 2 * p["mass"] * G / (RHO * v_trim**2 * S)
        x0 = p[grp].get(key, 0.0)
        x1 = x0 - 1.0
        f0, _ = cm_at(x0, cl_cruise)
        f1, a1 = cm_at(x1, cl_cruise)
        for _ in range(6):
            if abs(f1) < 2e-4 or f1 == f0:
                break
            x0, x1, f0 = x1, x1 - f1 * (x1 - x0) / (f1 - f0), f1
            x1 = max(-8.0, min(8.0, x1))
            f1, a1 = cm_at(x1, cl_cruise)
    trim_alpha = a1

    # 3) son geometri, mesh, turevler
    surfaces, fus = build_geometry(p)
    S, mac = write_avl(p, name, surfaces, xcg, avl_file, cdp)
    st, sb = avl_point(avl_bin, avl_file, avl_dir, alpha=0, tag=name)
    for f in avl_dir.glob("np_*"):
        f.unlink()
    for f in avl_dir.glob("trim_*"):
        f.unlink()

    def to_body(pt):
        return np.array([xcg - pt[0], -pt[1], pt[2] - zcg])

    fixed = Mesh()
    ctrl_meshes = {}
    for s in surfaces:
        s.build_meshes(fixed, ctrl_meshes)
    fusm = Mesh()
    fusm.loft(fus)
    for f in (out / "meshes").glob("*.obj"):
        f.unlink()
    fixed.write(out / "meshes" / "surfaces.obj", to_body)
    fusm.write(out / "meshes" / "fuselage.obj", to_body)

    # kontrol listesi (AVL siralamasi: dosyada ilk gorulme sirasi)
    order = []
    for s in surfaces:
        for stn in s.stations:
            for c in stn["controls"]:
                if c not in order:
                    order.append(c)
    px4_type = dict(ail_l=1, ail_r=2, elevator=3, rudder=4, el_l=5, el_r=6, rv_l=7, rv_r=8)
    # her yuzey tipinin kontrol tahsisinde kullanilan (asil) eksenleri
    primary = dict(ail_l="Cl", ail_r="Cl", elevator="Cm", rudder="Cn", el_l="ClCm", el_r="ClCm",
                   rv_l="CmCn", rv_r="CmCn")
    controls = []
    for k, cname in enumerate(order):
        surf = next(s for s in surfaces if any(cname in stn["controls"] for stn in s.stations))
        hp, ax = surf.hinge(cname)
        d = {kk: st.get(kk + f"d{k + 1}", 0.0) for kk in ("CL", "CY", "Cl", "Cm", "Cn")}
        d["CD"] = st.get(f"CDffd{k + 1}", 0.0)
        ctrl_meshes[cname].write(out / "meshes" / f"{cname}.obj", to_body)
        controls.append(dict(name=cname, type=px4_type[cname], primary=primary[cname], hinge=to_body(hp),
                             axis=np.array([-ax[0], -ax[1], ax[2]]), d=d))

    # 4) kutle, motor, katsayilar
    Ixx, Iyy, Izz = p["inertia"] if "inertia" in p else inertia(p, xcg, zcg, S)
    AR = w["span"]**2 / S
    e = min(max(st.get("e", 0.85), 0.5), 1.0)
    CL0 = st["CLtot"]
    aero = dict(CL0=CL0, CLa=st["CLa"], AR=AR, eff=e, CD0=st["CDtot"] - CL0**2 / (math.pi * AR * e),
                Cem0=st["Cmtot"], Cema=st["Cma"], CYb=st["CYb"], Cellb=st["Clb"], Cenb=st["Cnb"],
                CYp=st["CYp"], CLp=st["CLp"], Cellp=st["Clp"], Cemp=st["Cmp"], Cenp=st["Cnp"],
                CYq=st["CYq"], CLq=st["CLq"], Cellq=st["Clq"], Cemq=st["Cmq"], Cenq=st["Cnq"],
                CYr=st["CYr"], CLr=st["CLr"], Cellr=st["Clr"], Cemr=st["Cmr"], Cenr=st["Cnr"],
                CDp=-sb.get("CXp", 0.0), CDq=-sb.get("CXq", 0.0), CDr=-sb.get("CXr", 0.0))
    mot = p["motor"]
    kmot = mot["thrust"] / ESC_MAX**2

    # hizlar, gaz
    v_min = round(p.get("min_factor", 1.25) * v_stall)
    v_max = round(min(max(2.3 * v_stall, v_trim + 5), 30))
    q = 0.5 * RHO * v_trim**2
    drag = q * S * (aero["CD0"] + cl_cruise**2 / (math.pi * AR * e))
    thr_trim = min(0.8, max(0.15, math.sqrt(drag / kmot) / ESC_MAX))

    # 5) kazanclar: eksen basina kontrol gucu
    def power(axis, ref_len, I, v):
        tot = sum(abs(c["d"][axis]) for c in controls if axis in c["primary"])
        return tot * SERVO_MAX_DEG * 0.5 * RHO * v**2 * S * ref_len / I

    qt = TOY
    P_toy = dict(R=qt["cl"] * 25 * 0.5 * RHO * qt["v_trim"]**2 * qt["S"] * qt["b"] / qt["Ixx"],
                 P=qt["cm"] * 25 * 0.5 * RHO * qt["v_trim"]**2 * qt["S"] * qt["c"] / qt["Iyy"],
                 Y=qt["cn"] * 25 * 0.5 * RHO * qt["v_trim"]**2 * qt["S"] * qt["b"] / qt["Izz"])
    P_new = dict(R=power("Cl", w["span"], Ixx, v_trim), P=power("Cm", mac, Iyy, v_trim),
                 Y=power("Cn", w["span"], Izz, v_trim))
    scale = {a: min(3.0, max(0.33, P_toy[a] / P_new[a])) if P_new[a] > 1e-9 else 1.0 for a in "RPY"}
    gains = {}
    for key2, axis in (("RR", "R"), ("PR", "P"), ("YR", "Y")):
        P_, I_, FF_ = qt["gains"][key2]
        k = scale[axis]
        gains[key2] = (P_ * k, I_ * min(k, 1.5), FF_ * k)

    # PX4 kontrol tahsisi: eksen basina toplam |etki| = 1 olacak sekilde normalize
    norm = {}
    for axis in ("Cl", "Cm", "Cn"):
        tot = sum(abs(c["d"][axis]) for c in controls if axis in c["primary"])
        norm[axis] = 1 / tot if tot > 1e-9 else 0.0
    for c in controls:
        trq = {}
        for axis, pk in (("Cl", "R"), ("Cm", "P"), ("Cn", "Y")):
            trq[pk] = c["d"][axis] * norm[axis] if axis in c["primary"] else 0.0
        c["trq"] = trq

    info = dict(S=S, mac=mac, AR=AR, xnp=xnp, xcg=xcg, zcg=zcg, I=(Ixx, Iyy, Izz), v_stall=v_stall,
                v_min=v_min, v_trim=v_trim, v_max=v_max, thr_trim=thr_trim, cl_cruise=cl_cruise,
                trim_alpha=trim_alpha, trim_key=f"{grp}.{key}={p[grp][key]:.2f} deg", scale=scale,
                gains=gains, kmot=kmot, alpha_stall=alpha_stall)
    write_sdf(name, p, out, aero, controls, info)
    write_airframe(name, p, controls, info)
    return info


def fmt(v):
    return f"{v:.6g}"


def write_sdf(name, p, out, aero, controls, info):
    c = p["colors"]
    Ixx, Iyy, Izz = info["I"]
    mot = p["motor"]
    xcg, zcg = info["xcg"], info["zcg"]
    mpos = np.array([xcg - mot["x"], 0.0, mot["z"] - zcg])
    fus = p["fuselage"]
    xs = [f[0] for f in fus]
    fx0, fx1 = min(xs), max(xs)
    fw = max(f[1] for f in fus)
    fh = max(f[2] for f in fus)
    box_c = np.array([xcg - (fx0 + fx1) / 2, 0.0, -zcg])
    bottom = -zcg - fh / 2

    def material(rgb):
        r, g, b = rgb
        return (f"<material><ambient>{r*0.6:.3f} {g*0.6:.3f} {b*0.6:.3f} 1</ambient>"
                f"<diffuse>{r} {g} {b} 1</diffuse><specular>0.2 0.2 0.2 1</specular></material>")

    def mesh_visual(vname, f, rgb, pose="0 0 0 0 0 0"):
        return (f"      <visual name='{vname}'><pose>{pose}</pose><geometry><mesh>"
                f"<uri>model://{name}/meshes/{f}</uri></mesh></geometry>{material(rgb)}</visual>\n")

    skid_xml = ""
    if p.get("nose_skid"):
        # burun kizagi: govde altindan nose_skid kadar asagi uzanir -> yerde burun yukari oturus
        sk = p["nose_skid"]
        sx = xcg - (fx0 + 0.12 * (fx1 - fx0))
        skid_xml = (f"      <collision name='nose_skid'><pose>{sx:.3f} 0 {bottom - sk / 2 + 0.01:.3f} 0 0 0</pose>"
                    f"<geometry><box><size>0.04 0.04 {sk + 0.02:.3f}</size></box></geometry>"
                    "<surface><friction><ode><mu>0.1</mu><mu2>0.1</mu2></ode></friction></surface></collision>\n"
                    f"      <visual name='nose_skid_visual'><pose>{sx:.3f} 0 {bottom - sk / 2 + 0.01:.3f} 0 0 0</pose>"
                    f"<geometry><box><size>0.04 0.04 {sk + 0.02:.3f}</size></box></geometry>{material((0.1, 0.1, 0.1))}</visual>\n")
        bottom -= sk

    links = ""
    joints = ""
    plugins = ""
    surf_xml = ""
    for k, ctl in enumerate(controls):
        hx, hy, hz = ctl["hinge"]
        ax = ctl["axis"] / np.linalg.norm(ctl["axis"])
        links += (f"    <link name='{ctl['name']}'><pose>{hx:.4f} {hy:.4f} {hz:.4f} 0 0 0</pose>\n"
                  "      <inertial><mass>0.00000001</mass><inertia><ixx>0.000001</ixx><ixy>0</ixy><ixz>0</ixz>"
                  "<iyy>0.000001</iyy><iyz>0</iyz><izz>0.000001</izz></inertia></inertial>\n"
                  + mesh_visual(f"{ctl['name']}_visual", f"{ctl['name']}.obj", c["ctrl"],
                                f"{-hx:.4f} {-hy:.4f} {-hz:.4f} 0 0 0")
                  + "    </link>\n")
        joints += (f"    <joint name='servo_{k}' type='revolute'><parent>base_link</parent><child>{ctl['name']}</child>\n"
                   f"      <axis><xyz expressed_in='__model__'>{ax[0]:.4f} {ax[1]:.4f} {ax[2]:.4f}</xyz>"
                   f"<limit><lower>-0.44</lower><upper>0.44</upper></limit><dynamics><damping>1.0</damping></dynamics></axis>\n"
                   "      <physics><ode><implicit_spring_damper>1</implicit_spring_damper></ode></physics></joint>\n")
        plugins += ("    <plugin filename='gz-sim-joint-position-controller-system' name='gz::sim::systems::JointPositionController'>"
                    f"<joint_name>servo_{k}</joint_name><sub_topic>servo_{k}</sub_topic><p_gain>10</p_gain><i_gain>0</i_gain><d_gain>0</d_gain></plugin>\n")
        d = ctl["d"]
        surf_xml += (f"      <control_surface><name>servo_{k}</name><index>{k}</index><direction>1</direction>"
                     f"<CD_ctrl>{fmt(d['CD'])}</CD_ctrl><CY_ctrl>{fmt(d['CY'])}</CY_ctrl><CL_ctrl>{fmt(d['CL'])}</CL_ctrl>"
                     f"<Cell_ctrl>{fmt(d['Cl'])}</Cell_ctrl><Cem_ctrl>{fmt(d['Cm'])}</Cem_ctrl><Cen_ctrl>{fmt(d['Cn'])}</Cen_ctrl>"
                     f"</control_surface>\n")

    aero_xml = "".join(f"      <{k}>{fmt(v)}</{k}>\n" for k, v in aero.items())
    sensors = (REPO / "models" / "toy" / "model.sdf").read_text()
    s0 = sensors.index('      <sensor name="imu_sensor"')
    s1 = sensors.index("    </link>", s0)
    sensor_xml = sensors[s0:s1]
    cam_x = xcg - fx0 + 0.01

    sdf = f"""<?xml version="1.0"?>
<!-- {p['title']}: tools/gen_plane.py ile uretildi (tanim: tools/plane_specs.py). Elle duzenlemeyin.
     S={info['S']:.3f} m^2, MAC={info['mac']:.3f} m, AR={info['AR']:.2f}, kutle={p['mass']} kg
     Notr nokta burundan {info['xnp']:.3f} m, agirlik merkezi {info['xcg']:.3f} m (statik marj %{p['static_margin']*100:.0f} MAC)
     Trim: {info['trim_key']}, seyir CL={info['cl_cruise']:.2f} @ {info['v_trim']} m/s, alpha={info['trim_alpha']:.1f} deg -->
<sdf version="1.9">
  <model name="{name}">
    <pose>0 0 {-bottom + 0.03:.3f} 0 0 0</pose>
    <link name="base_link">
      <inertial><mass>{p['mass']}</mass><inertia><ixx>{Ixx:.4f}</ixx><ixy>0</ixy><ixz>0</ixz>
        <iyy>{Iyy:.4f}</iyy><iyz>0</iyz><izz>{Izz:.4f}</izz></inertia></inertial>
      <collision name="base_link_collision"><pose>{box_c[0]:.3f} 0 {box_c[2]:.3f} 0 0 0</pose>
        <geometry><box><size>{fx1 - fx0:.3f} {fw:.3f} {fh:.3f}</size></box></geometry>
        <surface><contact><ode><max_vel>10</max_vel><min_depth>0.01</min_depth></ode></contact>
          <friction><ode><mu>0.1</mu><mu2>0.1</mu2></ode></friction></surface></collision>
{skid_xml}{mesh_visual('fuselage_visual', 'fuselage.obj', c['fuselage'])}{mesh_visual('surfaces_visual', 'surfaces.obj', c['wing'])}      <gravity>1</gravity>
{sensor_xml}    </link>
    <link name="rotor"><pose>{mpos[0]:.3f} 0 {mpos[2]:.3f} 0 1.5708 0</pose>
      <inertial><mass>0.02</mass><inertia><ixx>1.5e-04</ixx><ixy>0</ixy><ixz>0</ixz><iyy>1.5e-04</iyy><iyz>0</iyz><izz>3e-04</izz></inertia></inertial>
      <visual name="rotor_visual"><geometry><cylinder><length>0.004</length><radius>{mot['prop'] / 2:.3f}</radius></cylinder></geometry>
        <material><ambient>0.1 0.1 0.1 0.6</ambient><diffuse>0.1 0.1 0.1 0.6</diffuse></material><transparency>0.4</transparency></visual>
    </link>
    <joint name="rotor_joint" type="revolute"><child>rotor</child><parent>base_link</parent>
      <axis><xyz expressed_in="__model__">1 0 0</xyz><limit><lower>-1e+16</lower><upper>1e+16</upper></limit></axis></joint>
{links}{joints}{plugins}    <plugin filename="libAirSpeedPlugin.so" name="px4::AirSpeed"><link_name>base_link</link_name></plugin>
    <plugin filename="gz-sim-advanced-lift-drag-system" name="gz::sim::systems::AdvancedLiftDrag">
{aero_xml}      <alpha_stall>{info['alpha_stall']:.3f}</alpha_stall>
      <CLa_stall>-3.85</CLa_stall><CDa_stall>-0.92</CDa_stall><Cema_stall>0</Cema_stall>
      <cp>0 0 0</cp><area>{info['S']:.4f}</area><mac>{info['mac']:.4f}</mac><air_density>{RHO}</air_density>
      <forward>1 0 0</forward><upward>0 0 1</upward><link_name>base_link</link_name>
      <num_ctrl_surfaces>{len(controls)}</num_ctrl_surfaces>
{surf_xml}    </plugin>
    <plugin filename="gz-sim-multicopter-motor-model-system" name="gz::sim::systems::MulticopterMotorModel">
      <jointName>rotor_joint</jointName><linkName>rotor</linkName><turningDirection>cw</turningDirection>
      <timeConstantUp>0.03</timeConstantUp><timeConstantDown>0.05</timeConstantDown>
      <maxRotVelocity>1100</maxRotVelocity><motorConstant>{info['kmot']:.4e}</motorConstant>
      <momentConstant>0.02</momentConstant><commandSubTopic>command/motor_speed</commandSubTopic>
      <motorNumber>0</motorNumber><rotorDragCoefficient>8.06428e-05</rotorDragCoefficient>
      <rollingMomentCoefficient>1e-06</rollingMomentCoefficient><rotorVelocitySlowdownSim>10</rotorVelocitySlowdownSim>
      <motorType>velocity</motorType>
    </plugin>
    <link name="camera_link"><pose>{cam_x:.3f} 0 {-zcg:.3f} 0 0.26 0</pose>
      <inertial><mass>0.03</mass><inertia><ixx>0.00001</ixx><ixy>0</ixy><ixz>0</ixz><iyy>0.00001</iyy><iyz>0</iyz><izz>0.00001</izz></inertia></inertial>
      <sensor name="camera" type="camera"><gz_frame_id>camera_link</gz_frame_id>
        <camera><horizontal_fov>1.5708</horizontal_fov><image><width>640</width><height>480</height></image>
          <clip><near>0.1</near><far>1500</far></clip></camera>
        <always_on>1</always_on><update_rate>20</update_rate><visualize>false</visualize></sensor>
    </link>
    <joint name="CameraJoint" type="fixed"><parent>base_link</parent><child>camera_link</child></joint>
  </model>
</sdf>
"""
    (out / "model.sdf").write_text(sdf)
    (out / "model.config").write_text(f"""<?xml version="1.0"?>
<model>
  <name>{p['title']}</name>
  <version>1.0</version>
  <sdf version="1.9">model.sdf</sdf>
  <description>{p['title']} (tools/gen_plane.py ile uretildi; olculer tools/plane_specs.py).</description>
</model>
""")


def write_airframe(name, p, controls, info):
    g = info["gains"]
    lines = [f"""#!/bin/sh
#
# @name {p['title']} SITL
#
# @type Standard Plane
# @class Plane
#
# tools/gen_plane.py ile uretildi (tanim: tools/plane_specs.py). Elle duzenlemeyin.
# Hiz ve kazanclar hesaplanmistir; gercek ucak icin baslangic degerleridir.
#

. ${{R}}etc/init.d/rc.fw_defaults

PX4_SIMULATOR=${{PX4_SIMULATOR:=gz}}
PX4_GZ_WORLD=${{PX4_GZ_WORLD:=toy_parkur}}
PX4_SIM_MODEL=${{PX4_SIM_MODEL:={name}}}
PX4_GZ_FOLLOW_OFFSET_X=${{PX4_GZ_FOLLOW_OFFSET_X:=-1.8}}
PX4_GZ_FOLLOW_OFFSET_Y=${{PX4_GZ_FOLLOW_OFFSET_Y:=-1.4}}
PX4_GZ_FOLLOW_OFFSET_Z=${{PX4_GZ_FOLLOW_OFFSET_Z:=0.8}}

param set-default SIM_GZ_EN 1

# Hizlar (m/s): kanat yuku {p['mass'] / info['S']:.1f} kg/m^2, stall ~{info['v_stall']:.1f} m/s
param set-default FW_AIRSPD_STALL {max(1, round(info['v_stall']))}
param set-default FW_AIRSPD_MIN {info['v_min']}
param set-default FW_AIRSPD_TRIM {info['v_trim']}
param set-default FW_AIRSPD_MAX {info['v_max']}

# Gaz: tahmini (suruklenme / motor modeli)
param set-default FW_THR_MIN 0.05
param set-default FW_THR_TRIM {info['thr_trim']:.2f}
param set-default FW_THR_MAX 0.9

# Acisal hiz kazanclari: toy'un kazanclari * kontrol gucu orani
#   (yatis x{info['scale']['R']:.2f}, yunuslama x{info['scale']['P']:.2f}, yon x{info['scale']['Y']:.2f})
param set-default FW_RR_P {g['RR'][0]:.3f}
param set-default FW_RR_I {g['RR'][1]:.3f}
param set-default FW_RR_FF {g['RR'][2]:.3f}
param set-default FW_PR_P {g['PR'][0]:.3f}
param set-default FW_PR_I {g['PR'][1]:.3f}
param set-default FW_PR_FF {g['PR'][2]:.3f}
param set-default FW_YR_P {g['YR'][0]:.3f}
param set-default FW_YR_I {g['YR'][1]:.3f}
param set-default FW_YR_FF {g['YR'][2]:.3f}
param set-default FW_R_LIM {p.get('roll_limit', 40)}
param set-default FW_P_LIM_MAX 30
param set-default FW_P_LIM_MIN -15
param set-default FW_T_CLMB_MAX 5
param set-default FW_T_SINK_MAX 3
param set-default FW_T_SINK_MIN 2
param set-default FW_W_EN 1
param set-default NAV_DLL_ACT 2
param set-default RWTO_TKOFF 1

# Motor: itki ekseni +x
param set-default CA_ROTOR_COUNT 1
param set-default CA_ROTOR0_PX {info['xcg'] - p['motor']['x']:.3f}
param set-default SIM_GZ_EC_FUNC1 101
param set-default SIM_GZ_EC_MIN1 0
param set-default SIM_GZ_EC_MAX1 {int(ESC_MAX)}

# Kontrol yuzeyleri: tork katsayilari AVL turevlerinden (pozitif cikis = AVL pozitif sapmasi)
param set-default CA_SV_CS_COUNT {len(controls)}
"""]
    for k, ctl in enumerate(controls):
        lines.append(f"param set-default CA_SV_CS{k}_TYPE {ctl['type']}\n")
        for ax in "RPY":
            if ctl["trq"][ax]:
                lines.append(f"param set-default CA_SV_CS{k}_TRQ_{ax} {ctl['trq'][ax]:.3f}\n")
        lines.append(f"param set-default SIM_GZ_SV_FUNC{k + 1} {201 + k}\n")
        lines.append(f"param set-default SIM_GZ_SV_MAXA{k + 1} {int(SERVO_MAX_DEG)}\n")
        lines.append(f"param set-default SIM_GZ_SV_MINA{k + 1} {-int(SERVO_MAX_DEG)}\n")
    path = REPO / "airframes" / f"{p['autostart']}_gz_{name}"
    path.write_text("".join(lines))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("names", nargs="*")
    ap.add_argument("--all", action="store_true")
    ap.add_argument("--avl", default=str(Path.home() / "Avl" / "bin" / "avl"))
    a = ap.parse_args()
    names = list(PLANES) if a.all else a.names
    if not names:
        ap.error("model adi verin ya da --all")
    for n in names:
        info = generate(n, a.avl)
        print(f"{n}: S={info['S']:.3f} MAC={info['mac']:.3f} AR={info['AR']:.1f} | "
              f"Xnp={info['xnp']:.3f} Xcg={info['xcg']:.3f} | trim {info['trim_key']} "
              f"alpha={info['trim_alpha']:.1f} | V stall/min/trim/max = {info['v_stall']:.1f}/"
              f"{info['v_min']}/{info['v_trim']}/{info['v_max']} | gaz {info['thr_trim']:.2f} | "
              f"kazanc olcegi R{info['scale']['R']:.2f} P{info['scale']['P']:.2f} Y{info['scale']['Y']:.2f}")


if __name__ == "__main__":
    main()
