#!/usr/bin/env python3
"""TOY gorev parkuru dunyasini uretir (internet gerektirmez).

Uretilenler:
  models/toy_ground/   cim zemini + pist (OBJ mesh, prosedurel PNG dokular)
  worlds/toy_parkur.sdf

Parkuru degistirmek icin asagidaki YERLESIM bolumunu duzenleyip tekrar calistirin:
  python3 tools/gen_world.py

Koordinatlar Gazebo dunya cercevesinde: x = dogu, y = kuzey, metre. Ucak (0, 0)'da,
doguya (+x) bakarak dogar; pist x ekseni boyuncadir.
Konum (enlem/boylam) dunya dosyasindaki ornek degerdir; calistirirken
PX4_HOME_LAT / PX4_HOME_LON / PX4_HOME_ALT ile degistirilebilir.
"""
import math
import random
from pathlib import Path

import numpy as np
from PIL import Image, ImageDraw, ImageFilter

REPO = Path(__file__).resolve().parent.parent
GROUND = REPO / "models" / "toy_ground"
WORLD = REPO / "worlds" / "toy_parkur.sdf"

# ----------------------------------------------------------------- YERLESIM
HOME = dict(lat=39.80, lon=32.60, alt=900.0)  # ornek konum (Ankara civari)
GROUND_SIZE = 1600.0                           # m, kare zemin
GRASS_TILE = 40.0                              # m, cim dokusunun tekrar boyu
RUNWAY = dict(x0=-40.0, x1=160.0, width=18.0)  # pist (x araligi, genislik)

PYLONS = [(0, 150), (300, 150), (300, 300), (0, 300)]          # yaris direkleri
GATES = [(150, 150, 0.0), (150, 300, 0.0)]                      # (x, y, yon rad): kanat ekseni y boyunca
GATE_WIDTH, GATE_HEIGHT = 16.0, 12.0
TARGETS = [(80, 225, "red"), (150, 225, "blue"), (220, 225, "yellow")]  # yer hedefleri
LANDING_ZONE = (230, -70)
HANGAR = (25, -32)
TOWER = (60, -38)
WINDSOCK = (-30, 22)
TREE_COUNT = 160
TREE_SEED = 7
# Agac konmayacak alanlar: (xmin, xmax, ymin, ymax)
KEEP_CLEAR = [(-160, 320, -60, 60),     # pist ve kalkis/inis koridoru
              (-60, 360, 90, 360),      # parkur
              (0, 80, -60, -20),        # hangar/kule
              (200, 260, -100, -40)]    # inis alani
# --------------------------------------------------------------------------

rng = np.random.default_rng(TREE_SEED)


def grass_texture(path, n=1024):
    """Hafif cim deseni + bicme seritleri (tekrar eden doku)."""
    base = np.zeros((n, n, 3), np.float32)
    # cok olcekli gurultu (periyodik oldugu icin dikissiz tekrar eder)
    for scale, amp in [(4, 0.10), (16, 0.06), (64, 0.05), (256, 0.04)]:
        g = rng.random((scale, scale)).astype(np.float32)
        # 3x3 dose, buyut, ortayi kes -> kenarlari birbirine uyan (dikissiz) gurultu
        big = Image.fromarray((np.tile(g, (3, 3)) * 255).astype(np.uint8)).resize((3 * n, 3 * n), Image.BICUBIC)
        g = np.array(big, np.float32)[n:2 * n, n:2 * n] / 255
        base += (g[..., None] - 0.5) * amp
    color = np.array([0.30, 0.47, 0.18], np.float32)
    img = color + base * np.array([0.6, 1.0, 0.5])
    stripe = (np.sin(np.linspace(0, 4 * math.pi, n, endpoint=False)) > 0)[None, :, None]
    img = img * np.where(stripe, 1.04, 0.96)
    img += (rng.random((n, n, 1)) - 0.5) * 0.06  # ince taneler
    Image.fromarray((np.clip(img, 0, 1) * 255).astype(np.uint8)).save(path)


def runway_texture(path, length, width, px_per_m=10):
    w, h = int(length * px_per_m), int(width * px_per_m)
    noise = (rng.random((h, w)) * 18).astype(np.uint8)
    arr = np.full((h, w, 3), 58, np.uint8) + noise[..., None]
    im = Image.fromarray(arr)
    d = ImageDraw.Draw(im)
    m = px_per_m
    white = (235, 235, 235)
    # kenar cizgileri
    d.rectangle([0, int(0.5 * m), w, int(0.9 * m)], fill=white)
    d.rectangle([0, h - int(0.9 * m), w, h - int(0.5 * m)], fill=white)
    # esik seritleri (iki uc)
    for x0 in (int(2 * m), w - int(10 * m)):
        for k in range(6):
            y = int((2 + k * 2.4) * m)
            d.rectangle([x0, y, x0 + int(8 * m), y + int(1.2 * m)], fill=white)
    # orta cizgi (kesikli)
    x = int(16 * m)
    while x < w - int(16 * m):
        d.rectangle([x, h // 2 - int(0.25 * m), x + int(6 * m), h // 2 + int(0.25 * m)], fill=white)
        x += int(12 * m)
    im.filter(ImageFilter.GaussianBlur(0.5)).save(path)


def quad_obj(path, sx, sy, u, v, mtl_name):
    """Merkezde, z=0'da sx*sy boyutunda tek dortgen; UV 0..u, 0..v."""
    hx, hy = sx / 2, sy / 2
    path.write_text(
        f"# {path.name}\n"
        f"v {-hx} {-hy} 0\nv {hx} {-hy} 0\nv {hx} {hy} 0\nv {-hx} {hy} 0\n"
        f"vt 0 0\nvt {u} 0\nvt {u} {v}\nvt 0 {v}\n"
        "vn 0 0 1\n"
        f"g {mtl_name}\nf 1/1/1 2/2/1 3/3/1\nf 1/1/1 3/3/1 4/4/1\n")


def build_ground_model():
    (GROUND / "meshes").mkdir(parents=True, exist_ok=True)
    tex = GROUND / "materials" / "textures"
    tex.mkdir(parents=True, exist_ok=True)
    grass_texture(tex / "grass.png")
    rl = RUNWAY["x1"] - RUNWAY["x0"]
    runway_texture(tex / "runway.png", rl, RUNWAY["width"])
    rep = GROUND_SIZE / GRASS_TILE
    quad_obj(GROUND / "meshes" / "grass.obj", GROUND_SIZE, GROUND_SIZE, rep, rep, "grass")
    quad_obj(GROUND / "meshes" / "runway.obj", rl, RUNWAY["width"], 1, 1, "runway")
    rx = (RUNWAY["x0"] + RUNWAY["x1"]) / 2
    # agaclar: malzeme basina tek mesh (performans icin carpisma yok)
    tree_visuals = "".join(
        f"""      <visual name="trees_{m}">
        <geometry><mesh><uri>model://toy_ground/meshes/trees_{m}.obj</uri></mesh></geometry>
        <material><ambient>{r*0.6:.3f} {g*0.6:.3f} {b*0.6:.3f} 1</ambient><diffuse>{r} {g} {b} 1</diffuse>
          <specular>0.02 0.02 0.02 1</specular></material>
      </visual>
""" for m, (r, g, b) in TREE_COLORS.items())
    (GROUND / "model.config").write_text("""<?xml version="1.0"?>
<model>
  <name>toy_ground</name>
  <version>1.0</version>
  <sdf version="1.9">model.sdf</sdf>
  <description>Cim zemini ve pist (tools/gen_world.py ile uretildi).</description>
</model>
""")
    (GROUND / "model.sdf").write_text(f"""<?xml version="1.0"?>
<!-- tools/gen_world.py ile uretildi, elle duzenlemeyin. -->
<sdf version="1.9">
  <model name="toy_ground">
    <static>true</static>
    <link name="link">
      <collision name="collision">
        <geometry><plane><normal>0 0 1</normal><size>{GROUND_SIZE} {GROUND_SIZE}</size></plane></geometry>
        <surface><friction><ode><mu>0.6</mu><mu2>0.6</mu2></ode></friction></surface>
      </collision>
      <visual name="grass">
        <geometry><mesh><uri>model://toy_ground/meshes/grass.obj</uri></mesh></geometry>
        <material>
          <diffuse>1 1 1 1</diffuse>
          <specular>0.05 0.05 0.05 1</specular>
          <pbr><metal>
            <albedo_map>model://toy_ground/materials/textures/grass.png</albedo_map>
            <roughness>0.95</roughness><metalness>0</metalness>
          </metal></pbr>
        </material>
      </visual>
{tree_visuals}      <visual name="runway">
        <pose>{rx} 0 0.02 0 0 0</pose>
        <geometry><mesh><uri>model://toy_ground/meshes/runway.obj</uri></mesh></geometry>
        <material>
          <diffuse>1 1 1 1</diffuse>
          <specular>0.1 0.1 0.1 1</specular>
          <pbr><metal>
            <albedo_map>model://toy_ground/materials/textures/runway.png</albedo_map>
            <roughness>0.85</roughness><metalness>0</metalness>
          </metal></pbr>
        </material>
      </visual>
    </link>
  </model>
</sdf>
""")


# ------------------------------------------------------------ SDF yardimcilari
def mat(r, g, b, a=1.0):
    return (f"<material><ambient>{r*0.6:.3f} {g*0.6:.3f} {b*0.6:.3f} {a}</ambient>"
            f"<diffuse>{r} {g} {b} {a}</diffuse><specular>0.1 0.1 0.1 1</specular></material>")


COLORS = dict(red=(0.85, 0.08, 0.08), white=(0.95, 0.95, 0.95), blue=(0.08, 0.25, 0.85),
              yellow=(0.95, 0.80, 0.05), orange=(1.0, 0.45, 0.0), black=(0.05, 0.05, 0.05),
              concrete=(0.62, 0.62, 0.60), roof=(0.25, 0.27, 0.30), glass=(0.35, 0.55, 0.65),
              trunk=(0.35, 0.22, 0.12))


def shape(kind, size, pose, color, collide=True, name=None):
    """kind: box (sx sy sz) | cylinder (r h) | sphere (r)."""
    if kind == "box":
        geo = f"<box><size>{size[0]:.3f} {size[1]:.3f} {size[2]:.3f}</size></box>"
    elif kind == "cylinder":
        geo = f"<cylinder><radius>{size[0]:.3f}</radius><length>{size[1]:.3f}</length></cylinder>"
    else:
        geo = f"<sphere><radius>{size[0]:.3f}</radius></sphere>"
    pose = tuple(pose) + (0.0,) * (6 - len(pose))  # x y z [roll pitch yaw]
    p = " ".join(f"{v:.3f}" for v in pose)
    n = name or f"s{shape.i}"
    shape.i += 1
    c = (f"<collision name='{n}_c'><pose>{p}</pose><geometry>{geo}</geometry></collision>"
         if collide else "")
    return f"<visual name='{n}'><pose>{p}</pose><geometry>{geo}</geometry>{mat(*color)}</visual>{c}"


shape.i = 0


def static_model(name, x, y, body, yaw=0.0):
    return (f"    <model name='{name}'><static>true</static><pose>{x} {y} 0 0 0 {yaw}</pose>"
            f"<link name='link'>{body}</link></model>\n")


def pylon(i, x, y):
    body = ""
    h = 1.0
    for k in range(10):  # kirmizi/beyaz seritli 10 m direk
        body += shape("cylinder", (0.6, h), (0, 0, k * h + h / 2), COLORS["red" if k % 2 == 0 else "white"])
    body += shape("sphere", (0.8,), (0, 0, 10.4), COLORS["orange"])
    return static_model(f"pylon_{i}", x, y, body)


def gate(i, x, y, yaw):
    w, hgt, t = GATE_WIDTH, GATE_HEIGHT, 0.5
    body = shape("box", (t, t, hgt), (0, -w / 2, hgt / 2), COLORS["orange"])
    body += shape("box", (t, t, hgt), (0, w / 2, hgt / 2), COLORS["orange"])
    body += shape("box", (t, w + t, t), (0, 0, hgt), COLORS["orange"])
    for s in (-1, 1):  # tabanda beyaz bayrak bloklari
        body += shape("box", (1.5, 1.5, 0.4), (0, s * w / 2, 0.2), COLORS["white"])
    return static_model(f"gate_{i}", x, y, body, yaw)


def target(i, x, y, color):
    body = ""
    for k, (r, c) in enumerate([(5.0, "white"), (4.0, color), (2.8, "white"), (1.6, color), (0.6, "white")]):
        body += shape("cylinder", (r, 0.02), (0, 0, 0.03 + k * 0.01), COLORS[c], collide=False)
    return static_model(f"target_{i}_{color}", x, y, body)


def landing_zone(x, y):
    body = shape("cylinder", (12.0, 0.02), (0, 0, 0.03), COLORS["white"], collide=False)
    body += shape("cylinder", (11.0, 0.02), (0, 0, 0.04), (0.20, 0.40, 0.15), collide=False)
    body += shape("box", (1.2, 8, 0.02), (-3, 0, 0.05), COLORS["white"], collide=False)
    body += shape("box", (1.2, 8, 0.02), (3, 0, 0.05), COLORS["white"], collide=False)
    body += shape("box", (6, 1.2, 0.02), (0, 0, 0.05), COLORS["white"], collide=False)
    return static_model("landing_zone", x, y, body)


def hangar(x, y):
    body = shape("box", (24, 16, 7), (0, 0, 3.5), COLORS["concrete"])
    body += shape("box", (25, 17, 0.6), (0, 0, 7.3), COLORS["roof"])
    body += shape("box", (0.2, 12, 5.5), (12.05, 0, 2.75), (0.30, 0.32, 0.35), collide=False)  # kapi
    return static_model("hangar", x, y, body)


def tower(x, y):
    body = shape("box", (4, 4, 10), (0, 0, 5), COLORS["concrete"])
    body += shape("box", (5, 5, 2.5), (0, 0, 11.25), COLORS["glass"])
    body += shape("box", (5.6, 5.6, 0.4), (0, 0, 12.7), COLORS["roof"])
    return static_model("tower", x, y, body)


def windsock(x, y):
    body = shape("cylinder", (0.08, 6), (0, 0, 3), COLORS["white"])
    body += shape("cylinder", (0.35, 2.2), (1.1, 0, 5.7, 0, 1.57, 0), COLORS["orange"], collide=False)
    return static_model("windsock", x, y, body)


class Mesh:
    """Tum agaclari tek OBJ'de toplar (malzeme basina bir cizim cagrisi)."""

    def __init__(self):
        self.v = []
        self.faces = {}

    def _add(self, mtl, verts, tris):
        # ucgenleri sekil merkezinden disa bakacak sekilde yonlendir (normal + cizim icin)
        c = np.mean(np.array(verts), axis=0)
        fixed = []
        for t in tris:
            a, b, d = (np.array(verts[i]) for i in t)
            n = np.cross(b - a, d - a)
            fixed.append(t if np.dot(n, (a + b + d) / 3 - c) >= 0 else (t[0], t[2], t[1]))
        base = len(self.v) + 1
        self.v.extend(verts)
        self.faces.setdefault(mtl, []).extend(tuple(base + i for i in t) for t in fixed)

    def cone(self, mtl, x, y, z0, r0, r1, h, n=7, yaw=0.0):
        """z0'dan h yukari kesik koni (r1=0 ise sivri); yanlar + taban."""
        ring = lambda r, z: [(x + r * math.cos(yaw + 2 * math.pi * k / n),
                              y + r * math.sin(yaw + 2 * math.pi * k / n), z) for k in range(n)]
        verts = ring(r0, z0) + (ring(r1, z0 + h) if r1 > 0 else [(x, y, z0 + h)])
        tris = []
        for k in range(n):
            k2 = (k + 1) % n
            if r1 > 0:
                tris += [(k, k2, n + k2), (k, n + k2, n + k)]
            else:
                tris.append((k, k2, n))
        tris += [(0, k2, k) for k, k2 in ((k, (k + 1) % n) for k in range(1, n - 1))]
        self._add(mtl, verts, tris)

    def blob(self, mtl, x, y, z, r, yaw=0.0):
        """Dusuk poligonlu kure (6 boylam x 4 enlem)."""
        nl, nb = 6, 4
        verts = [(x, y, z + r)]
        for j in range(1, nb):
            th = math.pi * j / nb
            for k in range(nl):
                ph = yaw + 2 * math.pi * k / nl
                verts.append((x + r * math.sin(th) * math.cos(ph), y + r * math.sin(th) * math.sin(ph),
                              z + r * math.cos(th)))
        verts.append((x, y, z - r))
        tris = []
        for k in range(nl):
            tris.append((0, 1 + k, 1 + (k + 1) % nl))
        for j in range(nb - 2):
            for k in range(nl):
                a0, a1 = 1 + j * nl + k, 1 + j * nl + (k + 1) % nl
                b0, b1 = a0 + nl, a1 + nl
                tris += [(a0, b0, b1), (a0, b1, a1)]
        last, bottom = 1 + (nb - 2) * nl, len(verts) - 1
        for k in range(nl):
            tris.append((bottom, last + (k + 1) % nl, last + k))
        self._add(mtl, verts, tris)

    def write(self, mesh_dir, prefix):
        """Malzeme basina bir OBJ yazar (renk SDF'de verilir). Ucgen sayisini dondurur."""
        n = 0
        for mtl, tris in self.faces.items():
            used = sorted({i for t in tris for i in t})
            remap = {old: new + 1 for new, old in enumerate(used)}
            out = [f"# tools/gen_world.py ile uretildi\n"]
            out += [f"v {self.v[i - 1][0]:.2f} {self.v[i - 1][1]:.2f} {self.v[i - 1][2]:.2f}\n" for i in used]
            for k, (a, b, c) in enumerate(tris):  # ucgen basina duz normal
                pa, pb, pc = (np.array(self.v[i - 1]) for i in (a, b, c))
                nrm = np.cross(pb - pa, pc - pa)
                nrm = nrm / (np.linalg.norm(nrm) or 1)
                out.append(f"vn {nrm[0]:.3f} {nrm[1]:.3f} {nrm[2]:.3f}\n")
            out += [f"f {remap[a]}//{k + 1} {remap[b]}//{k + 1} {remap[c]}//{k + 1}\n"
                    for k, (a, b, c) in enumerate(tris)]
            (mesh_dir / f"{prefix}_{mtl}.obj").write_text("".join(out))
            n += len(tris)
        return n


TREE_COLORS = {"trunk": (0.35, 0.22, 0.12), "leaf": (0.16, 0.40, 0.13),
               "leaf_light": (0.25, 0.48, 0.16), "pine": (0.08, 0.28, 0.12)}


def tree(mesh, x, y):
    kind = rng.integers(0, 3)
    s = rng.uniform(0.7, 1.4)
    yaw = rng.uniform(0, 6.28)
    leaf = "leaf" if rng.random() < 0.6 else "leaf_light"
    mesh.cone("trunk", x, y, 0, 0.3 * s, 0.2 * s, 3.2 * s, n=5, yaw=yaw)
    if kind == 0:  # yaprakli
        mesh.blob(leaf, x, y, 4.6 * s, 2.4 * s, yaw)
        mesh.blob(leaf, x + 0.8 * s, y + 0.5 * s, 5.6 * s, 1.7 * s, yaw + 1)
    elif kind == 1:  # cam
        for k in range(3):
            mesh.cone("pine", x, y, (2.2 + k * 1.6) * s, (2.4 - k * 0.6) * s, 0, 2.6 * s, n=7, yaw=yaw)
    else:  # cali / genc agac
        mesh.blob(leaf, x, y, 3.2 * s, 1.7 * s, yaw)


def clear(x, y):
    return not any(a <= x <= b and c <= y <= d for a, b, c, d in KEEP_CLEAR)


def build_world():
    models = "    <include><uri>model://toy_ground</uri><name>toy_ground</name></include>\n"
    for i, (x, y) in enumerate(PYLONS):
        models += pylon(i, x, y)
    for i, (x, y, yaw) in enumerate(GATES):
        models += gate(i, x, y, yaw)
    for i, (x, y, c) in enumerate(TARGETS):
        models += target(i, x, y, c)
    models += landing_zone(*LANDING_ZONE)
    models += hangar(*HANGAR)
    models += tower(*TOWER)
    models += windsock(*WINDSOCK)
    placed = 0
    mesh = Mesh()
    half = GROUND_SIZE / 2 - 20
    while placed < TREE_COUNT:
        if rng.random() < 0.6:  # orman kumeleri
            cx, cy = [(-250, 420), (480, -250), (-380, -300), (550, 450), (650, 100)][rng.integers(0, 5)]
            x, y = cx + rng.normal(0, 60), cy + rng.normal(0, 60)
        else:
            x, y = rng.uniform(-half, half, 2)
        if abs(x) < half and abs(y) < half and clear(x, y):
            tree(mesh, x, y)
            placed += 1
    for old in (GROUND / "meshes").glob("trees*"):
        old.unlink()
    tris = mesh.write(GROUND / "meshes", "trees")

    WORLD.parent.mkdir(parents=True, exist_ok=True)
    WORLD.write_text(f"""<?xml version="1.0" encoding="UTF-8"?>
<!-- tools/gen_world.py ile uretildi, elle duzenlemeyin; YERLESIM bolumunu degistirip yeniden uretin. -->
<sdf version="1.9">
  <world name="toy_parkur">
    <physics type="ode">
      <max_step_size>0.004</max_step_size>
      <real_time_factor>1.0</real_time_factor>
      <real_time_update_rate>250</real_time_update_rate>
    </physics>
    <gravity>0 0 -9.8</gravity>
    <magnetic_field>6e-06 2.3e-05 -4.2e-05</magnetic_field>
    <atmosphere type="adiabatic"/>
    <scene>
      <grid>false</grid>
      <ambient>0.55 0.55 0.6 1</ambient>
      <background>0.62 0.78 0.95 1</background>
      <shadows>true</shadows>
      <sky><clouds><speed>4</speed><humidity>0.4</humidity></clouds></sky>
    </scene>
    <light name="sun" type="directional">
      <pose>0 0 500 0 0 0</pose>
      <cast_shadows>true</cast_shadows>
      <intensity>1.1</intensity>
      <direction>-0.4 0.5 -0.75</direction>
      <diffuse>1 0.97 0.9 1</diffuse>
      <specular>0.3 0.3 0.3 1</specular>
      <attenuation><range>3000</range><constant>1</constant><linear>0</linear><quadratic>0</quadratic></attenuation>
    </light>
    <spherical_coordinates>
      <surface_model>EARTH_WGS84</surface_model>
      <world_frame_orientation>ENU</world_frame_orientation>
      <latitude_deg>{HOME['lat']}</latitude_deg>
      <longitude_deg>{HOME['lon']}</longitude_deg>
      <elevation>{HOME['alt']}</elevation>
    </spherical_coordinates>
{models}  </world>
</sdf>
""")
    return placed, tris


if __name__ == "__main__":
    build_ground_model()
    n, tris = build_world()
    print(f"yazildi: {GROUND.relative_to(REPO)}/, {WORLD.relative_to(REPO)} ({n} agac, tek mesh, {tris} ucgen)")
