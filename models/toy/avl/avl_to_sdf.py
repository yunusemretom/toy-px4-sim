#!/usr/bin/env python3
"""AVL turev dosyalarini (toy_st.txt, toy_sb.txt) okuyup ../model.sdf icindeki
AdvancedLiftDrag katsayilarini gunceller.

Isaret kurallari:
- AVL stabilite eksenleri: CL yukari, CY saga, Cl sag kanat asagi, Cm burun yukari, Cn burun saga.
  alpha = 0'da stabilite eksenleri govde eksenleriyle ayni, oranlar (p, q, r) boyutsuz (pb/2V, qc/2V, rb/2V).
- AVL kontrol turevleri derece basina; pozitif sapma = firar kenari asagi (rudder: saga).
- SDF eklemleri: aileron/elevator ekseni +y (sol) -> pozitif aci = firar kenari YUKARI,
  rudder ekseni +z -> pozitif aci = firar kenari saga.
  * elevator: direction -1 ile AVL degerleri oldugu gibi kullanilir.
  * rudder:   direction  1 ile AVL degerleri oldugu gibi kullanilir.
  * aileronlar: direction 1, her yuzey icin "firar kenari yukari" turevleri:
      antisimetrik 'aileron' (sag asagi / sol yukari) -> Cl, Cn, CY yariya bolunur,
      simetrik 'ailsym' (iki taraf asagi)            -> CL, Cm, CD yariya bolunur, isaret cevrilir.
- CD0: plugin induklenmis suruklemeyi CL^2/(pi*AR*e) ile ayrica ekler, bu yuzden
  CD0 = CDtot - CLtot^2/(pi*AR*e).
"""
import math
import re
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
ST = HERE / "toy_st.txt"
SB = HERE / "toy_sb.txt"
SDF = HERE.parent / "model.sdf"

AR = 1.40**2 / 0.308
NUM = r"[-+]?(?:\d+\.\d*|\.\d+|\d+)(?:[EeDd][-+]?\d+)?"
PAIR = re.compile(r"([A-Za-z][A-Za-z0-9']*)\s*=\s*(" + NUM + ")")


def parse(path):
    vals = {}
    for line in path.read_text().splitlines():
        for k, v in PAIR.findall(line):
            vals.setdefault(k, float(v.replace("D", "E").replace("d", "e")))
    return vals


def get(d, *names, default=None):
    for n in names:
        if n in d:
            return d[n]
    if default is not None:
        print(f"  uyari: {names} bulunamadi, {default} kullaniliyor")
        return default
    sys.exit(f"HATA: {names} AVL ciktisinda yok")


st = parse(ST)
sb = parse(SB) if SB.exists() else {}

CL0 = get(st, "CLtot")
CD_tot = get(st, "CDtot")
Cm0 = get(st, "Cmtot")
e = get(st, "e", default=0.85)
e = min(max(e, 0.5), 1.0)

model = {
    "CL0": CL0,
    "CLa": get(st, "CLa"),
    "AR": AR,
    "eff": e,
    "CD0": CD_tot - CL0**2 / (math.pi * AR * e),
    "Cem0": Cm0,
    "Cema": get(st, "Cma"),
    "CYb": get(st, "CYb"),
    "Cellb": get(st, "Clb", "Cl'b"),
    "Cenb": get(st, "Cnb", "Cn'b"),
    "CYp": get(st, "CYp"),
    "CLp": get(st, "CLp"),
    "Cellp": get(st, "Clp", "Cl'p"),
    "Cemp": get(st, "Cmp"),
    "Cenp": get(st, "Cnp", "Cn'p"),
    "CYq": get(st, "CYq"),
    "CLq": get(st, "CLq"),
    "Cellq": get(st, "Clq", "Cl'q"),
    "Cemq": get(st, "Cmq"),
    "Cenq": get(st, "Cnq", "Cn'q"),
    "CYr": get(st, "CYr"),
    "CLr": get(st, "CLr"),
    "Cellr": get(st, "Clr", "Cl'r"),
    "Cemr": get(st, "Cmr"),
    "Cenr": get(st, "Cnr", "Cn'r"),
    "CDp": -get(sb, "CXp", default=0.0),
    "CDq": -get(sb, "CXq", default=0.0),
    "CDr": -get(sb, "CXr", default=0.0),
}


def ctrl(i):
    s = f"d{i}"  # AVL 3.36: CLd1, Cmd3, CDffd2 ...
    return {
        "CL": get(st, "CL" + s),
        "CY": get(st, "CY" + s),
        "Cl": get(st, "Cl" + s, "Cl'" + s),
        "Cm": get(st, "Cm" + s),
        "Cn": get(st, "Cn" + s, "Cn'" + s),
        "CD": get(st, "CDff" + s, "CD" + s, default=0.0),
    }


ail, sym, elev, rud = ctrl(1), ctrl(2), ctrl(3), ctrl(4)

# Firar kenari yukari, tek yuzey
left = {"CD": -sym["CD"] / 2, "CY": ail["CY"] / 2, "CL": -sym["CL"] / 2,
        "Cell": ail["Cl"] / 2, "Cem": -sym["Cm"] / 2, "Cen": ail["Cn"] / 2}
right = {"CD": -sym["CD"] / 2, "CY": -ail["CY"] / 2, "CL": -sym["CL"] / 2,
         "Cell": -ail["Cl"] / 2, "Cem": -sym["Cm"] / 2, "Cen": -ail["Cn"] / 2}
elevator = {"CD": elev["CD"], "CY": elev["CY"], "CL": elev["CL"],
            "Cell": elev["Cl"], "Cem": elev["Cm"], "Cen": elev["Cn"]}
rudder = {"CD": rud["CD"], "CY": rud["CY"], "CL": rud["CL"],
          "Cell": rud["Cl"], "Cem": rud["Cm"], "Cen": rud["Cn"]}
surfaces = {"servo_0": left, "servo_1": right, "servo_2": elevator, "servo_3": rudder}

text = SDF.read_text()
start = text.index('<plugin filename="gz-sim-advanced-lift-drag-system"')
end = text.index("</plugin>", start)
plugin = text[start:end]
head, sep, ctrl_part = plugin.partition("<control_surface>")


def set_tag(block, tag, value):
    new, n = re.subn(rf"<{tag}>[^<]*</{tag}>", f"<{tag}>{value:.6g}</{tag}>", block, count=1)
    if n != 1:
        sys.exit(f"HATA: <{tag}> SDF'de bulunamadi")
    return new


for tag, value in model.items():
    head = set_tag(head, tag, value)

blocks = ctrl_part.split("<control_surface>")
for bi, block in enumerate(blocks):
    name = re.search(r"<name>([^<]+)</name>", block).group(1)
    for k, v in surfaces[name].items():
        block = set_tag(block, f"{k}_ctrl", v)
    blocks[bi] = block

plugin = head + sep + "<control_surface>".join(blocks)
SDF.write_text(text[:start] + plugin + text[end:])

print(f"Guncellendi: {SDF}")
print(f"  CL0={model['CL0']:.4f} CLa={model['CLa']:.3f} CD0={model['CD0']:.4f} e={e:.3f}")
print(f"  Cm0={Cm0:.4f} Cma={model['Cema']:.4f} Cmq={model['Cemq']:.3f}")
print(f"  Cnb={model['Cenb']:.4f} Clb={model['Cellb']:.4f} Clp={model['Cellp']:.3f} Cnr={model['Cenr']:.4f}")
xnp = get(st, "Xnp", default=0.0)
if xnp:
    sm = (xnp - 0.4093) / 0.222
    print(f"  Notr nokta Xnp={xnp:.4f} m (burundan) -> statik marj = %{sm*100:.1f} MAC")
for n, s in surfaces.items():
    print(f"  {n}: " + " ".join(f"{k}={v:+.5f}" for k, v in s.items()))
