"""gen_plane.py icin ucak tanimlari.

Koordinatlar AVL cercevesinde: x = burundan geriye, y = saga, z = yukari (metre).
Gövde referans cizgisi z = 0.

Yayinlanan degerler (satici sayfalari, 2026-11): aciklik, uzunluk, kanat alani (varsa), ucus
agirligi araligi, kuyruk tipi. Kiris dagilimi, kuyruk boyutlari, govde kesitleri, motor konumu ve
itkisi fotograf oranlarindan TAHMIN edilmistir; olculdukce guncelleyin ve
`python3 tools/gen_plane.py <ad>` ile yeniden uretin.

Kontrol yuzeyi tipleri (PX4 CA_SV_CSn_TYPE): ail_l 1, ail_r 2, elevator 3, rudder 4,
el_l 5 (sol elevon), el_r 6, rv_l 7 (sol V-kuyruk), rv_r 8.
"""

PLANES = {
    # ------------------------------------------------------------------ X-UAV Talon
    # Yayinlanan: aciklik 1718 mm, uzunluk 1100 mm, kanat alani 60 dm^2, 2.5-3.0 kg, V-kuyruk, itici.
    "talon": dict(
        title="X-UAV Talon",
        autostart=22001,
        mass=2.8,
        static_margin=0.10,
        wing=dict(span=1.718, root=0.40, tip=0.30, tip_le_offset=0.06, x_le=0.33, z=0.075,
                  dihedral=2.0, incidence=2.0, twist=-1.0, airfoil="2412",
                  control="aileron", ctrl_span=(0.50, 0.95), ctrl_chord=0.25),
        vtail=dict(span=0.31, root=0.20, tip=0.13, tip_le_offset=0.05, x_le=0.86, z=0.045,
                   dihedral=37.0, airfoil="0010", ctrl_chord=0.35),
        fuselage=[  # (x, genislik, yukseklik, z_merkez)
            (0.00, 0.02, 0.02, 0.000), (0.06, 0.09, 0.09, 0.000), (0.20, 0.13, 0.14, 0.010),
            (0.45, 0.13, 0.14, 0.020), (0.62, 0.09, 0.10, 0.030), (0.75, 0.045, 0.05, 0.040),
            (1.06, 0.025, 0.03, 0.045)],
        motor=dict(x=0.66, z=0.115, prop=0.254, thrust=30.0, pusher=True),
        colors=dict(fuselage=(0.55, 0.57, 0.60), wing=(0.92, 0.92, 0.92),
                    tail=(0.92, 0.92, 0.92), ctrl=(0.95, 0.35, 0.05)),
    ),
    # ------------------------------------------------------------------ X-UAV Mini Talon
    # Yayinlanan: aciklik 1300 mm, uzunluk 830 mm, 1.45-1.9 kg, V-kuyruk, itici. Kanat alani yok.
    "mini_talon": dict(
        title="X-UAV Mini Talon",
        autostart=22002,
        mass=1.6,
        static_margin=0.10,
        wing=dict(span=1.30, root=0.30, tip=0.22, tip_le_offset=0.045, x_le=0.25, z=0.057,
                  dihedral=2.0, incidence=2.0, twist=-1.0, airfoil="2412",
                  control="aileron", ctrl_span=(0.50, 0.95), ctrl_chord=0.25),
        vtail=dict(span=0.235, root=0.15, tip=0.10, tip_le_offset=0.04, x_le=0.65, z=0.034,
                   dihedral=37.0, airfoil="0010", ctrl_chord=0.35),
        fuselage=[
            (0.00, 0.015, 0.015, 0.000), (0.045, 0.07, 0.07, 0.000), (0.15, 0.10, 0.105, 0.008),
            (0.34, 0.10, 0.105, 0.015), (0.47, 0.07, 0.075, 0.023), (0.57, 0.035, 0.04, 0.030),
            (0.80, 0.02, 0.023, 0.034)],
        motor=dict(x=0.50, z=0.087, prop=0.203, thrust=18.0, pusher=True),
        colors=dict(fuselage=(0.20, 0.22, 0.25), wing=(0.20, 0.22, 0.25),
                    tail=(0.20, 0.22, 0.25), ctrl=(0.95, 0.80, 0.05)),
    ),
    # ------------------------------------------------------------------ Skywalker X8
    # Yayinlanan: aciklik 2120 mm, kanat alani 75-80 dm^2, govde 790 mm, ~3.2 kg, ucan kanat,
    # elevon, itici, agirlik merkezi burundan ~430-440 mm.
    "x8": dict(
        title="Skywalker X8",
        autostart=22003,
        mass=3.2,
        # Olculmus atalet (Low-Hansen vd., CEAS Aeronautical Journal 2025, Tablo 1: 3.364 kg icin
        # Ix 0.325, Iy 0.140, Iz 0.400 kg m^2), 3.2 kg'a olceklendi.
        inertia=(0.325 * 3.2 / 3.364, 0.140 * 3.2 / 3.364, 0.400 * 3.2 / 3.364),
        # Pistten kalkis icin burun altinda kizak: yerde ~7 deg burun yukari oturur
        # (gercek X8 elden/katapultla atilir; simulasyonda pist kalkisi icin).
        nose_skid=0.085,
        static_margin=0.06,
        flying_wing=True,
        # Ucan kanat dar hiz araliginda ucar; gercek X8 seyir hizi ~18 m/s. Daha dusuk hizlarda
        # 40 deg yatisli donuste stall oluyordu.
        trim_factor=2.0,
        min_factor=1.35,
        roll_limit=35,
        # Gercek X8 reflex profil kullanir (sifir-aci momenti ~0). AVL yalnizca NACA 4 hanelileri
        # destekledigi icin en yakin davranis olarak simetrik NACA 0012 secildi.
        wing=dict(span=2.12, root=0.56, tip=0.20, tip_le_offset=0.50, x_le=0.15, z=0.0,
                  dihedral=0.0, incidence=4.5, twist=-3.0, airfoil="0012",
                  control="elevon", ctrl_span=(0.25, 0.95), ctrl_chord=0.22),
        winglet=dict(height=0.20, root=0.20, tip=0.10, tip_le_offset=0.08, airfoil="0008"),
        fuselage=[
            (0.00, 0.04, 0.04, 0.000), (0.08, 0.16, 0.11, 0.000), (0.30, 0.22, 0.12, 0.000),
            (0.55, 0.20, 0.10, 0.000), (0.72, 0.12, 0.06, 0.000), (0.79, 0.06, 0.04, 0.000)],
        motor=dict(x=0.82, z=0.0, prop=0.279, thrust=35.0, pusher=True),
        colors=dict(fuselage=(0.10, 0.10, 0.10), wing=(0.95, 0.95, 0.95),
                    tail=(0.10, 0.10, 0.10), ctrl=(0.85, 0.10, 0.10)),
    ),
    # ------------------------------------------------------------------ Skywalker 1900
    # Yayinlanan: aciklik ~1900 mm, uzunluk 1180-1300 mm, kanat alani ~45 dm^2, 1.3-2.5 kg,
    # T-kuyruk. Motor konumu kaynaklarda net degil; klasik Skywalker duzeni (kanat arkasinda,
    # govde ustunde itici) varsayildi.
    "skywalker1900": dict(
        title="Skywalker 1900",
        autostart=22004,
        mass=2.0,
        static_margin=0.10,
        wing=dict(span=1.90, root=0.27, tip=0.20, tip_le_offset=0.04, x_le=0.30, z=0.065,
                  dihedral=3.0, incidence=2.0, twist=-1.0, airfoil="2412",
                  control="aileron", ctrl_span=(0.45, 0.92), ctrl_chord=0.25),
        htail=dict(span=0.50, root=0.14, tip=0.10, tip_le_offset=0.03, x_le=1.02, z=0.235,
                   airfoil="0009", ctrl_chord=0.35),
        fin=dict(height=0.21, root=0.18, tip=0.13, tip_le_offset=0.05, x_le=0.97, z=0.03,
                 airfoil="0009", ctrl_chord=0.35),
        fuselage=[
            (0.00, 0.02, 0.02, 0.000), (0.07, 0.09, 0.10, 0.000), (0.25, 0.11, 0.13, 0.010),
            (0.50, 0.10, 0.11, 0.020), (0.66, 0.05, 0.05, 0.030), (1.16, 0.022, 0.025, 0.035)],
        motor=dict(x=0.62, z=0.125, prop=0.229, thrust=22.0, pusher=True),
        colors=dict(fuselage=(0.93, 0.93, 0.93), wing=(0.93, 0.93, 0.93),
                    tail=(0.93, 0.93, 0.93), ctrl=(0.15, 0.35, 0.85)),
    ),
}
