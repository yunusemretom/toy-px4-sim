# toy-px4-sim

**Türkçe** | [English](README.en.md)

Yarışmalarda yaygın sabit kanat İHA'lar için hazır **Gazebo simülasyonu**. Aynı uçaklar ve aynı
görev parkuru hem **PX4** hem **ArduPilot** ile çalışır.

| Uçaklar | Otopilot | Dünya | Araçlar |
|---|---|---|---|
| TOY (ekibin uçağı), X-UAV Talon, Mini Talon, Skywalker X8, Skywalker 1900 | PX4 SITL · ArduPilot (ArduPlane) SITL | İnternet gerektirmeyen görev parkuru | 10'a kadar aracı aynı anda uçurma, pymavlink test betikleri |

**Öne çıkanlar**

- 🛩️ **5 uçak**, gerçek ölçülerden: hareketli kontrol yüzeyleri, AVL ile hesaplanmış aerodinamik.
- 🔁 **İki otopilot**, tek model kaynağı: PX4 ve ArduPilot sürümleri aynı tanımdan üretilir.
- 🗺️ **Görev parkuru**: pist, yarış direkleri, kapılar, kamerayla tespit için yer hedefleri.
- 👥 **Çoklu araç**: farklı modellerden karışık filo, gerçek zamanda.
- ⚡ **NVIDIA desteği**: hibrit dizüstülerde Gazebo otomatik olarak NVIDIA ekran kartına yönlendirilir.

> PX4 ve ArduPilot bu repoda değildir. Her biri kendi resmi deposundan kurulur; bu repo uçakları,
> dünyayı ve ayarları onlara bağlar. Bu repoda `git pull` yapmak her şeyi günceller.

---

## İçindekiler

1. [Hızlı başlangıç: PX4](#hızlı-başlangıç-px4)
2. [Hızlı başlangıç: ArduPilot](#hızlı-başlangıç-ardupilot)
3. [Kullanım](#kullanım)
4. [Uçaklar](#uçaklar)
5. [Görev parkuru](#görev-parkuru)
6. [Bağlantılar ve test betikleri](#bağlantılar-ve-test-betikleri)
7. [Performans](#performans)
8. [Geliştirme](#geliştirme)
9. [Sorun giderme](#sorun-giderme)
10. [Lisans](#lisans)

---

## Hızlı başlangıç: PX4

Ubuntu 22.04 / 24.04 gerekir. Gazebo Harmonic, PX4 kurulumuyla birlikte gelir.

```bash
# 1. PX4 (bir kez)
git clone https://github.com/PX4/PX4-Autopilot.git --recursive ~/PX4-Autopilot
bash ~/PX4-Autopilot/Tools/setup/ubuntu.sh          # sonra oturumu kapatıp açın

# 2. Bu repo
git clone https://github.com/yunusemretom/toy-px4-sim.git
cd toy-px4-sim
./install.sh ~/PX4-Autopilot
pip install -r requirements.txt

# 3. Çalıştır
./run.sh talon
```

Gazebo açılır, uçak parkurdaki pistte bekler. Kalkış ve manevra testi için ikinci bir terminalde:

```bash
python3 scripts/maneuver_test.py
```

## Hızlı başlangıç: ArduPilot

PX4 kurulumu gerekmez.

```bash
# 1. Bağımlılıklar (bir kez)
sudo apt install rapidjson-dev libgz-sim8-dev libopencv-dev libgstreamer1.0-dev \
     libgstreamer-plugins-base1.0-dev gstreamer1.0-plugins-bad gstreamer1.0-plugins-ugly \
     gstreamer1.0-libav gstreamer1.0-gl
pip install --user empy==3.3.4 pexpect future pymavlink

# 2. ArduPilot + Gazebo eklentisi (bir kez; ~1.5 GB indirir ve derler)
./ardupilot/setup.sh ~/

# 3. Çalıştır
./ardupilot/run.sh talon
```

Kalkış ve manevra testi için ikinci bir terminalde:

```bash
python3 scripts/ap_test.py maneuver
```

MAVProxy ile bağlanmak isterseniz: `mavproxy.py --master tcp:127.0.0.1:5760 --console --map`

> **Önemli:** Komutları `toy-px4-sim` klasörünün içinden çalıştırın. Başka bir klasördeyseniz
> tam yolu kullanın: `~/toy-px4-sim/ardupilot/run.sh talon`

---

## Kullanım

### Tek uçak

| | PX4 | ArduPilot |
|---|---|---|
| Başlat | `./run.sh <model>` | `./ardupilot/run.sh <model>` |
| Pencere olmadan | `--headless` | `--headless` |
| Serbest kamera | `--no-follow` | `--no-follow` |
| NVIDIA'yı zorlama | `--no-nvidia` | `--no-nvidia` |
| Boş dünya | `--world default` | — |
| Kapat | Ctrl+C | Ctrl+C |

`<model>`: `toy`, `talon`, `mini_talon`, `x8`, `skywalker1900` (verilmezse `toy`).

### Çok uçak

En fazla 10 uçak; her biri ayrı bir otopilot sürecidir.

```bash
./multi.sh --headless --vehicles "talon:2,x8:2,mini_talon:2,skywalker1900:2,toy:2"     # PX4
./ardupilot/run.sh --headless --vehicles "talon:2,x8:2,mini_talon:2,skywalker1900:2,toy:2"   # ArduPilot
```

Hepsini kademeli irtifalara (20, 28, 36 … m) birlikte kaldırmak için:

```bash
python3 scripts/swarm_test.py 10        # PX4
python3 scripts/ap_test.py swarm 10     # ArduPilot
```

Uçaklar pistin kuzeyine, 6 m arayla, doğuya bakarak dizilir. QGroundControl hepsini UDP 14550'den
görür. Loglar `logs/` klasöründedir.

### Kalkış konumu

Parkurun varsayılan konumu örnek bir değerdir. Uçuş alanınız belli olunca (yalnızca PX4):

```bash
PX4_HOME_LAT=39.80 PX4_HOME_LON=32.60 PX4_HOME_ALT=900 ./run.sh talon
```

---

## Uçaklar

| Model | Ad | Açıklık | Kütle | Yapı | Trim hızı |
|---|---|---|---|---|---|
| TOY (ekibin uçağı) | `toy` | 1.40 m | 2.75 kg | klasik kuyruk, itici | 18 m/s |
| X-UAV Talon | `talon` | 1.72 m | 2.8 kg | V-kuyruk, itici | 15 m/s |
| X-UAV Mini Talon | `mini_talon` | 1.30 m | 1.6 kg | V-kuyruk, itici | 15 m/s |
| Skywalker X8 | `x8` | 2.12 m | 3.2 kg | uçan kanat, elevon, itici | 18 m/s |
| Skywalker 1900 | `skywalker1900` | 1.90 m | 2.0 kg | T-kuyruk, itici | 13 m/s |

Her uçakta hareketli kontrol yüzeyleri, itici motor ve burunda kamera (640×480 @ 20 Hz, 15° aşağı)
vardır. Aerodinamik katsayılar [AVL](https://web.mit.edu/drela/Public/web/avl/) ile hesaplanmıştır.

### Test sonuçları

Her uçak iki otopilotla da aynı testten geçti: kalkış, ardından 40° yatışlı sert dönüşlerle 40,
20 ve 35 m'ye irtifa değişiklikleri.

| | PX4 | ArduPilot |
|---|---|---|
| Tek tek manevra testi | 5/5 uçak, en düşük irtifa 18–21 m | 5/5 uçak, en düşük irtifa 19 m |
| 10 uçaklı karışık filo (`--headless`) | 10/10 hedef irtifada, gerçek zamanda | 10/10 hedef irtifada, gerçek zamanda |

### Ölçülerin kaynağı

- **TOY:** Ekibin CAD montajı (`cad/1-Montaj.STEP`). Kanat +3.2° hücum açısıyla takılı, Clark Y profil.
- **Diğerleri:** Açıklık, uzunluk, kanat alanı, ağırlık ve kuyruk tipi satıcıların yayınladığı
  değerlerden. Kiriş dağılımı, kuyruk boyutları, gövde kesitleri ve motor konumu fotoğraf
  oranlarından tahmin edildi. X8'in ataleti ölçülmüş bir değerdir (Løw-Hansen vd., *CEAS
  Aeronautical Journal*, 2025).
- **Doğrulama:** X8 için hesaplanan ağırlık merkezi (burundan 443 mm), üreticinin önerdiği
  430–440 mm ile örtüşüyor.

### Gerçek uçak için uyarılar

> ⚠️ **TOY'un ağırlık merkezi:** Hücum kenarından 80 mm gerideyken AVL'ye göre statik marj yalnızca
> **~%1 MAC** (gövde modellenmedi; gerçekte daha düşük olabilir). Simülasyonda otopilot bunu telafi
> ediyor. Gerçek uçakta ilk uçuştan önce ağırlık merkezini öne alın (hücum kenarından ~6–6.5 cm).

- Hız ve kazanç ayarları simülasyon için hesaplandı; gerçek uçakta başlangıç değeridir, uçuş
  testleriyle (autotune) ayarlanmalıdır.
- **X8** gerçekte elden/katapultla atılır. Simülasyonda pistten kalkabilmesi için burnunun altında
  küçük bir kızak var.

---

## Görev parkuru

İnternet gerektirmeyen, gerçekçi görünümlü bir test alanı. Tüm dokular ve modeller repodadır.

```
                 kuzey (+y)
                    ^
   P4 (0,300) ------ G2 (150,300) ------ P3 (300,300)
      |                                       |
      |   T1 (80,225)   T2 (150,225)   T3 (220,225)
      |                                       |
   P1 (0,150) ------ G1 (150,150) ------ P2 (300,150)

   ===== PİST (x: -40..160) =====>  doğu (+x)       P  yarış direği
   * uçaklar y = 6..60'ta, doğuya bakar              G  kapı
       hangar (25,-32)  kule (60,-38)    H (230,-70) T  yer hedefi
                                                     H  iniş alanı
```

| Öğe | Ayrıntı |
|---|---|
| Pist | 200 × 18 m, asfalt ve işaretli |
| Yarış direkleri (P) | 4 adet, kırmızı-beyaz, 10 m |
| Kapılar (G) | 2 adet, turuncu, 16 m geniş × 12 m yüksek |
| Yer hedefleri (T) | Kırmızı, mavi, sarı halkalı |
| İniş alanı (H) | Beyaz çemberli "H" |
| Çevre | Hangar, kule, rüzgar tulumu, 160 ağaç, bulutlu gökyüzü |

Koordinatlar metre cinsindendir. Direk, kapı ve binaların çarpışması vardır; ağaçların performans
için yoktur. Parkuru değiştirmek için `tools/gen_world.py` içindeki `YERLESIM` bölümünü düzenleyip
`python3 tools/gen_world.py` çalıştırın.

---

## Bağlantılar ve test betikleri

| Bağlantı | PX4 | ArduPilot |
|---|---|---|
| QGroundControl | UDP 14550 (otomatik) | UDP 14550 (otomatik) |
| pymavlink / MAVSDK / MAVProxy | `udp:127.0.0.1:14540` (uçak i: `14540+i`) | `tcp:127.0.0.1:5760` (uçak i: `5760+10i`) |
| ROS 2 | uXRCE-DDS ajanı, UDP 8888 | — |
| Kamera video akışı | UDP 5600 (RTP/H.264) | — |

**Test betikleri** (`scripts/`)

| Betik | Otopilot | Ne yapar |
|---|---|---|
| `maneuver_test.py` | PX4 | Kalkış, sert dönüşler, irtifa değişimi; kontrol yüzeyi komutlarını yazdırır |
| `swarm_test.py [n]` | PX4 | n uçağı kademeli irtifalara kaldırır |
| `fw_test.py` | PX4 | Basit kalkış, loiter, RTL |
| `surf_test.py` | PX4 | Kontrol yüzeylerini sırayla ±25° oynatır (uçak yerdeyken) |
| `ap_test.py maneuver` | ArduPilot | TAKEOFF, ardından GUIDED modda sert dönüşler ve irtifa değişimi |
| `ap_test.py swarm [n]` | ArduPilot | n uçağı kademeli irtifalara kaldırır |
| `grab_cam.py <dosya.png>` | — | Burun kamerasından tek kare kaydeder |

`ap_test.py` Ctrl+C ile durdurulursa uçakları RTL moduna alır.

> PX4, yer istasyonu bağlantısı görmeden arm etmez. QGroundControl açık değilse kendi betiğiniz
> `MAV_TYPE_GCS` heartbeat'i göndermelidir (örnek: `scripts/fw_test.py`).

---

## Performans

Hibrit (Intel + NVIDIA) dizüstülerde Gazebo varsayılan olarak **entegre GPU** ile çizer ve kasar.
`run.sh` ve `ardupilot/run.sh`, NVIDIA kart varsa Gazebo'yu otomatik olarak ona yönlendirir.

Ölçüm (RTX 4050 dizüstü, uçuş sırasında):

| Durum | Gerçek zaman faktörü |
|---|---|
| Tek uçak, Intel GPU | 0.29 – 1.00 (kasıyor) |
| Tek uçak, NVIDIA | **1.00** |
| 10 uçak, PX4, `--headless` | **1.00** |
| 10 uçak, ArduPilot, `--headless` | **1.00** |

**Çok uçaklı simülasyon için:** `--headless` kullanın; pencere de ayrıca işlemci kullanır.

<details>
<summary>Elle NVIDIA ayarı ve hafifletmeler</summary>

`run.sh` kullanmıyorsanız:

```bash
export __NV_PRIME_RENDER_OFFLOAD=1
export __GLX_VENDOR_LIBRARY_NAME=nvidia
export __EGL_VENDOR_LIBRARY_FILENAMES=/usr/share/glvnd/egl_vendor.d/10_nvidia.json
```

Dünyadaki hafifletmeler: 160 ağaç renk başına tek mesh'tir (birkaç çizim çağrısı) ve çarpışmasızdır;
uçak kamerası 1280×960 @ 30 Hz yerine 640×480 @ 20 Hz'dir. Bunlarla Gazebo sunucusunun işlemci yükü
tek uçakta %140'tan %52'ye indi.

</details>

---

## Geliştirme

### Repo yapısı

```
run.sh, multi.sh             PX4: tek / çok uçak
install.sh, uninstall.sh     PX4'e bağlama / geri alma
ardupilot/                   ArduPilot: setup.sh, run.sh, params/
airframes/                   PX4 airframe'leri (22000–22004)
models/<ad>/                 PX4 modelleri
models/<ad>_ap/              ArduPilot modelleri
models/toy_ground/           parkur zemini, pist, ağaçlar
worlds/                      toy_parkur (PX4), toy_parkur_ap (ArduPilot)
tools/                       üreticiler (aşağıda)
scripts/                     test betikleri
cad/1-Montaj.STEP            TOY'un CAD montajı
```

### Yeni uçak eklemek

`tools/plane_specs.py`'ye bir tanım ekleyin. Klasik kuyruk, T-kuyruk, V-kuyruk ve uçan kanat
desteklenir. Sonra:

```bash
python3 tools/gen_plane.py <ad>        # PX4 modeli + airframe (AVL gerekir)
python3 tools/gen_ardupilot.py <ad>    # ArduPilot modeli + parametreler
./install.sh ~/PX4-Autopilot           # PX4'e bağla
```

Airframe numarası için PX4'ün özel modellere ayırdığı **22000–22999** aralığını kullanın.

<details>
<summary>Üreticiler nasıl çalışır</summary>

**`tools/gen_plane.py`** (PX4) tek tanımdan:

1. Mesh'leri (kontrol yüzeyleri ayrı, hareketli parça) ve AVL geometrisini üretir.
2. AVL ile nötr noktayı bulur; ağırlık merkezini statik marja göre yerleştirir (klasik %10, uçan
   kanat %6 MAC).
3. Seyir hızında kumandasız denge için kuyruk açısını (uçan kanatta uç bükümünü) ayarlar.
4. Her kontrol yüzeyi için ayrı AdvancedLiftDrag katsayıları hesaplar.
5. PX4 airframe'ini yazar: kontrol tahsisi AVL türevlerinden, hızlar modelin kendi stall hızından,
   açısal hız kazançları TOY'a göre kontrol gücü oranından.

**`tools/gen_ardupilot.py`** PX4 modelinden ArduPilot sürümünü üretir. Aerodinamik, mesh'ler ve
eklemler aynı kalır; değişenler:

| | PX4 modeli | ArduPilot modeli |
|---|---|---|
| Otopilot köprüsü | PX4 `gz_bridge` | `ArduPilotPlugin` (JSON, kilitli adım) |
| Sensörler | IMU, barometre, manyetometre, GPS | yalnızca IMU (gerisini ArduPilot SITL üretir) |
| IMU ekseni | gövdeyle aynı | x etrafında 180° (eklenti ileri-sağ-aşağı bekler) |
| Motor | `MulticopterMotorModel` | Gazebo `Thruster` (gaz → itki, N) |
| Kontrol yüzeyleri | PX4 servo konuları | `COMMAND` kanalları aynı eklem kontrolcülerine yazar |
| Dünya | `toy_parkur` (4 ms) | `toy_parkur_ap` (4 ms, gerçek zaman, sistem eklentileri dünyada) |

ArduPilot servo yönleri tahmin edilmez, ölçülür: `tools/ap_calibrate.py` ArduPlane'i kendi iç
modeliyle çalıştırıp her servo fonksiyonunun PWM yönünü ölçer (`tools/ardupilot_signs.json`);
üretici bunu AVL türevleriyle birleştirerek her yüzey için doğru fonksiyonu (aileron 4, elevatör
19, rudder 21, elevon 77/78, V-kuyruk 79/80) ve yönü seçer.

</details>

<details>
<summary>TOY'un aerodinamiğini yeniden hesaplamak (AVL)</summary>

TOY elle kurulmuştur (CAD mesh'leri). Kontrol yüzeyi ölçüleri veya ağırlık merkezi değişince:

```bash
nano models/toy/avl/toy.avl                   # geometriyi düzenle
models/toy/avl/run_avl.sh ~/Avl/bin/avl       # AVL'yi çalıştır
python3 models/toy/avl/avl_to_sdf.py          # model.sdf'yi güncelle, statik marjı yazdır
python3 tools/gen_ardupilot.py toy            # ArduPilot sürümünü güncelle
```

AVL 3.36: <https://web.mit.edu/drela/Public/web/avl/> (gfortran ile derlenir).

| TOY | Değer | Kaynak |
|---|---|---|
| Kanat açıklığı / alan / MAC | 1.40 m / 0.308 m² / 0.222 m | CAD |
| Profil / kanat hücum açısı | Clark Y (~NACA 3412) / +3.2° | CAD |
| Kütle / atalet | 2.75 kg / Ixx 0.15, Iyy 0.14, Izz 0.27 kg·m² | tahmin |
| Ağırlık merkezi | kanat kök hücum kenarından 80 mm geride | ekip |
| Motor | itici, 12" pervane, 4S, ~29 N statik itki | tahmin |
| Kontrol yüzeyleri | aileron (dış yarı, %25 kiriş), elevatör/rudder (%30 kiriş), ±25° | varsayım |

</details>

<details>
<summary><code>install.sh</code> ne yapar</summary>

| Bu repoda | PX4'te (symlink) |
|---|---|
| `models/*/` | `Tools/simulation/gz/models/` |
| `worlds/*.sdf` | `Tools/simulation/gz/worlds/` |
| `airframes/*` | `ROMFS/px4fmu_common/init.d-posix/airframes/` |

Ayrıca airframe adlarını PX4'ün `airframes/CMakeLists.txt` listesine ekler; PX4 reposunda bu dosya
"değişmiş" görünür. Geri almak için: `./uninstall.sh ~/PX4-Autopilot`

</details>

---

## Sorun giderme

| Belirti | Çözüm |
|---|---|
| `no such file or directory: ./run.sh` | Komutu `toy-px4-sim` klasöründe çalıştırın ya da tam yolu verin. |
| Gazebo kasıyor | `run.sh` ile başlatın (NVIDIA'yı seçer); `nvidia-smi` çıktısında `gz sim` görünmeli. Çok uçakta `--headless`. |
| Fare ile yakınlaşamıyorum | Takip modunda yakınlaşma çalışmaz: `--no-follow` |
| PX4: `PX4 server already running` | Önceki simülasyon kapanmamış: `pkill -x px4; pkill -f "gz sim"` |
| PX4: arm reddediliyor (`No connection to the GCS`) | QGroundControl'ü açın ya da betikten GCS heartbeat gönderin. |
| ArduPilot: Gazebo donuk, uçak bağlanmıyor | `ardupilot/run.sh` önceki çalıştırmadan kalanları kendisi kapatır; yine de olursa: `pkill -x arduplane; pkill -f "gz sim"` |
| Çok uçakta bazı uçaklar kalkmıyor | Kalkış yolunda engel olabilir (uçaklar doğuya koşar); `logs/` altındaki loglara bakın. |
| Kontrol yüzeyleri oynamıyor gibi | Düz uçuşta sapmalar 1–2°'dir; manevra testinde belirginleşir. |

---

## Lisans

[MIT](LICENSE) © 2026 Yunus Emre Tom

PX4 ([BSD-3-Clause](https://github.com/PX4/PX4-Autopilot/blob/main/LICENSE)), ArduPilot ve
ardupilot_gazebo ([GPL-3.0](https://github.com/ArduPilot/ardupilot/blob/master/COPYING.txt)) bu
repoya dahil değildir; kendi depolarından, kendi lisanslarıyla kurulur.
