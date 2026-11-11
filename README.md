# toy-px4-sim

**TOY** itici (pusher) sabit kanat uçağı için PX4 + Gazebo simülasyonu: uçak modeli, PX4 airframe
dosyası ve görev parkuru dünyası.

> PX4'ün kendisi bu repoda yok. Herkes PX4'ü kendi bilgisayarına normal şekilde kurar; bu repo
> modeli, dünyayı ve airframe'i o PX4 kopyasına **symlink** ile bağlar. Bu repoda `git pull` yapmak
> PX4 tarafını da günceller.

---

## İçindekiler

1. [Hızlı başlangıç](#hızlı-başlangıç)
2. [Çalıştırma seçenekleri](#çalıştırma-seçenekleri)
3. [Görev parkuru](#görev-parkuru)
4. [Uçağa bağlanmak](#uçağa-bağlanmak)
5. [Performans ve GPU](#performans-ve-gpu)
6. [Uçak modeli](#uçak-modeli)
7. [Geliştirme](#geliştirme)
8. [Sorun giderme](#sorun-giderme)

---

## Hızlı başlangıç

**1. PX4'ü kur** (bir kez; Ubuntu 22.04 / 24.04, Gazebo Harmonic dahil):

```bash
git clone https://github.com/PX4/PX4-Autopilot.git --recursive ~/PX4-Autopilot
bash ~/PX4-Autopilot/Tools/setup/ubuntu.sh
```

Oturumu kapatıp açın. PX4'ün çalıştığını görmek için: `cd ~/PX4-Autopilot && make px4_sitl gz_x500`

**2. Bu repoyu kur:**

```bash
git clone https://github.com/yunusemretom/toy-px4-sim.git
cd toy-px4-sim
./install.sh ~/PX4-Autopilot
pip install -r requirements.txt      # test betikleri için (pymavlink, pillow)
```

**3. Çalıştır:**

```bash
./run.sh
```

Gazebo açılır, uçak parkurdaki pistte durur. QGroundControl açıksa otomatik bağlanır.

**Güncellemek için:** `git pull` (tekrar kurulum gerekmez).

---

## Çalıştırma seçenekleri

| Komut | Ne yapar |
|---|---|
| `./run.sh` | Parkur dünyası, NVIDIA GPU varsa onunla |
| `./run.sh --headless` | Gazebo penceresi olmadan (en hafif) |
| `./run.sh --world default` | PX4'ün boş, gri dünyası |
| `./run.sh --no-follow` | Kamera uçağı takip etmez; fare ile serbest dolaşma ve yakınlaşma |
| `./run.sh --no-nvidia` | NVIDIA'yı zorlamadan sistemin GPU'su |

**Kalkış konumunu değiştirmek** (uçuş alanı belli olunca):

```bash
PX4_HOME_LAT=39.80 PX4_HOME_LON=32.60 PX4_HOME_ALT=900 ./run.sh
```

`run.sh` olmadan da çalışır: `cd ~/PX4-Autopilot && make px4_sitl gz_toy`

---

## Görev parkuru

Gerçekçi görünümlü, internet gerektirmeyen bir test alanı (`worlds/toy_parkur.sdf`).
Konum sabit değil; yukarıdaki `PX4_HOME_*` değişkenleriyle istediğiniz yere taşınır.

```
                 kuzey (+y)
                    ^
   P4 (0,300) ------ G2 (150,300) ------ P3 (300,300)
      |                                       |
      |   T1 (80,225)   T2 (150,225)   T3 (220,225)
      |                                       |
   P1 (0,150) ------ G1 (150,150) ------ P2 (300,150)

   ===== PİST (x: -40..160) =====>  doğu (+x)       P  yarış direği
   * uçak (0,0), doğuya bakar                        G  kapı
       hangar (25,-32)  kule (60,-38)    H (230,-70) T  yer hedefi
                                                     H  iniş alanı
```

| Öğe | Ayrıntı |
|---|---|
| Pist | 200 × 18 m, asfalt ve işaretli |
| Yarış direkleri (P) | 4 adet, kırmızı-beyaz, 10 m |
| Kapılar (G) | 2 adet, turuncu, 16 m geniş × 12 m yüksek |
| Yer hedefleri (T) | Kırmızı, mavi, sarı halkalı (kamerayla tespit için) |
| İniş alanı (H) | Beyaz çemberli "H" |
| Çevre | Hangar, kule, rüzgar tulumu, 160 ağaç, bulutlu gökyüzü |

Koordinatlar metre cinsinden (x = doğu, y = kuzey). Direk, kapı ve binaların çarpışması var;
ağaçların performans için yok.

**Parkuru değiştirmek:** `tools/gen_world.py` içindeki `YERLESIM` bölümünü düzenleyin, sonra
`python3 tools/gen_world.py` çalıştırın.

---

## Uçağa bağlanmak

| Bağlantı | Adres |
|---|---|
| QGroundControl | UDP 14550 (otomatik) |
| pymavlink / MAVSDK | `udp:127.0.0.1:14540` |
| ROS 2 (uXRCE-DDS ajanı) | UDP 8888 |
| Kamera video akışı | UDP 5600 (RTP/H.264) |

**Hazır betikler** (`scripts/`):

| Betik | Ne yapar |
|---|---|
| `fw_test.py [url] [süre]` | Kalkış, 25 m'de loiter, irtifa/hız takibi, RTL |
| `maneuver_test.py` | Sert dönüşler ve irtifa değişiklikleri; PX4'ün kontrol yüzeylerine verdiği komutları yazdırır |
| `surf_test.py [süre]` | Kontrol yüzeylerini sırayla ±25° oynatır (uçak disarm iken) |
| `grab_cam.py <dosya.png>` | Burun kamerasından tek kare kaydeder |

> PX4, yer istasyonu (GCS) bağlantısı görmeden arm etmez. QGC açık değilse betiğiniz
> `MAV_TYPE_GCS` heartbeat'i göndermelidir (örnek: `fw_test.py`).

**Kamera:** burunda, 15° aşağı bakar, 640×480 @ 20 Hz.

- QGC: *Application Settings → General → Video Source: UDP h.264*, port 5600
- Gazebo konusu: `/world/toy_parkur/model/toy_0/link/camera_link/sensor/camera/image`
  (ROS 2'ye `ros_gz_bridge` ile aktarılır)

---

## Performans ve GPU

Hibrit (Intel + NVIDIA) dizüstülerde Gazebo varsayılan olarak **entegre GPU** ile çizer ve kasar.
`run.sh`, NVIDIA kart varsa Gazebo'yu otomatik olarak ona yönlendirir. `run.sh` kullanmıyorsanız
aynısını elle yapın:

```bash
export __NV_PRIME_RENDER_OFFLOAD=1
export __GLX_VENDOR_LIBRARY_NAME=nvidia
export __EGL_VENDOR_LIBRARY_FILENAMES=/usr/share/glvnd/egl_vendor.d/10_nvidia.json
```

Ölçüm (RTX 4050 dizüstü, uçuş sırasında):

| Durum | Gerçek zaman faktörü | Gazebo sunucu CPU |
|---|---|---|
| Intel GPU, ilk dünya | 0.29 – 1.00 (kasıyor) | %105 |
| NVIDIA, ilk dünya | 1.00 | %140 |
| **NVIDIA, hafifletilmiş dünya ve kamera** | **1.00** | **%52** |

Hafifletmeler: ağaçlar tek mesh (renk başına bir çizim çağrısı) ve çarpışmasız; kamera
1280×960 @ 30 Hz yerine 640×480 @ 20 Hz.

**Çok araçlı simülasyon için:** `--headless` kullanın (GUI'nin işlemci yükü gider) ve gerekmeyen
araçlarda kamera kullanmayın; her kamera sahneyi bir kez daha çizer.

---

## Uçak modeli

| Özellik | Değer | Kaynak |
|---|---|---|
| Kanat açıklığı / alan / MAC | 1.40 m / 0.308 m² / 0.222 m | CAD (`cad/1-Montaj.STEP`) |
| Profil / kanat hücum açısı | Clark Y (~NACA 3412) / +3.2° | CAD |
| Kütle / atalet | 2.75 kg / Ixx 0.15, Iyy 0.14, Izz 0.27 kg·m² | tahmin |
| Ağırlık merkezi | kanat kök hücum kenarından 80 mm geride | ekip |
| Motor | itici, 12" pervane, 4S, ~29 N statik itki | tahmin |
| Kontrol yüzeyleri | aileron (dış yarı, %25 kiriş), elevatör/rudder (%30 kiriş), ±25° | varsayım |
| Aerodinamik | AVL 3.36 ile hesaplandı | `models/toy/avl/` |

**Uçuş ayarları** (`airframes/22000_gz_toy`):

- Hızlar: stall 12 / min 15 / trim 18 / max 26 m/s; trim gazı 0.31; yatış sınırı 40°.
- Kanat yükü ~8.9 kg/m² olduğu için daha düşük hızlarda uçak sert dönüşte stall oluyordu.
- Açısal hız kazançları bu modelin kontrol gücüne göre ölçeklendi.
- Gerçek uçak için bunlar başlangıç değerleridir; uçuş testleriyle ayarlanmalıdır.

> ⚠️ **Ağırlık merkezi:** AVL'ye göre statik marj yalnızca **~%1 MAC** (gövde modellenmedi;
> gerçekte daha düşük olabilir). Simülasyonda PX4 bunu telafi ediyor. Gerçek uçakta ilk uçuştan
> önce ağırlık merkezini öne alın (hücum kenarından ~6–6.5 cm).

---

## Geliştirme

### Repo yapısı

```
run.sh                     simülasyonu başlatır (GPU seçimi dahil)
install.sh, uninstall.sh   PX4'e bağlama / geri alma
airframes/22000_gz_toy     PX4 airframe (parametreler, kontrol tahsisi)
models/toy/                uçak: model.sdf, mesh'ler, avl/
models/toy_ground/         parkur zemini, pist, ağaçlar (üretilir)
worlds/toy_parkur.sdf      parkur dünyası (üretilir)
tools/gen_world.py         parkuru üreten betik
scripts/                   pymavlink test betikleri
cad/1-Montaj.STEP          kaynak CAD montajı
```

### `install.sh` ne yapar

| Bu repoda | PX4'te (symlink) |
|---|---|
| `models/*/` | `Tools/simulation/gz/models/` |
| `worlds/*.sdf` | `Tools/simulation/gz/worlds/` |
| `airframes/*` | `ROMFS/px4fmu_common/init.d-posix/airframes/` |

Ayrıca airframe adını PX4'ün `airframes/CMakeLists.txt` listesine ekler; PX4 reposunda bu dosya
"değişmiş" görünür. Geri almak için: `./uninstall.sh ~/PX4-Autopilot`

### Aerodinamik katsayıları yeniden hesaplamak (AVL)

Kontrol yüzeyi ölçüleri veya ağırlık merkezi değişince:

```bash
nano models/toy/avl/toy.avl                   # geometriyi düzenle
models/toy/avl/run_avl.sh ~/Avl/bin/avl       # AVL'yi çalıştır
python3 models/toy/avl/avl_to_sdf.py          # model.sdf'yi güncelle, statik marjı yazdır
```

AVL 3.36: <https://web.mit.edu/drela/Public/web/avl/> (gfortran ile derlenir; ayrıntı için
PX4 dokümanı `docs/en/sim_gazebo_gz/tools_avl_automation.md`).

### Yeni araç eklemek

1. `models/<ad>/` altına `model.sdf` ve `model.config` koyun.
2. `airframes/<id>_gz_<ad>` dosyası oluşturun (`PX4_SIM_MODEL=${PX4_SIM_MODEL:=<ad>}`).
   PX4 özel modeller için **22000–22999** aralığını ayırmıştır; bu aralığı kullanın.
3. `./install.sh ~/PX4-Autopilot`, ardından `./run.sh <ad>`.

---

## Sorun giderme

| Belirti | Çözüm |
|---|---|
| Gazebo kasıyor | `./run.sh` kullanın (NVIDIA'yı seçer); `nvidia-smi` çıktısında `gz sim` görünmeli. Gerekirse `--headless`. |
| `No rule to make target 'px4_sitl'` | Komut PX4 klasöründe çalıştırılmalı; ya da `./run.sh` kullanın. |
| `PX4 server already running` | Önceki simülasyon kapanmamış: `pkill -x px4; pkill -f "gz sim"` |
| Fare ile yakınlaşamıyorum | Takip modunda yakınlaşma çalışmaz: `./run.sh --no-follow` |
| Arm reddediliyor (`No connection to the GCS`) | QGC'yi açın ya da betikten GCS heartbeat gönderin. |
| Kontrol yüzeyleri oynamıyor gibi | Düz uçuşta sapmalar 1–2°'dir. `scripts/maneuver_test.py` ile manevrada görün. |
