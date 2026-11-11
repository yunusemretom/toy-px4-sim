# toy-px4-sim

Yarışmalarda yaygın sabit kanat uçaklar için PX4 + Gazebo simülasyonu: **TOY** (ekibin uçağı),
**X-UAV Talon**, **Mini Talon**, **Skywalker X8** ve **Skywalker 1900** modelleri, PX4 airframe
dosyaları ve görev parkuru dünyası.

> PX4'ün kendisi bu repoda yok. Herkes PX4'ü kendi bilgisayarına normal şekilde kurar; bu repo
> modeli, dünyayı ve airframe'i o PX4 kopyasına **symlink** ile bağlar. Bu repoda `git pull` yapmak
> PX4 tarafını da günceller.

---

## İçindekiler

1. [Hızlı başlangıç](#hızlı-başlangıç)
2. [Çalıştırma seçenekleri](#çalıştırma-seçenekleri)
3. [Çok araçlı simülasyon](#çok-araçlı-simülasyon)
4. [Görev parkuru](#görev-parkuru)
5. [Uçağa bağlanmak](#uçağa-bağlanmak)
6. [Performans ve GPU](#performans-ve-gpu)
7. [Uçak modelleri](#uçak-modelleri)
8. [Geliştirme](#geliştirme)
9. [Sorun giderme](#sorun-giderme)

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
| `./run.sh` | TOY, parkur dünyası, NVIDIA GPU varsa onunla |
| `./run.sh talon` | Başka bir model (`talon`, `mini_talon`, `x8`, `skywalker1900`, `toy`) |
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

## Çok araçlı simülasyon

Aynı dünyada birden fazla uçak (her biri ayrı PX4 süreci):

```bash
./multi.sh                         # 5 toy + 5 advanced_plane, Gazebo penceresiyle
./multi.sh --headless              # pencere yok, çok araçta önerilir
./multi.sh --vehicles "talon:2,x8:2,mini_talon:2,skywalker1900:2,toy:2"   # karışık filo
./multi.sh --toy 3 --plane 2       # kısa yol: 3 toy + 2 advanced_plane (toplam en fazla 10)
```

Başka bir terminalde hepsini birlikte kaldırmak için:

```bash
python3 scripts/swarm_test.py 10   # kalkış irtifaları 20, 28, 36 ... m (havada çarpışmasınlar)
```

| Araç | PX4 örneği | Sistem id | pymavlink portu | Gazebo modeli |
|---|---|---|---|---|
| 1. araç | 0 | 1 | `udp:127.0.0.1:14540` | `toy_0` |
| 2. araç | 1 | 2 | `udp:127.0.0.1:14541` | `toy_1` |
| ... | i | i+1 | `udp:127.0.0.1:(14540+i)` | `<model>_i` |

- Uçaklar pistin kuzeyinde (y = 6 … 60 m), 6 m arayla, doğuya bakarak dizilir.
- QGroundControl hepsini UDP 14550'den görür.
- Loglar `logs/px4_<i>.log`; Ctrl+C hepsini kapatır.
- PX4'ün port düzeni en fazla 10 aracı ayrı portlarla destekler.

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
| `swarm_test.py [n] [süre]` | `multi.sh` ile açılan n aracı kademeli irtifalara birlikte kaldırır |

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
| 10 araç (5 toy + 5 advanced_plane), pencereli | ort. 0.79 (en düşük 0.31) | %230 |
| 10 araç, `--headless` | ort. 0.94 (anlık düşüşler var) | %198 |
| 10 araç karışık (her modelden 2), `--headless` | 1.00 | — |

10 araçlı testte 10 uçağın 10'u da kalkıp hedef irtifasına oturdu. Simülasyon gerçek zamandan
yavaş koşsa da PX4 Gazebo'nun saatine kilitli (lockstep) olduğu için uçuş doğru kalır; sadece
yavaş ilerler.

Hafifletmeler: ağaçlar tek mesh (renk başına bir çizim çağrısı) ve çarpışmasız; kamera
1280×960 @ 30 Hz yerine 640×480 @ 20 Hz.

**Çok araçlı simülasyon için:** `--headless` kullanın (GUI'nin işlemci yükü gider) ve gerekmeyen
araçlarda kamera kullanmayın; her kamera sahneyi bir kez daha çizer.

---

## Uçak modelleri

| Model | `run.sh` adı | Airframe | Açıklık | Kütle | Yapı | Trim hızı |
|---|---|---|---|---|---|---|
| TOY (ekibin uçağı) | `toy` | 22000 | 1.40 m | 2.75 kg | klasik kuyruk, itici | 18 m/s |
| X-UAV Talon | `talon` | 22001 | 1.72 m | 2.8 kg | V-kuyruk, itici | 15 m/s |
| X-UAV Mini Talon | `mini_talon` | 22002 | 1.30 m | 1.6 kg | V-kuyruk, itici | 15 m/s |
| Skywalker X8 | `x8` | 22003 | 2.12 m | 3.2 kg | uçan kanat, elevon, itici | 18 m/s |
| Skywalker 1900 | `skywalker1900` | 22004 | 1.90 m | 2.0 kg | T-kuyruk, itici | 13 m/s |

Her model manevra testinden geçti (`scripts/maneuver_test.py`: kalkış, 40° yatışlı sert dönüşler,
±15 m irtifa değişiklikleri, irtifa kaybı yok) ve 10 araçlı karışık filoda birlikte uçtu.

### Hazır modeller nasıl üretildi

Talon, Mini Talon, X8 ve Skywalker 1900 **parametrik üreticiyle** (`tools/gen_plane.py`) üretilir.
Ölçüler `tools/plane_specs.py` dosyasındadır:

- **Yayınlanan değerler** (satıcı sayfaları): açıklık, uzunluk, kanat alanı, uçuş ağırlığı, kuyruk tipi.
  X8'in ataleti ölçülmüş değerdir (Løw-Hansen vd., CEAS Aeronautical Journal 2025).
- **Tahmin edilenler** (fotoğraf oranlarından): kiriş dağılımı, kuyruk boyutları, gövde kesitleri,
  motor konumu ve itkisi. Skywalker 1900'ün motor konumu kaynaklarda net değil (itici varsayıldı).

Üretici tek tanımdan şunları çıkarır:

1. Mesh'ler (kontrol yüzeyleri ayrı, hareketli parça) ve AVL geometrisi.
2. AVL ile nötr nokta → ağırlık merkezi statik marja göre (klasik %10, uçan kanat %6 MAC).
   X8 için hesaplanan ağırlık merkezi (burundan 443 mm) üreticinin verdiği 430–440 mm ile örtüşüyor.
3. Seyir hızında kumandasız denge için kuyruk açısı (uçan kanatta uç bükümü) ayarı.
4. AdvancedLiftDrag katsayıları; her kontrol yüzeyi ayrı hesaplanır.
5. PX4 airframe: kontrol tahsisi AVL türevlerinden, hızlar modelin kendi stall hızından,
   açısal hız kazançları TOY'a göre kontrol gücü oranından.

Bir ölçüyü değiştirmek ya da yeni model eklemek için `tools/plane_specs.py`'yi düzenleyin, sonra:

```bash
python3 tools/gen_plane.py talon        # ya da --all
./install.sh ~/PX4-Autopilot            # yeni model eklendiyse
```

Notlar:
- **X8** gerçekte elden/katapultla atılır. Simülasyonda pistten kalkabilmesi için burnunun altında
  küçük bir kızak var (yerde ~7° burun yukarı oturur).
- **X8**'in dönüşlerde yatışı sınırı (35°) 50°'ye kadar aşıyor; yatış kazançları daha ayarlanabilir.
- AVL yalnızca NACA 4 haneli profilleri destekler; X8'in reflex profili yerine NACA 0012 kullanıldı.

### TOY

| Özellik | Değer | Kaynak |
|---|---|---|
| Kanat açıklığı / alan / MAC | 1.40 m / 0.308 m² / 0.222 m | CAD (`cad/1-Montaj.STEP`) |
| Profil / kanat hücum açısı | Clark Y (~NACA 3412) / +3.2° | CAD |
| Kütle / atalet | 2.75 kg / Ixx 0.15, Iyy 0.14, Izz 0.27 kg·m² | tahmin |
| Ağırlık merkezi | kanat kök hücum kenarından 80 mm geride | ekip |
| Motor | itici, 12" pervane, 4S, ~29 N statik itki | tahmin |
| Kontrol yüzeyleri | aileron (dış yarı, %25 kiriş), elevatör/rudder (%30 kiriş), ±25° | varsayım |
| Aerodinamik | AVL 3.36 ile hesaplandı | `models/toy/avl/` |

TOY elle kuruldu (CAD mesh'li); üretici kullanılmadı. Uçuş ayarları (`airframes/22000_gz_toy`):
hızlar stall 12 / min 15 / trim 18 / max 26 m/s, trim gazı 0.31, yatış sınırı 40°.

> ⚠️ **TOY'un ağırlık merkezi:** AVL'ye göre statik marj yalnızca **~%1 MAC** (gövde modellenmedi;
> gerçekte daha düşük olabilir). Simülasyonda PX4 bunu telafi ediyor. Gerçek uçakta ilk uçuştan
> önce ağırlık merkezini öne alın (hücum kenarından ~6–6.5 cm).

---

## Geliştirme

### Repo yapısı

```
run.sh                     tek araç simülasyonu (GPU seçimi dahil)
multi.sh                   çok araçlı simülasyon
install.sh, uninstall.sh   PX4'e bağlama / geri alma
airframes/220xx_gz_<ad>    PX4 airframe'leri (parametreler, kontrol tahsisi)
models/toy/                TOY: model.sdf, CAD mesh'leri, avl/
models/<ad>/               üretilen modeller (talon, mini_talon, x8, skywalker1900)
models/toy_ground/         parkur zemini, pist, ağaçlar (üretilir)
worlds/toy_parkur.sdf      parkur dünyası (üretilir)
tools/gen_world.py         parkuru üreten betik
tools/gen_plane.py         uçak modeli üreticisi (mesh + AVL + SDF + airframe)
tools/plane_specs.py       model ölçüleri
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

### TOY'un aerodinamik katsayılarını yeniden hesaplamak (AVL)

Kontrol yüzeyi ölçüleri veya ağırlık merkezi değişince (diğer modeller için `tools/gen_plane.py`):

```bash
nano models/toy/avl/toy.avl                   # geometriyi düzenle
models/toy/avl/run_avl.sh ~/Avl/bin/avl       # AVL'yi çalıştır
python3 models/toy/avl/avl_to_sdf.py          # model.sdf'yi güncelle, statik marjı yazdır
```

AVL 3.36: <https://web.mit.edu/drela/Public/web/avl/> (gfortran ile derlenir; ayrıntı için
PX4 dokümanı `docs/en/sim_gazebo_gz/tools_avl_automation.md`).

### Yeni araç eklemek

En kolayı: `tools/plane_specs.py`'ye bir tanım ekleyip `python3 tools/gen_plane.py <ad>` çalıştırmak
(klasik kuyruk, T-kuyruk, V-kuyruk ve uçan kanat destekleniyor). Elle eklemek için:

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
| Çok araçta bazı uçaklar kalkmıyor | Kalkış yolunda engel olabilir (uçaklar +x yönünde koşar); `logs/px4_<i>.log` dosyasına bakın. |
| `PX4 server already running` | Önceki simülasyon kapanmamış: `pkill -x px4; pkill -f "gz sim"` |
| Fare ile yakınlaşamıyorum | Takip modunda yakınlaşma çalışmaz: `./run.sh --no-follow` |
| Arm reddediliyor (`No connection to the GCS`) | QGC'yi açın ya da betikten GCS heartbeat gönderin. |
| Kontrol yüzeyleri oynamıyor gibi | Düz uçuşta sapmalar 1–2°'dir. `scripts/maneuver_test.py` ile manevrada görün. |
