# toy-px4-sim

TOY itici (pusher) sabit kanat uçağının PX4 SITL + Gazebo simülasyon modeli.

PX4'ün kendisi bu repoda yok: PX4-Autopilot'u herkes kendi bilgisayarına normal şekilde kurar,
bu repo da modeli ve airframe dosyasını o PX4 kopyasına **symlink** ile bağlar.
Bu repoda `git pull` yapınca PX4 tarafındaki model de güncellenir.


## Gereksinimler

- Ubuntu 22.04 (veya 24.04)
- PX4-Autopilot ve geliştirme ortamı (Gazebo Harmonic dahil):
  ```bash
  git clone https://github.com/PX4/PX4-Autopilot.git --recursive
  bash ./PX4-Autopilot/Tools/setup/ubuntu.sh
  ```
  Kurulumdan sonra oturumu kapatıp açın. PX4'ün kendi örneğiyle test edin: `make px4_sitl gz_x500`
- Test betikleri için: `pip install -r requirements.txt`
- (İsteğe bağlı) QGroundControl

## Kurulum

```bash
git clone https://github.com/yunusemretom/toy-px4-sim.git
cd toy-px4-sim
./install.sh ~/PX4-Autopilot
```

`install.sh` şunları yapar:

| Bu repoda | PX4'te (symlink) |
|---|---|
| `models/toy/` | `Tools/simulation/gz/models/toy` |
| `airframes/22000_gz_toy` | `ROMFS/px4fmu_common/init.d-posix/airframes/22000_gz_toy` |

ve `22000_gz_toy` adını PX4'ün `ROMFS/px4fmu_common/init.d-posix/airframes/CMakeLists.txt` listesine
ekler (PX4'ün airframe'i derlemeye dahil etmesi için gerekli). PX4 reposunda bu dosya "değişmiş"
görünür; geri almak için `./uninstall.sh ~/PX4-Autopilot`.

## Çalıştırma

```bash
cd ~/PX4-Autopilot
make px4_sitl gz_toy
```

- Görüntüsüz (CI / uzak makine): `HEADLESS=1 make px4_sitl gz_toy`
- Serbest kamera (fare ile yakınlaştırma için takip modunu kapat): `PX4_GZ_NO_FOLLOW=1 make px4_sitl gz_toy`
- QGroundControl otomatik bağlanır (UDP 14550).

## Uçağa programla bağlanmak

PX4 SITL portları: QGC `14550`, offboard / API `14540`, ROS 2 uXRCE-DDS ajanı `8888`.

```bash
# Kalkış, 25 m'de loiter, irtifa/hız takibi, RTL
python3 scripts/fw_test.py udp:127.0.0.1:14540 90

# Kontrol yüzeylerini sırayla ±25° oynat (uçak DISARM iken)
python3 scripts/surf_test.py 3

# Burun kamerasından tek kare kaydet
python3 scripts/grab_cam.py kare.png
```

Not: PX4 bir GCS bağlantısı görmeden arm etmez. QGC açık değilse betik `MAV_TYPE_GCS`
heartbeat'i göndermelidir (`fw_test.py` bunu yapıyor).

## Kamera

Burunda 15° aşağı bakan kamera (PX4'ün `mono_cam` modeli, 1280×960, 30 Hz).

- Video akışı: UDP 5600 (RTP/H.264). QGC → Application Settings → General → Video Source: *UDP h.264*, port 5600.
- Gazebo konusu: `/world/default/model/toy_0/link/camera_link/sensor/camera/image` (ROS 2 için `ros_gz_bridge`).

## Model

| | Değer | Kaynak |
|---|---|---|
| Kanat açıklığı / alan / MAC | 1.40 m / 0.308 m² / 0.222 m | CAD (`cad/1-Montaj.STEP`) |
| Kanat profili / hücum açısı | Clark Y (~NACA 3412) / +3.2° | kullanıcı / CAD |
| Kütle | 2.75 kg | tahmin (2.5–3 kg) |
| Atalet | Ixx 0.15, Iyy 0.14, Izz 0.27 kg·m² | tahmin |
| Ağırlık merkezi | kanat kök hücum kenarından 80 mm geride | kullanıcı |
| Motor | itici, 12" pervane, 4S, ~29 N statik itki | tahmin |
| Kontrol yüzeyleri | aileron (dış yarı, %25 kiriş), elevatör ve rudder (%30 kiriş), ±25° | VARSAYIM |
| Aerodinamik katsayılar | AVL 3.36 | `models/toy/avl/` |

SITL'de: 16 m/s trim hızında ~%28 gaz, kararlı loiter.

### ⚠️ Ağırlık merkezi uyarısı

AVL'ye göre nötr nokta burundan 411.7 mm, ağırlık merkezi 409.3 mm → **statik marj ~%1 MAC**
(gövde modellenmedi, gerçekte daha düşük olabilir). Simülasyonda PX4 bunu telafi ediyor; gerçek
uçakta ilk uçuştan önce ağırlık merkezini öne (hücum kenarından ~6–6.5 cm) almak önerilir.

### Aerodinamik katsayıları yeniden üretmek (AVL)

Kontrol yüzeyi ölçüleri, ağırlık merkezi vb. değişince:

```bash
# AVL 3.36: https://web.mit.edu/drela/Public/web/avl/ (gfortran ile derlenir, bkz. PX4 dokümanı
# docs/en/sim_gazebo_gz/tools_avl_automation.md)
nano models/toy/avl/toy.avl                    # geometriyi düzenle
models/toy/avl/run_avl.sh ~/Avl/bin/avl        # toy_st.txt / toy_sb.txt üretir
python3 models/toy/avl/avl_to_sdf.py           # model.sdf'deki katsayıları günceller
```

`avl_to_sdf.py` statik marjı da yazdırır. İşaret kuralları dosyanın başında açıklanmıştır.

## Repo yapısı

```
airframes/22000_gz_toy     PX4 airframe (parametreler, kontrol tahsisi, servo/motor eşleme)
models/toy/model.sdf      Gazebo modeli (kütle, sensörler, motor, AdvancedLiftDrag, kamera)
models/toy/meshes/        CAD'den üretilen parça mesh'leri (gövde ekseni, mm)
models/toy/avl/           AVL geometrisi ve SDF dönüştürücü
scripts/                  pymavlink test betikleri
cad/1-Montaj.STEP         kaynak CAD montajı
install.sh / uninstall.sh PX4'e bağlama / geri alma
```

## Yeni model eklemek

1. `models/<ad>/` altına `model.sdf` + `model.config` koy.
2. `airframes/<boş-id>_gz_<ad>` dosyası oluştur (`PX4_SIM_MODEL=${PX4_SIM_MODEL:=<ad>}`).
   PX4, özel modeller için 22000–22999 aralığını ayırmıştır; çakışma olmaması için bu aralığı kullanın.
3. `./install.sh ~/PX4-Autopilot` ve `make px4_sitl gz_<ad>`.
