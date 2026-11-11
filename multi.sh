#!/usr/bin/env bash
# Ayni Gazebo dunyasinda birden fazla arac baslatir (her biri ayri PX4 sureci).
#
# Kullanim:  ./multi.sh [secenekler]
#   --vehicles "<model>:<adet>,..."   ornek: "talon:2,x8:2,mini_talon:2,skywalker1900:2,toy:2"
#                  model: bu repodaki airframes/<id>_gz_<model> ya da PX4'un advanced_plane'i
#                  (varsayilan "toy:5,advanced_plane:5")
#   --toy <n> / --plane <n>   kisa yol: n toy / n advanced_plane (--vehicles yerine)
#   --world <ad>   dunya (varsayilan toy_parkur)
#   --headless     Gazebo penceresi acmadan (cok aracta onerilir)
#   --no-nvidia    NVIDIA'yi zorlama
#
# Araclar pistin kuzeyinde, 6 m arayla, doguya (+x) bakarak tek sira dizilir (y = 6, 12, ...).
#   Bu serit (y = 6..60) bina ve agactan temizdir; guneyde hangar kalkis yoluna denk gelir.
# Arac i (0'dan baslar): MAVLink sistem id = i+1, pymavlink/MAVSDK portu udp:127.0.0.1:(14540+i),
#   QGroundControl hepsini UDP 14550'den gorur. Gazebo model adi: <model>_<i>.
#   PX4'un port duzeni en fazla 10 araci (i = 0..9) ayri portlarla destekler.
# Loglar: logs/px4_<i>.log.  Ctrl+C hepsini kapatir.
set -euo pipefail

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$REPO/tools/gpu_env.sh"
PX4="$(find_px4 "$REPO")"

SPEC="toy:5,advanced_plane:5"
N_TOY=""
N_PLANE=""
WORLD=toy_parkur
NVIDIA=1
while [[ $# -gt 0 ]]; do
	case "$1" in
		--vehicles) SPEC="$2"; shift ;;
		--toy) N_TOY="$2"; shift ;;
		--plane) N_PLANE="$2"; shift ;;
		--world) WORLD="$2"; shift ;;
		--headless) export HEADLESS=1 ;;
		--no-nvidia) NVIDIA=0 ;;
		-h|--help) sed -n '2,20p' "$0"; exit 0 ;;
		*) echo "Bilinmeyen secenek: $1" >&2; exit 1 ;;
	esac
	shift
done
if [[ -n "$N_TOY$N_PLANE" ]]; then
	SPEC="toy:${N_TOY:-0},advanced_plane:${N_PLANE:-0}"
fi

# model -> "autostart z" (z: yerden dogma yuksekligi, model.sdf'deki ust <pose>'dan)
model_info() {
	local m="$1" af
	if [[ "$m" == advanced_plane ]]; then echo "4008 0.25"; return; fi
	af=$(ls "$REPO"/airframes/*_gz_"$m" 2>/dev/null | head -1)
	[[ -n "$af" ]] || { echo "HATA: '$m' icin airframe yok (airframes/*_gz_$m)" >&2; return 1; }
	local id z
	id=$(basename "$af" | cut -d_ -f1)
	z=$(grep -m1 -o '<pose>[^<]*</pose>' "$REPO/models/$m/model.sdf" | awk '{print $3}')
	echo "$id ${z:-0.15}"
}

# arac listesi: "autostart model z y"
VEHICLES=()
slot=0
IFS=',' read -ra PARTS <<< "$SPEC"
for part in "${PARTS[@]}"; do
	m="${part%%:*}"; n="${part##*:}"
	[[ "$n" =~ ^[0-9]+$ ]] || { echo "HATA: '$part' (beklenen <model>:<adet>)" >&2; exit 1; }
	read -r id z <<< "$(model_info "$m")"
	for ((k = 0; k < n; k++)); do
		slot=$((slot + 1))
		VEHICLES+=("$id $m $z $((6 * slot))")
	done
done
if (( ${#VEHICLES[@]} == 0 || ${#VEHICLES[@]} > 10 )); then
	echo "HATA: 1-10 arac olmali (PX4 MAVLink portlari 14540..14549), istenen: ${#VEHICLES[@]}" >&2
	exit 1
fi

use_nvidia_if_available "$NVIDIA"
BIN="$PX4/build/px4_sitl_default/bin/px4"
LOGS="$REPO/logs"
mkdir -p "$LOGS"

echo "PX4 derleniyor (gerekirse)..."
make -C "$PX4" px4_sitl_default > "$LOGS/build.log" 2>&1 || { echo "Derleme hatasi, bkz. $LOGS/build.log"; exit 1; }

PIDS=()
cleanup() {
	echo; echo "Kapatiliyor..."
	for p in "${PIDS[@]}"; do kill "$p" 2>/dev/null || true; done
	pkill -f "gz sim" 2>/dev/null || true
}
trap cleanup EXIT INT TERM

i=-1
for v in "${VEHICLES[@]}"; do
	read -r autostart model z y <<< "$v"
	i=$((i + 1))
	env_common=(PX4_SYS_AUTOSTART="$autostart" PX4_SIM_MODEL="gz_$model" PX4_GZ_MODEL_POSE="0,$y,$z,0,0,0" PX4_GZ_WORLD="$WORLD")
	if [[ $i -eq 0 ]]; then
		# ilk arac Gazebo sunucusunu (ve GUI'yi) baslatir
		env "${env_common[@]}" "$BIN" -d -i "$i" > "$LOGS/px4_$i.log" 2>&1 &
		PIDS+=($!)
		echo "[$i] $model y=$y  (Gazebo baslatiliyor)"
		timeout 120 bash -c "until grep -q 'Startup script returned' '$LOGS/px4_0.log'; do sleep 1; done" \
			|| { echo "Ilk arac baslamadi, bkz. $LOGS/px4_0.log"; exit 1; }
	else
		env "${env_common[@]}" PX4_GZ_STANDALONE=1 "$BIN" -d -i "$i" > "$LOGS/px4_$i.log" 2>&1 &
		PIDS+=($!)
		echo "[$i] $model y=$y"
		sleep 2
	fi
done

echo
echo "$((i + 1)) arac calisiyor. Arac i icin pymavlink: udp:127.0.0.1:(14540+i). Hepsini kaldirmak icin:"
echo "  python3 scripts/swarm_test.py $((i + 1))"
echo "Kapatmak icin Ctrl+C."
wait
