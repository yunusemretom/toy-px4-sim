#!/usr/bin/env bash
# Ayni Gazebo dunyasinda birden fazla arac baslatir (her biri ayri PX4 sureci).
#
# Kullanim:  ./multi.sh [secenekler]
#   --toy <n>      toy sayisi (varsayilan 5)
#   --plane <n>    PX4 advanced_plane sayisi (varsayilan 5)
#   --world <ad>   dunya (varsayilan toy_parkur)
#   --headless     Gazebo penceresi acmadan (cok aracta onerilir)
#   --no-nvidia    NVIDIA'yi zorlama
#
# Araclar pistin kuzeyinde, 6 m arayla, doguya (+x) bakarak tek sira dizilir:
#   once toy'lar (y = 6, 12, ...), sonra advanced_plane'ler. Bu serit (y = 6..60) bina ve agactan
#   temizdir; guneyde hangar (y -24..-40) kalkis yoluna denk gelir.
# Arac i (0'dan baslar): MAVLink sistem id = i+1, pymavlink/MAVSDK portu udp:127.0.0.1:(14540+i),
#   QGroundControl hepsini UDP 14550'den gorur. Gazebo model adi: toy_<i> / advanced_plane_<i>.
#   PX4'un port duzeni en fazla 10 araci (i = 0..9) ayri portlarla destekler.
# Loglar: logs/px4_<i>.log.  Ctrl+C hepsini kapatir.
set -euo pipefail

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$REPO/tools/gpu_env.sh"
PX4="$(find_px4 "$REPO")"

N_TOY=5
N_PLANE=5
WORLD=toy_parkur
NVIDIA=1
while [[ $# -gt 0 ]]; do
	case "$1" in
		--toy) N_TOY="$2"; shift ;;
		--plane) N_PLANE="$2"; shift ;;
		--world) WORLD="$2"; shift ;;
		--headless) export HEADLESS=1 ;;
		--no-nvidia) NVIDIA=0 ;;
		-h|--help) sed -n '2,16p' "$0"; exit 0 ;;
		*) echo "Bilinmeyen secenek: $1" >&2; exit 1 ;;
	esac
	shift
done
if (( N_TOY + N_PLANE > 10 )); then
	echo "HATA: en fazla 10 arac (PX4 MAVLink portlari 14540..14549)." >&2
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

# arac listesi: "autostart model z y"
VEHICLES=()
for ((k = 1; k <= N_TOY; k++)); do VEHICLES+=("22000 toy 0.12 $((6 * k))"); done
for ((k = 1; k <= N_PLANE; k++)); do VEHICLES+=("4008 advanced_plane 0.25 $((6 * (N_TOY + k)))"); done

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
