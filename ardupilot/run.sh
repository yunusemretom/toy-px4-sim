#!/usr/bin/env bash
# ArduPilot (ArduPlane SITL) + Gazebo ile bir ya da birden fazla ucak baslatir.
#
# Kullanim:  ./ardupilot/run.sh [model] [secenekler]
#   model                      tek arac: toy | talon | mini_talon | x8 | skywalker1900 (varsayilan toy)
#   --vehicles "talon:2,x8:1"  cok arac (toplam en fazla 10)
#   --headless                 Gazebo penceresi acmadan
#   --no-follow                kamera ilk ucagi takip etmesin (fare ile serbest kamera)
#   --world <ad>               dunya (varsayilan toy_parkur_ap)
#   --no-nvidia                NVIDIA'yi zorlama
#
# Arac i (0'dan baslar): MAVLink TCP 5760+10*i (pymavlink: tcp:127.0.0.1:5760), sistem id i+1,
#   QGroundControl'e UDP 14550 (otomatik). Gazebo modeli <model>_ap_<i>, JSON portu 9002+10*i.
# Loglar ve ArduPilot calisma klasorleri: logs/ap/. Ctrl+C hepsini kapatir.
set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO="$(cd "$HERE/.." && pwd)"
source "$REPO/tools/gpu_env.sh"
[[ -f "$HERE/.ap_paths" ]] && source "$HERE/.ap_paths"
AP="${ARDUPILOT_DIR:-$HOME/ardupilot}"
APGZ="${ARDUPILOT_GAZEBO_DIR:-$HOME/ardupilot_gazebo}"
BIN="$AP/build/sitl/bin/arduplane"
[[ -x "$BIN" ]] || { echo "HATA: $BIN yok. Once ./ardupilot/setup.sh calistirin." >&2; exit 1; }
[[ -f "$APGZ/build/libArduPilotPlugin.so" ]] || { echo "HATA: ardupilot_gazebo derlenmemis (setup.sh)." >&2; exit 1; }

SPEC=""
WORLD=toy_parkur_ap
NVIDIA=1
HEADLESS=0
FOLLOW=1
while [[ $# -gt 0 ]]; do
	case "$1" in
		--vehicles) SPEC="$2"; shift ;;
		--world) WORLD="$2"; shift ;;
		--headless) HEADLESS=1 ;;
		--no-follow) FOLLOW=0 ;;
		--no-nvidia) NVIDIA=0 ;;
		-h|--help) sed -n '2,14p' "$0"; exit 0 ;;
		-*) echo "Bilinmeyen secenek: $1" >&2; exit 1 ;;
		*) SPEC="$1:1" ;;
	esac
	shift
done
SPEC="${SPEC:-toy:1}"

export GZ_SIM_SYSTEM_PLUGIN_PATH="$APGZ/build:${GZ_SIM_SYSTEM_PLUGIN_PATH:-}"
export GZ_SIM_RESOURCE_PATH="$REPO/models:$REPO/worlds:$APGZ/models:$APGZ/worlds:${GZ_SIM_RESOURCE_PATH:-}"
use_nvidia_if_available "$NVIDIA"

# dunyanin konumu (ArduPilot home): spherical_coordinates
WFILE="$REPO/worlds/$WORLD.sdf"
LAT=$(grep -o '<latitude_deg>[^<]*' "$WFILE" | cut -d'>' -f2 | tr -d ' ')
LON=$(grep -o '<longitude_deg>[^<]*' "$WFILE" | cut -d'>' -f2 | tr -d ' ')
ALT=$(grep -o '<elevation>[^<]*' "$WFILE" | cut -d'>' -f2 | tr -d ' ')

LOGS="$REPO/logs/ap"
mkdir -p "$LOGS"
PIDS=()
GZ_SERVER_PAT="gz sim -r -s -v1 $WFILE"
GZ_GUI_PAT="gz sim -g -v1"

# Bu dunyanin Gazebo sunucusunu/arayuzunu ve ArduPilot SITL'i kapatir. 'gz sim' asil simulasyonu
# alt surec olarak baslattigi icin yalnizca $! PID'ini oldurmek yetmez; desene gore kapatilir,
# kapanmayanlar birkac saniye sonra zorla (-9) kapatilir.
stop_all() {
	pkill -f "$GZ_SERVER_PAT" 2>/dev/null || true
	pkill -f "$GZ_GUI_PAT" 2>/dev/null || true
	pkill -x arduplane 2>/dev/null || true
	for _ in $(seq 10); do
		pgrep -f "$GZ_SERVER_PAT" >/dev/null || pgrep -f "$GZ_GUI_PAT" >/dev/null || pgrep -x arduplane >/dev/null || return 0
		sleep 0.5
	done
	pkill -9 -f "$GZ_SERVER_PAT" 2>/dev/null || true
	pkill -9 -f "$GZ_GUI_PAT" 2>/dev/null || true
	pkill -9 -x arduplane 2>/dev/null || true
}

CLEANED=0
cleanup() {
	[[ $CLEANED == 1 ]] && return
	CLEANED=1
	echo; echo "Kapatiliyor..."
	for p in "${PIDS[@]}"; do kill "$p" 2>/dev/null || true; done
	stop_all
}
trap cleanup EXIT INT TERM

# Onceki bir calistirmadan kalan sureçler ayni dunya adiyla cakisir (servis zaman asimi, ArduPilot
# "No JSON sensor message received"): once temizle.
if pgrep -f "$GZ_SERVER_PAT" >/dev/null || pgrep -x arduplane >/dev/null; then
	echo "Onceki calistirmadan kalan Gazebo/ArduPilot surecleri kapatiliyor..."
	stop_all
fi

gz sim -r -s -v1 "$WFILE" > "$LOGS/gz_server.log" 2>&1 &
PIDS+=($!)
if [[ $HEADLESS == 0 ]]; then
	gz sim -g -v1 > "$LOGS/gz_gui.log" 2>&1 &
	PIDS+=($!)
fi
timeout 60 bash -c "until gz service -l 2>/dev/null | grep -q '/world/$WORLD/create'; do sleep 1; done" \
	|| { echo "HATA: Gazebo dunyasi acilmadi, bkz. $LOGS/gz_server.log" >&2; exit 1; }

i=-1
SPAWNED=""
IFS=',' read -ra PARTS <<< "$SPEC"
for part in "${PARTS[@]}"; do
	m="${part%%:*}"; n="${part##*:}"
	src="$REPO/models/${m}_ap/model.sdf"
	[[ -f "$src" ]] || { echo "HATA: $src yok (python3 tools/gen_ardupilot.py $m)" >&2; exit 1; }
	z=$(grep -m1 -o '<pose>[^<]*</pose>' "$src" | awk '{print $3}')
	for ((k = 0; k < n; k++)); do
		i=$((i + 1))
		(( i < 10 )) || { echo "HATA: en fazla 10 arac" >&2; exit 1; }
		name="${m}_ap_$i"
		port=$((9002 + 10 * i))
		y=$((6 * (i + 1)))
		sdf="$LOGS/$name.sdf"
		# model adi, konu adlari ve JSON portu bu ornege gore
		sed -e "s#<model name=\"${m}_ap\">#<model name=\"$name\">#" \
			-e "s#/model/${m}_ap/#/model/$name/#g" \
			-e "s#<namespace>${m}_ap</namespace>#<namespace>$name</namespace>#" \
			-e "s#<fdm_port_in>9002</fdm_port_in>#<fdm_port_in>$port</fdm_port_in>#" "$src" > "$sdf"
		gz service -s "/world/$WORLD/create" --reqtype gz.msgs.EntityFactory --reptype gz.msgs.Boolean \
			--timeout 5000 --req "sdf_filename: \"$sdf\", name: \"$name\", allow_renaming: false, \
			pose: {position: {x: 0, y: $y, z: $z}}" > /dev/null
		# --serial0 tcp:0: MAVLink istemcisi beklemeden basla (varsayilan tcp:0:wait). Kilitli adimda
		# Gazebo tum araclardan veri bekledigi icin, bir arac baglanti beklerse hepsi donar.
		wd="$LOGS/$name"
		mkdir -p "$wd"
		echo "SYSID_THISMAV $((i + 1))" > "$wd/sysid.parm"
		(cd "$wd" && exec "$BIN" --model JSON:127.0.0.1 --speedup 1 -I "$i" \
			--home "$LAT,$LON,$ALT,90" --serial0 tcp:0 --serial1 udpclient:127.0.0.1:14550 \
			--defaults "$AP/Tools/autotest/models/plane.parm,$REPO/ardupilot/params/$m.parm,$wd/sysid.parm" \
			> "$wd/arduplane.log" 2>&1) &
		PIDS+=($!)
		SPAWNED+="[$i] $m"$'\n'
		echo "[$i] $m  y=$y m  MAVLink tcp:127.0.0.1:$((5760 + 10 * i))  JSON $port"
	done
done
# Gazebo kamerasi ilk ucagi takip etsin (PX4'te bunu PX4'un baslangic betigi yapiyor).
if [[ $HEADLESS == 0 && $FOLLOW == 1 && $i -ge 0 ]]; then
	first=$(sed -n 's/^\[0\] \([a-z0-9_]*\).*/\1/p' <<< "$SPAWNED")
	for _ in $(seq 10); do
		gz topic -t /gui/track -m gz.msgs.CameraTrack -p "track_mode: FOLLOW, follow_target: {name: '${first}_ap_0'}, \
follow_offset: {x: -1.8, y: -1.4, z: 0.8}, follow_pgain: 1.0, track_pgain: 1.0" 2>/dev/null
		sleep 1
	done &
fi

echo
if (( i == 0 )); then
	echo "1 arac calisiyor. Kalkis + manevra testi: python3 scripts/ap_test.py maneuver"
else
	echo "$((i + 1)) arac calisiyor. Suru kalkis testi: python3 scripts/ap_test.py swarm $((i + 1))"
fi
echo "Kapatmak icin Ctrl+C."
wait
