#!/usr/bin/env bash
# Tek arac simulasyonu baslatir. NVIDIA kart varsa Gazebo'yu ona yonlendirir.
#
# Kullanim:  ./run.sh [model] [secenekler]
#   model          airframe adi, varsayilan: toy   (make px4_sitl gz_<model>)
#   --headless     Gazebo penceresi acmadan
#   --world <ad>   dunya (varsayilan airframe'deki: toy_parkur; bos dunya: default)
#   --no-follow    kamera ucagi takip etmesin (fare ile serbest kamera)
#   --no-nvidia    NVIDIA'yi zorlama
# Cok arac icin: ./multi.sh
set -euo pipefail

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$REPO/tools/gpu_env.sh"
PX4="$(find_px4 "$REPO")"

MODEL=toy
NVIDIA=1
while [[ $# -gt 0 ]]; do
	case "$1" in
		--headless) export HEADLESS=1 ;;
		--world) export PX4_GZ_WORLD="$2"; shift ;;
		--no-follow) export PX4_GZ_NO_FOLLOW=1 ;;
		--no-nvidia) NVIDIA=0 ;;
		-h|--help) sed -n '2,10p' "$0"; exit 0 ;;
		*) MODEL="$1" ;;
	esac
	shift
done

use_nvidia_if_available "$NVIDIA"
cd "$PX4"
exec make px4_sitl "gz_$MODEL"
