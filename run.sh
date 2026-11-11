#!/usr/bin/env bash
# Simulasyonu baslatir. NVIDIA kart varsa Gazebo'yu ona yonlendirir (PRIME offload);
# yoksa (veya --no-nvidia ile) sistemin varsayilan GPU'su kullanilir.
#
# Kullanim:  ./run.sh [model] [secenekler]
#   model          airframe adi, varsayilan: toy   (make px4_sitl gz_<model>)
#   --headless     Gazebo penceresi acmadan
#   --world <ad>   dunya (varsayilan airframe'deki: toy_parkur; bos dunya: default)
#   --no-follow    kamera ucagi takip etmesin (fare ile serbest kamera)
#   --no-nvidia    NVIDIA'yi zorlama
#
# PX4 klasoru: install.sh'in kaydettigi yol (.px4_path) ya da PX4_DIR ortam degiskeni.
set -euo pipefail

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PX4="${PX4_DIR:-$(cat "$REPO/.px4_path" 2>/dev/null || true)}"
if [[ -z "$PX4" || ! -d "$PX4" ]]; then
	echo "HATA: PX4 klasoru bilinmiyor. Once ./install.sh /yol/PX4-Autopilot calistirin ya da PX4_DIR verin." >&2
	exit 1
fi

MODEL=toy
NVIDIA=1
while [[ $# -gt 0 ]]; do
	case "$1" in
		--headless) export HEADLESS=1 ;;
		--world) export PX4_GZ_WORLD="$2"; shift ;;
		--no-follow) export PX4_GZ_NO_FOLLOW=1 ;;
		--no-nvidia) NVIDIA=0 ;;
		-h|--help) sed -n '2,12p' "$0"; exit 0 ;;
		*) MODEL="$1" ;;
	esac
	shift
done

NV_EGL=/usr/share/glvnd/egl_vendor.d/10_nvidia.json
if [[ $NVIDIA == 1 ]] && command -v nvidia-smi >/dev/null && nvidia-smi -L >/dev/null 2>&1 && [[ -f $NV_EGL ]]; then
	# Hibrit (Intel + NVIDIA) dizustulerde Gazebo aksi halde entegre GPU ile cizer.
	export __NV_PRIME_RENDER_OFFLOAD=1
	export __GLX_VENDOR_LIBRARY_NAME=nvidia
	export __EGL_VENDOR_LIBRARY_FILENAMES=$NV_EGL
	echo "GPU: $(nvidia-smi --query-gpu=name --format=csv,noheader | head -1) (NVIDIA)"
else
	echo "GPU: sistem varsayilani"
fi

cd "$PX4"
exec make px4_sitl "gz_$MODEL"
