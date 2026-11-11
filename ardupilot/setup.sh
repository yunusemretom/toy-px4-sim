#!/usr/bin/env bash
# ArduPilot (ArduPlane SITL) ve ardupilot_gazebo eklentisini indirir ve derler.
#
# Kullanim:  ./ardupilot/setup.sh [hedef_klasor]      (varsayilan: ~/)
#
# Once (bir kez, sudo gerekir):
#   sudo apt install rapidjson-dev libgz-sim8-dev libopencv-dev libgstreamer1.0-dev \
#        libgstreamer-plugins-base1.0-dev gstreamer1.0-plugins-bad gstreamer1.0-plugins-ugly \
#        gstreamer1.0-libav gstreamer1.0-gl
#   pip install --user empy==3.3.4 pexpect future pymavlink
#
# Klasor yollari ardupilot/.ap_paths dosyasina yazilir; ardupilot/run.sh oradan okur.
set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
DEST="$(cd "${1:-$HOME}" && pwd)"
AP="$DEST/ardupilot"
APGZ="$DEST/ardupilot_gazebo"

if [[ ! -d "$AP" ]]; then
	git clone --depth 1 --recurse-submodules --shallow-submodules https://github.com/ArduPilot/ardupilot.git "$AP"
fi
(cd "$AP" && ./waf configure --board sitl && ./waf plane)

if [[ ! -d "$APGZ" ]]; then
	git clone --depth 1 https://github.com/ArduPilot/ardupilot_gazebo.git "$APGZ"
fi
# CMake 4 ile sistemdeki eski jsoncpp yapilandirmasi uyumsuz: CMAKE_POLICY_VERSION_MINIMUM gerekli.
# rapidjson ~/.local altina kurulduysa (sudo'suz) CMAKE_PREFIX_PATH ve include yolu da eklenir.
GZ_VERSION=harmonic cmake -S "$APGZ" -B "$APGZ/build" -DCMAKE_BUILD_TYPE=RelWithDebInfo \
	-DCMAKE_POLICY_VERSION_MINIMUM=3.5 -DCMAKE_PREFIX_PATH="$HOME/.local" \
	-DCMAKE_CXX_FLAGS="-I$HOME/.local/include"
cmake --build "$APGZ/build" -j"$(nproc)"

printf 'ARDUPILOT_DIR=%s\nARDUPILOT_GAZEBO_DIR=%s\n' "$AP" "$APGZ" > "$HERE/.ap_paths"
echo "Tamam. Calistirmak icin: ./ardupilot/run.sh talon"
