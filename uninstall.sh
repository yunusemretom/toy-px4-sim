#!/usr/bin/env bash
# install.sh'in yaptiklarini geri alir: symlink'leri siler, CMakeLists.txt satirlarini kaldirir.
# Kullanim:  ./uninstall.sh /yol/PX4-Autopilot
set -euo pipefail

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PX4="$(cd "${1:?Kullanim: $0 /yol/PX4-Autopilot}" && pwd)"
MODELS_DIR="$PX4/Tools/simulation/gz/models"
WORLDS_DIR="$PX4/Tools/simulation/gz/worlds"
AIRFRAMES_DIR="$PX4/ROMFS/px4fmu_common/init.d-posix/airframes"
CMAKELISTS="$AIRFRAMES_DIR/CMakeLists.txt"

for m in "$REPO"/models/*/; do
	dst="$MODELS_DIR/$(basename "${m%/}")"
	if [[ -L "$dst" ]]; then rm "$dst"; echo "  silindi: $dst"; fi
done

for w in "$REPO"/worlds/*.sdf; do
	dst="$WORLDS_DIR/$(basename "$w")"
	if [[ -L "$dst" ]]; then rm "$dst"; echo "  silindi: $dst"; fi
done

for a in "$REPO"/airframes/*; do
	name="$(basename "$a")"
	dst="$AIRFRAMES_DIR/$name"
	if [[ -L "$dst" ]]; then rm "$dst"; echo "  silindi: $dst"; fi
	if grep -q -E "^[[:space:]]*${name}[[:space:]]*$" "$CMAKELISTS"; then
		sed -i -E "/^[[:space:]]*${name}[[:space:]]*$/d" "$CMAKELISTS"
		echo "  CMakeLists.txt: $name kaldirildi"
	fi
done
