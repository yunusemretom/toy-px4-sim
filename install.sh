#!/usr/bin/env bash
# toy-px4-sim modellerini ve airframe'lerini bir PX4-Autopilot kopyasina baglar.
#
# Kullanim:  ./install.sh /yol/PX4-Autopilot [--force]
#
# - models/<ad>/          -> PX4/Tools/simulation/gz/models/<ad>          (symlink)
# - airframes/<id>_gz_<ad> -> PX4/ROMFS/px4fmu_common/init.d-posix/airframes/ (symlink)
# - airframe adi PX4'un airframes/CMakeLists.txt listesine eklenir (bir kez).
#
# Symlink kullanildigi icin bu repoda 'git pull' yapmak PX4 tarafini da gunceller.
# Hedefte symlink olmayan gercek bir dosya/klasor varsa --force verilmedikce dokunulmaz.
set -euo pipefail

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PX4="${1:-}"
FORCE="${2:-}"

if [[ -z "$PX4" ]]; then
	echo "Kullanim: $0 /yol/PX4-Autopilot [--force]" >&2
	exit 1
fi
PX4="$(cd "$PX4" && pwd)"

MODELS_DIR="$PX4/Tools/simulation/gz/models"
AIRFRAMES_DIR="$PX4/ROMFS/px4fmu_common/init.d-posix/airframes"
CMAKELISTS="$AIRFRAMES_DIR/CMakeLists.txt"

for p in "$MODELS_DIR" "$AIRFRAMES_DIR" "$CMAKELISTS"; do
	if [[ ! -e "$p" ]]; then
		echo "HATA: $p bulunamadi. PX4-Autopilot klasoru dogru mu? (submodule'ler: git submodule update --init --recursive)" >&2
		exit 1
	fi
done

link() {
	local src="$1" dst="$2"
	if [[ -L "$dst" ]]; then
		ln -sfn "$src" "$dst"
	elif [[ -e "$dst" ]]; then
		if [[ "$FORCE" == "--force" ]]; then
			local backup="$dst.bak.$(date +%Y%m%d%H%M%S)"
			mv "$dst" "$backup"
			echo "  yedeklendi: $backup"
			ln -s "$src" "$dst"
		else
			echo "HATA: $dst zaten var ve symlink degil. Uzerine yazmak icin --force kullanin (yedeklenir)." >&2
			exit 1
		fi
	else
		ln -s "$src" "$dst"
	fi
	echo "  $dst -> $src"
}

echo "Modeller:"
for m in "$REPO"/models/*/; do
	m="${m%/}"
	link "$m" "$MODELS_DIR/$(basename "$m")"
done

echo "Airframe'ler:"
for a in "$REPO"/airframes/*; do
	name="$(basename "$a")"
	link "$a" "$AIRFRAMES_DIR/$name"
	if grep -q -E "^[[:space:]]*${name}[[:space:]]*$" "$CMAKELISTS"; then
		echo "  CMakeLists.txt: $name zaten listede"
	else
		# px4_add_romfs_files( ... ) listesinin kapanis parantezinden once ekle
		python3 - "$CMAKELISTS" "$name" <<'EOF'
import sys
path, name = sys.argv[1], sys.argv[2]
lines = open(path).read().split("\n")
start = next(i for i, l in enumerate(lines) if l.strip().startswith("px4_add_romfs_files("))
end = next(i for i in range(start, len(lines)) if lines[i].strip() == ")")
lines.insert(end, "\t" + name)
open(path, "w").write("\n".join(lines))
EOF
		echo "  CMakeLists.txt: $name eklendi"
	fi
done

echo
echo "Tamam. Calistirmak icin:"
echo "  cd $PX4"
for a in "$REPO"/airframes/*_gz_*; do
	echo "  make px4_sitl gz_$(basename "$a" | sed 's/^[0-9]*_gz_//')"
done
