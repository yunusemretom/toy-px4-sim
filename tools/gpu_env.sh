# run.sh ve multi.sh tarafindan source edilir.
# NVIDIA kart varsa Gazebo'yu ona yonlendirir (PRIME offload); hibrit (Intel + NVIDIA)
# dizustulerde aksi halde Gazebo entegre GPU ile cizer ve kasar.
# Kullanim: use_nvidia_if_available <0|1>
use_nvidia_if_available() {
	local want="${1:-1}"
	local nv_egl=/usr/share/glvnd/egl_vendor.d/10_nvidia.json
	if [[ $want == 1 ]] && command -v nvidia-smi >/dev/null && nvidia-smi -L >/dev/null 2>&1 && [[ -f $nv_egl ]]; then
		export __NV_PRIME_RENDER_OFFLOAD=1
		export __GLX_VENDOR_LIBRARY_NAME=nvidia
		export __EGL_VENDOR_LIBRARY_FILENAMES=$nv_egl
		echo "GPU: $(nvidia-smi --query-gpu=name --format=csv,noheader | head -1) (NVIDIA)"
	else
		echo "GPU: sistem varsayilani"
	fi
}

# PX4 klasorunu bul: PX4_DIR ya da install.sh'in kaydettigi .px4_path
find_px4() {
	local repo="$1"
	local px4="${PX4_DIR:-$(cat "$repo/.px4_path" 2>/dev/null || true)}"
	if [[ -z "$px4" || ! -d "$px4" ]]; then
		echo "HATA: PX4 klasoru bilinmiyor. Once ./install.sh /yol/PX4-Autopilot calistirin ya da PX4_DIR verin." >&2
		return 1
	fi
	echo "$px4"
}
