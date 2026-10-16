# Source me inside the distrobox container to render Gazebo / RViz on the host's NVIDIA GPU:
#     source src/tunibot_delivery/scripts/nvidia_gl.sh
# The Ubuntu 22.04 container has no NVIDIA OpenGL libraries, so Mesa tries "nouveau", fails
# ("libGL error: failed to load driver: nouveau") and falls back to the CPU renderer (swrast).
# The host's driver libraries are visible under /run/host: link them into a user folder and
# point libglvnd at them. Nothing is installed; does nothing without an NVIDIA host.
_nv_host=/run/host/usr/lib/x86_64-linux-gnu
_nv_dir=$HOME/.local/lib/nvidia-host-gl
if [ -e "$_nv_host/libGLX_nvidia.so.0" ] && [ -e /dev/nvidia0 ]; then
  mkdir -p "$_nv_dir"
  find "$_nv_dir" -xtype l -delete                 # links left by an older driver version
  for _f in $(ls "$_nv_host" | grep -E '^(libGLX_nvidia|libEGL_nvidia|libnvidia-[a-z0-9-]+)\.so'); do
    ln -sfn "$_nv_host/$_f" "$_nv_dir/$_f"
  done
  case ":$LD_LIBRARY_PATH:" in *":$_nv_dir:"*) ;; *) export LD_LIBRARY_PATH=$_nv_dir${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH} ;; esac
  export __GLX_VENDOR_LIBRARY_NAME=nvidia
fi
unset _nv_host _nv_dir _f
