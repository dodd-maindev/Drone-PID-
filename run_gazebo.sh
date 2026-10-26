#!/bin/bash
# ==============================================================================
# Script khởi chạy Gazebo Sim (Tự động phát hiện GPU / WSL2)
# ==============================================================================

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
export GZ_SIM_RESOURCE_PATH="$SCRIPT_DIR/simulation/models:${HOME}/.gz/models"

# Kiểm tra D3D12 GPU Passthrough trên WSL2
D3D12_OK=false
if grep -qi microsoft /proc/version 2>/dev/null && ([ -d "/usr/lib/wsl/lib" ] || [ -d "/dev/dri" ]); then
    sudo chmod 666 /dev/dri/* 2>/dev/null || true
    if command -v glxinfo &>/dev/null && GALLIUM_DRIVER=d3d12 LD_LIBRARY_PATH="/usr/lib/wsl/lib:$LD_LIBRARY_PATH" glxinfo -B &>/dev/null; then
        export GALLIUM_DRIVER=d3d12
        export LD_LIBRARY_PATH="/usr/lib/wsl/lib:$LD_LIBRARY_PATH"
        export MESA_D3D12_DEFAULT_ADAPTER_NAME=GeForce
        D3D12_OK=true
    fi
fi

if [ "$D3D12_OK" = false ]; then
    unset GALLIUM_DRIVER
    unset MESA_D3D12_DEFAULT_ADAPTER_NAME
    export LIBGL_ALWAYS_SOFTWARE=1
fi

echo "🚀 Đang khởi động Gazebo Sim (Thế giới 3D Drone)..."
gz sim -r "$SCRIPT_DIR/simulation/worlds/drone_world.sdf" "$@"
