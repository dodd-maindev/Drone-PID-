#!/bin/bash
# ==============================================================================
# Script khởi chạy Gazebo Sim (WSL2 GPU Direct3D 12)
# ==============================================================================

# Đường dẫn tài nguyên mô hình 3D
export GZ_SIM_RESOURCE_PATH=/home/do/drone_control_cpp/simulation/models:~/.gz/models

# Kích hoạt GPU Nvidia thông qua Direct3D 12 trên WSL2
export MESA_D3D12_DEFAULT_ADAPTER_NAME=GeForce
export GALLIUM_DRIVER=d3d12

echo "🚀 Đang khởi động Gazebo Sim (Thế giới 3D Drone)..."
gz sim -r /home/do/drone_control_cpp/simulation/worlds/drone_world.sdf "$@"
