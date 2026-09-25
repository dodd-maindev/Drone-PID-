#!/bin/bash
# ==============================================================================
# Script khởi chạy HITL Gateway với Tay cầm Xbox 360 & Gazebo Sim
# ==============================================================================

PORT=${1:-/dev/ttyUSB0}

echo "============================================================"
echo " 🎮 KHỞI CHẠY HITL GATEWAY - ĐIỀU KHIỂN TAY CẦM XBOX 360"
echo "============================================================"
echo " Cổng Serial ESP32: $PORT"
echo ""

# Kiểm tra quyền truy cập cổng Serial
if [ -e "$PORT" ]; then
    sudo chmod 666 "$PORT" 2>/dev/null || true
else
    echo "⚠️ Lưu ý: Chưa thấy cổng $PORT. Nếu đang dùng WSL2 hãy đảm bảo đã attach ESP32 (usbipd wsl attach)."
fi

# Biên dịch chương trình C++ nếu chưa có
cd /home/do/drone_control_cpp
if [ ! -f "build/06_hitl_esp32_bridge" ]; then
    echo "[*] Đang biên dịch 06_hitl_esp32_bridge..."
    cmake --build build --target 06_hitl_esp32_bridge -j4
fi

# Chạy cầu nối
./build/06_hitl_esp32_bridge "$PORT"
