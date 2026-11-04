#!/bin/bash
# ==============================================================================
#  📦 ĐÓNG GÓI DRONE HITL SIMULATOR THÀNH PACKAGE HOÀN CHỈNH
# ==============================================================================
#  Tạo folder "DroneHITL/" chứa mọi thứ cần thiết:
#    - Binary đã biên dịch (không cần source code)
#    - Models 3D (nhà, cây, xe, drone...)
#    - World SDF
#    - Xbox Bridge (Windows)
#    - Launcher script (1 click chạy)
# ==============================================================================

set -e

SRC_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BUILD_DIR="$SRC_DIR/build"
PKG_NAME="DroneHITL"
PKG_DIR="$SRC_DIR/$PKG_NAME"

RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
CYAN='\033[0;36m'
BOLD='\033[1m'
NC='\033[0m'

echo -e "${CYAN}${BOLD}"
echo "╔══════════════════════════════════════════════════════════════╗"
echo "║        📦 ĐÓNG GÓI DRONE HITL SIMULATOR                   ║"
echo "╚══════════════════════════════════════════════════════════════╝"
echo -e "${NC}"

# === 1. Biên dịch binary ===
echo -e "${BOLD}[1/4] Biên dịch binary...${NC}"
if [ ! -d "$BUILD_DIR" ]; then
    cmake -B "$BUILD_DIR" -S "$SRC_DIR"
fi
cmake --build "$BUILD_DIR" --target 06_hitl_esp32_bridge -j$(nproc)
echo -e "${GREEN}  ✓ Biên dịch thành công${NC}"

# === 2. Tạo cấu trúc package ===
echo -e "\n${BOLD}[2/4] Tạo cấu trúc package...${NC}"
rm -rf "$PKG_DIR"
mkdir -p "$PKG_DIR/bin"
mkdir -p "$PKG_DIR/simulation/worlds"
mkdir -p "$PKG_DIR/simulation/models"

# === 3. Copy file cần thiết ===
echo -e "\n${BOLD}[3/4] Copy file...${NC}"

# Binary đã compile
cp "$BUILD_DIR/06_hitl_esp32_bridge" "$PKG_DIR/bin/"
echo -e "${GREEN}  ✓ Binary: bin/06_hitl_esp32_bridge${NC}"

# World SDF
cp "$SRC_DIR/simulation/worlds/drone_world.sdf" "$PKG_DIR/simulation/worlds/"
echo -e "${GREEN}  ✓ World: simulation/worlds/drone_world.sdf${NC}"

# Models 3D (chỉ copy models thực sự dùng)
for model_dir in "$SRC_DIR/simulation/models"/*/; do
    model_name=$(basename "$model_dir")
    cp -r "$model_dir" "$PKG_DIR/simulation/models/"
    echo -e "${GREEN}  ✓ Model: $model_name${NC}"
done

# Xbox Bridge cho Windows
if [ -f "$SRC_DIR/tools/xbox_bridge/XboxBridge.exe" ]; then
    cp "$SRC_DIR/tools/xbox_bridge/XboxBridge.exe" "$PKG_DIR/bin/"
    echo -e "${GREEN}  ✓ Xbox Bridge: bin/XboxBridge.exe${NC}"
elif [ -f "$SRC_DIR/XboxBridge.exe" ]; then
    cp "$SRC_DIR/XboxBridge.exe" "$PKG_DIR/bin/"
    echo -e "${GREEN}  ✓ Xbox Bridge: bin/XboxBridge.exe${NC}"
fi

# Âm thanh động cơ drone (start.mp3, continue.mp3)
if [ -d "$SRC_DIR/simulation/audio" ]; then
    mkdir -p "$PKG_DIR/simulation/audio"
    cp "$SRC_DIR/simulation/audio"/*.mp3 "$PKG_DIR/simulation/audio/" 2>/dev/null || true
    echo -e "${GREEN}  ✓ Audio: simulation/audio/ (start.mp3, continue.mp3)${NC}"
fi

# Web Oscilloscope & Telemetry Lab
if [ -d "$SRC_DIR/tools/web_oscilloscope" ]; then
    mkdir -p "$PKG_DIR/tools"
    cp -r "$SRC_DIR/tools/web_oscilloscope" "$PKG_DIR/tools/"
    echo -e "${GREEN}  ✓ Web Oscilloscope: tools/web_oscilloscope/${NC}"
fi

# Firmware ESP32 (Đầy đủ mã nguồn và thư viện nạp ESP32)
if [ -d "$SRC_DIR/firmware_esp32" ]; then
    mkdir -p "$PKG_DIR/firmware_esp32"
    cp -r "$SRC_DIR/firmware_esp32"/* "$PKG_DIR/firmware_esp32/" 2>/dev/null || true
    rm -rf "$PKG_DIR/firmware_esp32/.pio" 2>/dev/null || true
    echo -e "${GREEN}  ✓ Firmware ESP32: firmware_esp32/${NC}"
fi

# Chuyển đổi và tài liệu hướng dẫn
cp "$SRC_DIR"/chuyen_sang_*.sh "$PKG_DIR/" 2>/dev/null || true
cp "$SRC_DIR"/CHUYEN_SANG_*.bat "$PKG_DIR/" 2>/dev/null || true
cp "$SRC_DIR"/HUONG_DAN_*.md "$PKG_DIR/" 2>/dev/null || true
echo -e "${GREEN}  ✓ Scripts chuyển đổi chế độ & Tài liệu hướng dẫn${NC}"

# === 4. Cài đặt launcher script & Hướng dẫn ===
echo -e "\n${BOLD}[4/4] Cài đặt launcher script...${NC}"

if [ -f "$SRC_DIR/scripts/run_hitl_launcher.sh" ]; then
    cp -f "$SRC_DIR/scripts/run_hitl_launcher.sh" "$PKG_DIR/run.sh"
elif [ -f "$SRC_DIR/DroneHITL/run.sh" ]; then
    cp -f "$SRC_DIR/DroneHITL/run.sh" "$PKG_DIR/run.sh"
fi

chmod +x "$PKG_DIR/run.sh"
echo -e "${GREEN}  ✓ Launcher: run.sh (Tự động mở QGroundControl, Web Oscilloscope, D3D12 GPU Passthrough)${NC}"

# Tạo README
cat > "$PKG_DIR/README.txt" << 'README_EOF'
╔══════════════════════════════════════════════════════════════╗
║            🚁 DRONE HITL SIMULATOR v1.0                    ║
║         Hardware-In-The-Loop Flight Controller              ║
╚══════════════════════════════════════════════════════════════╝

YÊU CẦU HỆ THỐNG:
  • Ubuntu 22.04+ hoặc WSL2
  • Gazebo Harmonic (gz-sim 8)
    Cài đặt: sudo apt install gz-harmonic
  • ESP32 nạp Firmware cắm qua USB (tự động phát hiện cổng)
  • Tay cầm Xbox 360 / One (tùy chọn)
  • QGroundControl trên Windows (tự động mở khi chạy)

CÁCH CHẠY:
  1. Mở Terminal tại thư mục này
  2. Chạy: ./run.sh
  3. Xong! Gazebo + Web Oscilloscope + QGroundControl tự khởi động

ĐIỀU KHIỂN (Tay cầm Xbox 360):
  1. Bấm Y                  : ARM động cơ (quay nhẹ tại chỗ 200 rad/s)
  2. Đẩy Stick trái ↑      : Tăng ga cất cánh mượt mà / Bay lên
  3. Kéo Stick trái ↓      : Hạ cánh an toàn (chạm đất tự về Idle)
  4. Giữ Stick trái ↓ 1.5s  : DISARM tắt hẳn động cơ (hoặc bấm Y)
  • Stick phải              : Di chuyển (Tiến/Lùi/Trái/Phải)
  • Stick trái ↔ hoặc X / B : Xoay góc hướng (Yaw)
  • Phím A                  : Hạ cánh tự động (Auto-Land)
  • Phím BACK               : Phanh dừng khẩn cấp / DISARM
  • LB + Stick phải         : Lộn nhào 360° (Acro Flip)

ĐIỀU KHIỂN (Bàn phím):
  Y / Q = ARM / DISARM động cơ (quay nhẹ tại chỗ)
  W / S = Tăng ga cất cánh / Hạ cánh
  Mũi tên = Di chuyển (Tiến/Lùi/Trái/Phải)
  A / D = Xoay Yaw góc hướng
  SPACE = Phanh khẩn / DISARM  |  X = Thoát

Ctrl+C = Tắt tất cả (Tự động tắt QGroundControl và dọn sạch tiến trình)
README_EOF

echo -e "${GREEN}  ✓ README.txt${NC}"

# === Tổng kết ===
PKG_SIZE=$(du -sh "$PKG_DIR" | cut -f1)
FILE_COUNT=$(find "$PKG_DIR" -type f | wc -l)

echo -e "\n${CYAN}${BOLD}══════════════════════════════════════════════════════════════${NC}"
echo -e "${GREEN}${BOLD}  ✅ ĐÓNG GÓI HOÀN TẤT!${NC}"
echo -e "${CYAN}${BOLD}══════════════════════════════════════════════════════════════${NC}"
echo -e "  📁 Thư mục: ${BOLD}$PKG_DIR/${NC}"
echo -e "  📊 Kích thước: ${BOLD}$PKG_SIZE${NC} ($FILE_COUNT files)"
echo -e ""
echo -e "  ${BOLD}Cách sử dụng:${NC}"
echo -e "    1. Copy folder ${BOLD}$PKG_NAME/${NC} sang máy đích"
echo -e "    2. Chạy: ${BOLD}./$PKG_NAME/run.sh${NC}"
echo -e ""
echo -e "  ${BOLD}Nén thành file zip:${NC}"
echo -e "    tar -czf ${PKG_NAME}.tar.gz $PKG_NAME/"
echo -e "${CYAN}══════════════════════════════════════════════════════════════${NC}"
