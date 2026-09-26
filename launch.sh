#!/bin/bash
# ==============================================================================
#  🚁 DRONE HITL SIMULATOR - ONE CLICK LAUNCHER
# ==============================================================================
#  Chạy file này duy nhất để khởi động toàn bộ hệ thống:
#    1. Tự động biên dịch (nếu cần)
#    2. Khởi động Gazebo Sim (thế giới 3D)
#    3. Khởi động HITL Bridge (cầu nối ESP32 <-> Gazebo)
#    4. Tự động phát hiện ESP32 + Tay cầm Xbox 360
#    5. Ctrl+C để tắt tất cả
# ==============================================================================

set -e

# === CẤU HÌNH ===
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BUILD_DIR="$SCRIPT_DIR/build"
BRIDGE_BIN="$BUILD_DIR/06_hitl_esp32_bridge"
WORLD_SDF="$SCRIPT_DIR/simulation/worlds/drone_world.sdf"

# Màu sắc terminal
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
CYAN='\033[0;36m'
BOLD='\033[1m'
NC='\033[0m' # No Color

# PID tracking để cleanup
GAZEBO_PID=""
BRIDGE_PID=""

cleanup() {
    echo ""
    echo -e "${YELLOW}[*] Đang tắt tất cả...${NC}"
    
    # Tắt bridge trước
    if [ -n "$BRIDGE_PID" ] && kill -0 "$BRIDGE_PID" 2>/dev/null; then
        kill "$BRIDGE_PID" 2>/dev/null
        wait "$BRIDGE_PID" 2>/dev/null || true
        echo -e "${GREEN}[✓] Đã tắt HITL Bridge${NC}"
    fi
    
    # Tắt Gazebo
    if [ -n "$GAZEBO_PID" ] && kill -0 "$GAZEBO_PID" 2>/dev/null; then
        kill "$GAZEBO_PID" 2>/dev/null
        wait "$GAZEBO_PID" 2>/dev/null || true
        echo -e "${GREEN}[✓] Đã tắt Gazebo Sim${NC}"
    fi
    
    # Tắt Xbox bridge trên Windows (nếu có)
    pkill -f xbox_controller_bridge.exe 2>/dev/null || true
    
    echo -e "${GREEN}[✓] Tất cả đã tắt. Hẹn gặp lại!${NC}"
    exit 0
}

trap cleanup SIGINT SIGTERM

# === BANNER ===
clear
echo -e "${CYAN}${BOLD}"
echo "╔══════════════════════════════════════════════════════════════╗"
echo "║            🚁 DRONE HITL SIMULATOR v1.0                    ║"
echo "║         Hardware-In-The-Loop Flight Controller              ║"
echo "║                                                            ║"
echo "║  ESP32 Firmware ←→ Gazebo 3D Simulation ←→ Xbox 360        ║"
echo "╚══════════════════════════════════════════════════════════════╝"
echo -e "${NC}"

# === BƯỚC 1: KIỂM TRA MÔI TRƯỜNG ===
echo -e "${BOLD}[1/5] Kiểm tra môi trường...${NC}"

# Kiểm tra Gazebo
if ! command -v gz &>/dev/null; then
    echo -e "${RED}[✗] Gazebo Sim chưa được cài đặt!${NC}"
    echo "    Cài đặt: sudo apt install gz-harmonic"
    exit 1
fi
echo -e "${GREEN}  ✓ Gazebo Sim: $(gz sim --version 2>/dev/null | head -1 || echo 'OK')${NC}"

# Kiểm tra CMake
if ! command -v cmake &>/dev/null; then
    echo -e "${RED}[✗] CMake chưa được cài đặt!${NC}"
    echo "    Cài đặt: sudo apt install cmake"
    exit 1
fi
echo -e "${GREEN}  ✓ CMake: $(cmake --version | head -1)${NC}"

# Kiểm tra World SDF
if [ ! -f "$WORLD_SDF" ]; then
    echo -e "${RED}[✗] Không tìm thấy world SDF: $WORLD_SDF${NC}"
    exit 1
fi
echo -e "${GREEN}  ✓ World SDF: drone_world.sdf${NC}"

# === BƯỚC 2: BIÊN DỊCH NẾU CẦN ===
echo -e "\n${BOLD}[2/5] Kiểm tra & biên dịch...${NC}"

if [ ! -d "$BUILD_DIR" ]; then
    echo -e "${YELLOW}  → Tạo thư mục build...${NC}"
    cmake -B "$BUILD_DIR" -S "$SCRIPT_DIR"
fi

if [ ! -f "$BRIDGE_BIN" ] || [ "$SCRIPT_DIR/experiments/06_hitl_esp32_bridge.cpp" -nt "$BRIDGE_BIN" ] || [ "$SCRIPT_DIR/CMakeLists.txt" -nt "$BRIDGE_BIN" ]; then
    echo -e "${YELLOW}  → Đang biên dịch HITL Bridge...${NC}"
    cmake --build "$BUILD_DIR" --target 06_hitl_esp32_bridge -j$(nproc)
    echo -e "${GREEN}  ✓ Biên dịch thành công!${NC}"
else
    echo -e "${GREEN}  ✓ Binary đã cập nhật, không cần biên dịch lại.${NC}"
fi

# === BƯỚC 3: PHÁT HIỆN & TỰ ĐỘNG KẾT NỐI ESP32 ===
echo -e "\n${BOLD}[3/5] Phát hiện & kết nối thiết bị ESP32...${NC}"

ESP_PORT=""

# Nếu đang ở WSL2 và chưa thấy cổng Serial, tự động gọi usbipd trên Windows để attach
if [ ! -e "/dev/ttyUSB0" ] && [ ! -e "/dev/ttyACM0" ] && ([ -d "/usr/lib/wsl" ] || grep -qi microsoft /proc/version 2>/dev/null); then
    if command -v powershell.exe &>/dev/null; then
        echo -e "${YELLOW}  → Đang tự động tìm & attach ESP32 từ Windows qua usbipd...${NC}"
        for hwid in "10c4:ea60" "1a86:7523" "1a86:55d4" "0403:6001" "303a:1001"; do
            powershell.exe -NoProfile -Command "usbipd attach --wsl --hardware-id $hwid" &>/dev/null && break || true
        done
        sleep 1
    fi
fi

for candidate in /dev/ttyUSB0 /dev/ttyUSB1 /dev/ttyACM0 /dev/ttyACM1; do
    if [ -e "$candidate" ]; then
        ESP_PORT="$candidate"
        break
    fi
done

if [ -n "$ESP_PORT" ]; then
    sudo chmod 666 "$ESP_PORT" 2>/dev/null || true
    echo -e "${GREEN}  ✓ ESP32 đã kết nối: ${BOLD}$ESP_PORT${NC}"
else
    echo -e "${YELLOW}  ⚠ Chưa tìm thấy ESP32. Bridge sẽ chờ kết nối...${NC}"
    ESP_PORT="/dev/ttyUSB0"
fi

# === BƯỚC 4: KHỞI ĐỘNG GAZEBO SIM ===
echo -e "\n${BOLD}[4/5] Khởi động Gazebo Sim...${NC}"

export GZ_SIM_RESOURCE_PATH="$SCRIPT_DIR/simulation/models:${HOME}/.gz/models"

# Tự động phát hiện GPU
if [ -d "/usr/lib/wsl" ] || grep -qi microsoft /proc/version 2>/dev/null; then
    # WSL2: Windows Subsystem for Linux (dùng D3D12 passthrough)
    sudo chmod 666 /dev/dri/* 2>/dev/null || true
    export GALLIUM_DRIVER=d3d12
    export LD_LIBRARY_PATH="/usr/lib/wsl/lib:$LD_LIBRARY_PATH"
    
    if nvidia-smi &>/dev/null; then
        export MESA_D3D12_DEFAULT_ADAPTER_NAME=NVIDIA
        NV_NAME=$(nvidia-smi --query-gpu=name --format=csv,noheader 2>/dev/null | head -1 | xargs)
        echo -e "${GREEN}  ✓ GPU: WSL2 D3D12 GPU Passthrough ($NV_NAME)${NC}"
    else
        export MESA_D3D12_DEFAULT_ADAPTER_NAME=""
        echo -e "${GREEN}  ✓ GPU: WSL2 D3D12 GPU Passthrough${NC}"
    fi
elif command -v nvidia-smi &>/dev/null && nvidia-smi &>/dev/null; then
    NV_NAME=$(nvidia-smi --query-gpu=name --format=csv,noheader 2>/dev/null | head -1 | xargs)
    echo -e "${GREEN}  ✓ GPU: NVIDIA Native ($NV_NAME)${NC}"
elif lspci 2>/dev/null | grep -qi "AMD\|Radeon"; then
    echo -e "${GREEN}  ✓ GPU: AMD${NC}"
elif lspci 2>/dev/null | grep -qi "Intel.*Graphics"; then
    echo -e "${GREEN}  ✓ GPU: Intel${NC}"
else
    export LIBGL_ALWAYS_SOFTWARE=1
    echo -e "${YELLOW}  ⚠ Không tìm thấy GPU — CPU rendering${NC}"
fi

gz sim -r "$WORLD_SDF" &
GAZEBO_PID=$!
echo -e "${GREEN}  ✓ Gazebo Sim đang khởi động (PID: $GAZEBO_PID)${NC}"

# Chờ Gazebo sẵn sàng (kiểm tra topic odometry)
echo -e "${YELLOW}  → Đang chờ Gazebo sẵn sàng...${NC}"
MAX_WAIT=30
for i in $(seq 1 $MAX_WAIT); do
    if ! kill -0 "$GAZEBO_PID" 2>/dev/null; then
        echo -e "${RED}[✗] Gazebo đã crash! Kiểm tra cấu hình GPU.${NC}"
        exit 1
    fi
    # Kiểm tra topic odometry có xuất hiện chưa
    if gz topic -l 2>/dev/null | grep -q "/x500/odometry" 2>/dev/null; then
        echo -e "${GREEN}  ✓ Gazebo Sim đã sẵn sàng!${NC}"
        break
    fi
    if [ "$i" -eq "$MAX_WAIT" ]; then
        echo -e "${YELLOW}  ⚠ Timeout chờ Gazebo, khởi động bridge anyway...${NC}"
    fi
    sleep 1
done

# Tự động kích hoạt Camera bám theo Drone (góc nhìn thứ 3 - Follow Mode)
(
    for i in $(seq 1 20); do
        sleep 1
        gz topic -t /gui/track -m gz.msgs.CameraTrack -p 'track_mode: 2, follow_target: {name: "x500", type: 2}, follow_offset: {x: -3.5, y: 0.0, z: 1.8}, follow_pgain: 0.08' &>/dev/null || true
    done
) &

# === BƯỚC 5: KHỞI ĐỘNG HITL BRIDGE ===
echo -e "\n${BOLD}[5/5] Khởi động HITL Bridge...${NC}"
echo -e "${CYAN}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
echo -e "${BOLD}  🎮 ĐIỀU KHIỂN XBOX 360:${NC}"
echo -e "    Y = Cất cánh  │  A = Hạ cánh  │  BACK = Phanh khẩn"
echo -e "    Stick trái ↕ = Độ cao  │  Stick trái ↔ = Xoay Yaw (Hướng)"
echo -e "    Stick phải = Di chuyển (Tiến/Lùi/Trái/Phải)"
echo -e "    X / B = Xoay trái/phải  │  RB = Khóa Camera bám theo drone"
echo -e "    LB + Stick phải (Trái/Phải/Lên/Xuống) = LỘN NHÀO 360° (ACRO FLIP)"
echo -e "${CYAN}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
echo -e "${YELLOW}  Nhấn Ctrl+C để tắt tất cả${NC}"
echo ""

# Chạy bridge ở foreground (để user thấy output)
"$BRIDGE_BIN" "$ESP_PORT"
BRIDGE_EXIT=$?

# Bridge kết thúc → cleanup
echo -e "\n${YELLOW}[*] HITL Bridge đã thoát (code: $BRIDGE_EXIT)${NC}"
cleanup
