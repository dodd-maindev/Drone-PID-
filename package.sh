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
if [ -f "$SRC_DIR/XboxBridge.exe" ]; then
    cp "$SRC_DIR/XboxBridge.exe" "$PKG_DIR/bin/"
    echo -e "${GREEN}  ✓ Xbox Bridge: bin/XboxBridge.exe${NC}"
fi

# Âm thanh động cơ drone (start.mp3, continue.mp3)
if [ -d "$SRC_DIR/simulation/audio" ]; then
    mkdir -p "$PKG_DIR/simulation/audio"
    cp "$SRC_DIR/simulation/audio"/*.mp3 "$PKG_DIR/simulation/audio/" 2>/dev/null || true
    echo -e "${GREEN}  ✓ Audio: simulation/audio/ (start.mp3, continue.mp3)${NC}"
fi

# === 4. Tạo launcher script ===
echo -e "\n${BOLD}[4/4] Tạo launcher...${NC}"

cat > "$PKG_DIR/run.sh" << 'LAUNCHER_EOF'
#!/bin/bash
# ==============================================================================
#  🚁 DRONE HITL SIMULATOR - ONE CLICK RUN
# ==============================================================================
#  Yêu cầu: Gazebo Harmonic đã cài trên máy (sudo apt install gz-harmonic)
#  Chạy: ./run.sh hoặc double-click file này
# ==============================================================================

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BRIDGE_BIN="$SCRIPT_DIR/bin/06_hitl_esp32_bridge"
WORLD_SDF="$SCRIPT_DIR/simulation/worlds/drone_world.sdf"

# Màu sắc
RED='\033[0;31m'; GREEN='\033[0;32m'; YELLOW='\033[1;33m'
CYAN='\033[0;36m'; BOLD='\033[1m'; NC='\033[0m'

GAZEBO_PID=""

cleanup() {
    echo ""
    echo -e "${YELLOW}[*] Đang tắt tất cả...${NC}"
    if [ -n "$GAZEBO_PID" ] && kill -0 "$GAZEBO_PID" 2>/dev/null; then
        kill "$GAZEBO_PID" 2>/dev/null
        wait "$GAZEBO_PID" 2>/dev/null || true
    fi
    pkill -f xbox_controller_bridge.exe 2>/dev/null || true
    powershell.exe -NoProfile -Command "Stop-Process -Name 'XboxBridge' -ErrorAction SilentlyContinue; Stop-Process -Name 'xbox_controller_bridge' -ErrorAction SilentlyContinue" 2>/dev/null || true
    echo -e "${GREEN}[✓] Đã tắt. Hẹn gặp lại!${NC}"
    exit 0
}
trap cleanup SIGINT SIGTERM

clear
echo -e "${CYAN}${BOLD}"
echo "╔══════════════════════════════════════════════════════════════╗"
echo "║            🚁 DRONE HITL SIMULATOR v1.0                    ║"
echo "║         Hardware-In-The-Loop Flight Controller              ║"
echo "╚══════════════════════════════════════════════════════════════╝"
echo -e "${NC}"

# Kiểm tra Gazebo
if ! command -v gz &>/dev/null; then
    echo -e "${RED}[✗] Gazebo Sim chưa cài! Chạy: sudo apt install gz-harmonic${NC}"
    read -p "Nhấn Enter để thoát..."
    exit 1
fi

# Kiểm tra binary
if [ ! -f "$BRIDGE_BIN" ]; then
    echo -e "${RED}[✗] Không tìm thấy binary: $BRIDGE_BIN${NC}"
    read -p "Nhấn Enter để thoát..."
    exit 1
fi

# 1. Quản lý kết nối phần cứng từ Windows (WSL2)
if [ -d "/usr/lib/wsl" ] || grep -qi microsoft /proc/version 2>/dev/null; then
    if command -v powershell.exe &>/dev/null; then
        # Nếu tay cầm Xbox vô tình bị attach vào WSL qua usbipd, trả lại cho Windows
        # (Tay cầm Xbox cần ở trên Windows để dùng driver XInput mượt mà, không lag)
        for xhwid in "045e:028e" "045e:028f" "045e:02d1" "045e:02dd" "045e:02e3" "045e:02ea" "045e:0b12" "045e:0719"; do
            powershell.exe -NoProfile -Command "usbipd detach --hardware-id $xhwid" &>/dev/null || true
        done

        # Tự động tìm & attach ESP32 từ Windows qua usbipd
        if [ ! -e "/dev/ttyUSB0" ] && [ ! -e "/dev/ttyACM0" ]; then
            echo -e "${YELLOW}[*] Đang tự động quét tìm cổng ESP32 trên Windows qua usbipd...${NC}"
            
            # Kiểm tra xem usbipd đã cài trên Windows chưa
            if powershell.exe -NoProfile -Command "Get-Command usbipd -ErrorAction SilentlyContinue" | grep -qi "usbipd"; then
                powershell.exe -NoProfile -Command "
                    \$list = usbipd list
                    \$esp = \$list | Where-Object { \$_ -match 'CP210|CH34|CH910|FTDI|UART|Serial|Espressif' } | Select-Object -First 1
                    if (\$esp) {
                        \$parts = (\$esp.Trim() -split '\s+')
                        \$busid = \$parts[0]
                        Write-Host '[✓] Tìm thấy ESP32 trên Windows (BUSID:' \$busid ')'
                        if (\$esp -match 'Not shared') {
                            Write-Host '[*] Thiết bị chưa được chia sẻ. Đang yêu cầu cấp quyền Admin để bind...'
                            Start-Process usbipd -ArgumentList \"bind --busid \$busid\" -Verb RunAs -Wait
                            Start-Sleep -Seconds 1
                        }
                        usbipd attach --wsl --busid \$busid --auto-attach 2>\$null
                        if (\$LASTEXITCODE -ne 0) {
                            usbipd attach --wsl --busid \$busid 2>\$null
                        }
                    } else {
                        \$hwids = @('10c4:ea60','10c4:ea70','10c4:ea80','1a86:7523','1a86:5523','1a86:7522','1a86:55d4','0403:6001','0403:6010','0403:6015','303a:1001','303a:0002','303a:1002','2341:0043')
                        foreach (\$h in \$hwids) {
                            usbipd attach --wsl --hardware-id \$h 2>\$null
                            if (\$LASTEXITCODE -eq 0) { break }
                        }
                    }
                "
                sleep 2
            else
                echo -e "${RED}[⚠] Windows chưa cài đặt 'usbipd-win'!${NC}"
                echo -e "${YELLOW}    Để ESP32 kết nối được vào WSL2, bạn hãy mở PowerShell trên Windows (Run as Administrator) và chạy:${NC}"
                echo -e "      ${CYAN}winget install --interactive --exact dorssel.usbipd-win${NC}"
            fi
        fi

        # Tự động kích hoạt Cầu nối Tay Cầm Xbox trên Windows (gửi trực tiếp tới WSL IP)
        XBOX_EXE="$SCRIPT_DIR/bin/XboxBridge.exe"
        WSL_IP=$(ip -4 addr show eth0 2>/dev/null | grep -oP '(?<=inet\s)\d+(\.\d+){3}' || hostname -I | awk '{print $1}')
        [ -z "$WSL_IP" ] && WSL_IP="127.0.0.1"
        if [ -f "$XBOX_EXE" ]; then
            echo -e "${YELLOW}[*] Khởi động Cầu nối Tay cầm Xbox trên Windows...${NC}"
            # Sao chép sang C:\DroneHITL\XboxBridge.exe để tránh lỗi PowerShell chặn chạy file từ mạng UNC (\\wsl.localhost)
            mkdir -p /mnt/c/DroneHITL 2>/dev/null || true
            cp -f "$XBOX_EXE" /mnt/c/DroneHITL/XboxBridge.exe 2>/dev/null || true

            powershell.exe -NoProfile -Command "Stop-Process -Name 'XboxBridge' -ErrorAction SilentlyContinue; Stop-Process -Name 'xbox_controller_bridge' -ErrorAction SilentlyContinue" 2>/dev/null || true

            WIN_RUN_STATUS=$(powershell.exe -NoProfile -Command "
                \$exe = 'C:\DroneHITL\XboxBridge.exe'
                if (-not (Test-Path \$exe)) {
                    \$tempDir = Join-Path \$env:TEMP 'DroneHITL'
                    if (-not (Test-Path \$tempDir)) { New-Item -ItemType Directory -Path \$tempDir -Force | Out-Null }
                    \$exe = Join-Path \$tempDir 'XboxBridge.exe'
                    \$src = wslpath -w '$XBOX_EXE' 2>\$null
                    if (\$src) { Copy-Item -Path \$src -Destination \$exe -Force -ErrorAction SilentlyContinue }
                }
                if (Test-Path \$exe) {
                    # Khởi chạy cửa sổ console bình thường để người dùng theo dõi log trực tiếp trên màn hình Windows
                    Start-Process -FilePath \$exe -ArgumentList '$WSL_IP 9099'
                    Start-Sleep -Milliseconds 400
                    if (Get-Process -Name 'XboxBridge' -ErrorAction SilentlyContinue) {
                        Write-Output 'STARTED'
                    } else {
                        Write-Output 'NOT_RUNNING'
                    }
                } else {
                    Write-Output 'FILE_NOT_FOUND'
                }
            " 2>/dev/null | tr -d '\r\n')

            if [[ "$WIN_RUN_STATUS" == *"STARTED"* ]]; then
                echo -e "${GREEN}[✓] Cửa sổ Tay cầm Xbox đã mở trên màn hình Windows (kết nối tới $WSL_IP:9099)!${NC}"
                echo -e "${CYAN}    • Cửa sổ console trên Windows hiển thị trạng thái nút bấm trực tiếp${NC}"
                echo -e "${CYAN}    • File log Windows: C:\\DroneHITL\\XboxBridge.log | WSL: ./hitl_bridge.log${NC}"
            else
                echo -e "${YELLOW}[⚠] Trạng thái XboxBridge Windows: $WIN_RUN_STATUS${NC}"
                echo -e "${YELLOW}    Nếu tay cầm không nhận, hãy click trực tiếp file: C:\\DroneHITL\\XboxBridge.exe${NC}"
            fi
        fi
    fi
fi

ESP_PORT=""
for i in {1..12}; do
    for p in /dev/ttyUSB0 /dev/ttyUSB1 /dev/ttyACM0 /dev/ttyACM1; do
        [ -e "$p" ] && ESP_PORT="$p" && break 2
    done
    sleep 0.25
done

if [ -n "$ESP_PORT" ]; then
    sudo chmod 666 "$ESP_PORT" 2>/dev/null || true
    echo -e "${GREEN}[✓] ESP32 đã kết nối thành công: $ESP_PORT${NC}"
else
    echo -e "${RED}[✗] Chưa tìm thấy cổng ESP32 trong WSL2!${NC}"
    echo -e "${YELLOW}════════════════════════════════════════════════════════════════${NC}"
    echo -e "${YELLOW}  HƯỚNG DẪN KẾT NỐI ESP32 (Chỉ cần làm 1 lần trên máy mới):${NC}"
    echo -e "  1. Cắm cáp USB ESP32 vào máy tính."
    echo -e "  2. Mở PowerShell trên Windows (Chuột phải -> Run as Administrator), gõ:"
    echo -e "       ${CYAN}usbipd list${NC}"
    echo -e "       ${CYAN}usbipd bind --busid <MÃ_BUSID_CỦA_ESP32>${NC}"
    echo -e "       ${CYAN}usbipd attach --wsl --busid <MÃ_BUSID_CỦA_ESP32>${NC}"
    echo -e "  3. Chạy lại: ${CYAN}./run.sh${NC}"
    echo -e "${YELLOW}════════════════════════════════════════════════════════════════${NC}"
    echo ""
    read -p "Nhấn Enter để thử lại với /dev/ttyUSB0 hoặc Ctrl+C để thoát..."
    ESP_PORT="/dev/ttyUSB0"
fi

# Khởi động Gazebo
echo -e "${YELLOW}[*] Khởi động Gazebo Sim...${NC}"
export GZ_SIM_RESOURCE_PATH="$SCRIPT_DIR/simulation/models:${HOME}/.gz/models"

# === TỰ ĐỘNG PHÁT HIỆN GPU ===
GPU_INFO=""
if [ -d "/usr/lib/wsl" ] || grep -qi microsoft /proc/version 2>/dev/null; then
    # WSL2: dùng Direct3D 12 passthrough
    sudo chmod 666 /dev/dri/* 2>/dev/null || true
    export GALLIUM_DRIVER=d3d12
    export LD_LIBRARY_PATH="/usr/lib/wsl/lib:$LD_LIBRARY_PATH"
    
    if nvidia-smi &>/dev/null; then
        export MESA_D3D12_DEFAULT_ADAPTER_NAME=NVIDIA
        NV_NAME=$(nvidia-smi --query-gpu=name --format=csv,noheader 2>/dev/null | head -1 | xargs)
        GPU_INFO="WSL2 D3D12 GPU Passthrough ($NV_NAME)"
    else
        export MESA_D3D12_DEFAULT_ADAPTER_NAME=""
        GPU_INFO="WSL2 D3D12 GPU Passthrough"
    fi
    echo -e "${GREEN}[✓] GPU: $GPU_INFO (Tăng tốc phần cứng D3D12)${NC}"
elif command -v nvidia-smi &>/dev/null && nvidia-smi &>/dev/null; then
    NV_NAME=$(nvidia-smi --query-gpu=name --format=csv,noheader 2>/dev/null | head -1 | xargs)
    GPU_INFO="NVIDIA Native ($NV_NAME)"
    echo -e "${GREEN}[✓] GPU: $GPU_INFO${NC}"
elif lspci 2>/dev/null | grep -qi "AMD.*VGA\|Radeon"; then
    GPU_INFO="AMD $(lspci 2>/dev/null | grep -i 'vga\|3d' | grep -i amd | head -1 | sed 's/.*: //')"
    echo -e "${GREEN}[✓] GPU: $GPU_INFO${NC}"
elif lspci 2>/dev/null | grep -qi "Intel.*VGA\|Intel.*Graphics"; then
    GPU_INFO="Intel $(lspci 2>/dev/null | grep -i 'vga\|3d' | grep -i intel | head -1 | sed 's/.*: //')"
    echo -e "${GREEN}[✓] GPU: $GPU_INFO${NC}"
else
    GPU_INFO="CPU Software Rendering"
    export LIBGL_ALWAYS_SOFTWARE=1
    echo -e "${YELLOW}[⚠] Không tìm thấy GPU — dùng CPU rendering (chậm)${NC}"
fi

gz sim -r "$WORLD_SDF" &
GAZEBO_PID=$!

# Chờ Gazebo sẵn sàng
for i in $(seq 1 30); do
    kill -0 "$GAZEBO_PID" 2>/dev/null || { echo -e "${RED}[✗] Gazebo crash!${NC}"; exit 1; }
    gz topic -l 2>/dev/null | grep -q "/x500/odometry" && break
    sleep 1
done
echo -e "${GREEN}[✓] Gazebo đã sẵn sàng!${NC}"

# Tự động kích hoạt Camera bám theo Drone (góc nhìn thứ 3 - Follow Mode)
(
    for i in $(seq 1 20); do
        sleep 1
        gz topic -t /gui/track -m gz.msgs.CameraTrack -p 'track_mode: 2, follow_target: {name: "x500", type: 2}, follow_offset: {x: -3.5, y: 0.0, z: 1.8}, follow_pgain: 0.08' &>/dev/null || true
    done
) &

# Hiển thị hướng dẫn
echo -e "${CYAN}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
echo -e "${BOLD}  🎮 ĐIỀU KHIỂN TAY CẦM XBOX 360:${NC}"
echo -e "    ${GREEN}✓ Tay cầm cắm vào Windows (tự động kết nối, KHÔNG CẦN usbipd)${NC}"
echo -e "    Y = Cất cánh lên 2.0m  │  A = Hạ cánh an toàn  │  BACK = Phanh khẩn"
echo -e "    Stick trái ↕ = Độ cao  │  Stick trái ↔ = Xoay Yaw (Hướng)"
echo -e "    Stick phải = Di chuyển (Tiến/Lùi/Trái/Phải)"
echo -e "    X / B = Xoay trái/phải  │  RB = Khóa Camera bám theo drone"
echo -e "    LB + Stick phải = LỘN NHÀO 360° (ACRO FLIP)"
echo -e "${CYAN}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
echo -e "${YELLOW}  Ctrl+C để tắt tất cả${NC}"
echo ""

# Chạy bridge
"$BRIDGE_BIN" "$ESP_PORT"
cleanup
LAUNCHER_EOF

chmod +x "$PKG_DIR/run.sh"
echo -e "${GREEN}  ✓ Launcher: run.sh${NC}"

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
  • ESP32 cắm qua USB (tự động phát hiện)
  • Tay cầm Xbox 360 (tùy chọn)

CÁCH CHẠY:
  1. Mở Terminal tại thư mục này
  2. Chạy: ./run.sh
  3. Xong! Gazebo + Bridge tự khởi động

ĐIỀU KHIỂN (Xbox 360):
  Y           = Cất cánh lên 2m
  A           = Hạ cánh an toàn
  Stick trái  = Điều khiển độ cao
  Stick phải  = Di chuyển (Tiến/Lùi/Trái/Phải)
  X           = Xoay quanh trục trái
  B           = Xoay quanh trục phải
  BACK        = Phanh dừng khẩn cấp

ĐIỀU KHIỂN (Bàn phím):
  Q = Cất cánh  |  A = Hạ cánh
  W/S = Bay lên/xuống
  Mũi tên = Di chuyển
  Z/C = Xoay Yaw
  SPACE = Phanh khẩn  |  X = Thoát

Ctrl+C = Tắt tất cả
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
