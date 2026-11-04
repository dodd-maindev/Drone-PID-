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
GUI_PID=""
OSC_PID=""

cleanup() {
    echo ""
    echo -e "${YELLOW}[*] Đang tắt tất cả...${NC}"
    if [ -n "$GUI_PID" ] && kill -0 "$GUI_PID" 2>/dev/null; then
        kill "$GUI_PID" 2>/dev/null
    fi
    if [ -n "$GAZEBO_PID" ] && kill -0 "$GAZEBO_PID" 2>/dev/null; then
        kill "$GAZEBO_PID" 2>/dev/null
        wait "$GAZEBO_PID" 2>/dev/null || true
    fi
    if [ -n "$OSC_PID" ] && kill -0 "$OSC_PID" 2>/dev/null; then
        kill "$OSC_PID" 2>/dev/null
    fi
    pkill -f "web_oscilloscope/server.py" 2>/dev/null || true
    pkill -f xbox_controller_bridge.exe 2>/dev/null || true
    pkill -9 -f 06_hitl_esp32_bridge 2>/dev/null || true
    # Giải phóng cưỡng bức mọi tiến trình đang giữ cổng Serial để kernel trả về ttyUSB0
    fuser -k /dev/ttyUSB* /dev/ttyACM* 2>/dev/null || true
    powershell.exe -NoProfile -Command "Stop-Process -Name 'XboxBridge' -ErrorAction SilentlyContinue; Stop-Process -Name 'xbox_controller_bridge' -ErrorAction SilentlyContinue" 2>/dev/null || true
    
    # Tự động đóng QGroundControl khi tắt mô phỏng (chạy ngầm không chặn thoát)
    echo -e "${YELLOW}[*] Đang tắt QGroundControl...${NC}"
    ( powershell.exe -NoProfile -Command "Stop-Process -Name 'QGroundControl' -ErrorAction SilentlyContinue" </dev/null >/dev/null 2>&1 & )
    pkill -f -i qgroundcontrol 2>/dev/null || true

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

# Dọn dẹp triệt để các tiến trình cũ còn sót lại để cổng serial luôn trở về /dev/ttyUSB0
pkill -9 -f 06_hitl_esp32_bridge 2>/dev/null || true
fuser -k /dev/ttyUSB* /dev/ttyACM* 2>/dev/null || true

# 1. Quản lý kết nối phần cứng từ Windows (WSL2)
if [ -d "/usr/lib/wsl" ] || grep -qi microsoft /proc/version 2>/dev/null; then
    if command -v powershell.exe &>/dev/null; then
        # Nếu tay cầm Xbox vô tình bị attach vào WSL qua usbipd, trả lại cho Windows
        # (Tay cầm Xbox cần ở trên Windows để dùng driver XInput mượt mà, không lag)
        for xhwid in "045e:028e" "045e:028f" "045e:02d1" "045e:02dd" "045e:02e3" "045e:02ea" "045e:0b12" "045e:0719"; do
            powershell.exe -NoProfile -Command "usbipd detach --hardware-id $xhwid" &>/dev/null || true
        done

        # Tự động tìm & attach ESP32 từ Windows qua usbipd nếu chưa có cổng serial nào đang kết nối
        if [ -z "$(ls /sys/class/tty/ttyUSB* /sys/class/tty/ttyACM* /dev/ttyUSB* /dev/ttyACM* 2>/dev/null)" ]; then
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
                        usbipd attach --wsl --busid \$busid 2>\$null
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
# 1. Nếu người dùng truyền cổng trực tiếp qua tham số (ví dụ: ./run.sh /dev/ttyUSB1)
for arg in "$@"; do
    if [[ "$arg" == /dev/tty* ]] && [ -e "$arg" ]; then
        ESP_PORT="$arg"
        break
    fi
done

# 2. Tự động dò tìm cổng ESP32 khả dụng thực tế:
# Kiểm tra mở cổng thực tế (os.open) để loại bỏ hoàn toàn các file node "ma" (stale node)
find_live_esp_port() {
    python3 -c "
import glob, os
# Ưu tiên các cổng mới cắm (số lớn hơn như ttyUSB1, ttyUSB2 trước)
ports = sorted(glob.glob('/dev/ttyUSB*') + glob.glob('/dev/ttyACM*'), reverse=True)
for p in ports:
    try:
        os.system(f'sudo chmod 666 {p} 2>/dev/null')
        fd = os.open(p, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
        os.close(fd)
        print(p)
        break
    except Exception:
        pass
" 2>/dev/null
}

if [ -z "$ESP_PORT" ]; then
    for i in {1..12}; do
        ESP_PORT=$(find_live_esp_port)
        [ -n "$ESP_PORT" ] && break
        sleep 0.25
    done
fi

if [ -n "$ESP_PORT" ]; then
    sudo chmod 666 "$ESP_PORT" 2>/dev/null || true
    if [ "$ESP_PORT" != "/dev/ttyUSB0" ] && [ ! -e "/dev/ttyUSB0" ]; then
        sudo ln -sf "$ESP_PORT" /dev/ttyUSB0 2>/dev/null || true
    fi
    echo -e "${GREEN}[✓] ESP32 đã kết nối và thông suốt: $ESP_PORT${NC}"
else
    echo -e "${YELLOW}[*] Đang thử kết nối lại qua tunnel USBIP...${NC}"
    "/mnt/c/Program Files/usbipd-win/usbipd.exe" detach --hardware-id 10c4:ea60 2>/dev/null || true
    "/mnt/c/Program Files/usbipd-win/usbipd.exe" detach --hardware-id 1a86:7523 2>/dev/null || true
    sleep 0.8
    "/mnt/c/Program Files/usbipd-win/usbipd.exe" attach --wsl --hardware-id 10c4:ea60 2>/dev/null || true
    "/mnt/c/Program Files/usbipd-win/usbipd.exe" attach --wsl --hardware-id 1a86:7523 2>/dev/null || true
    sleep 1.2
    ESP_PORT=$(find_live_esp_port)
    if [ -n "$ESP_PORT" ]; then
        echo -e "${GREEN}[✓] ESP32 đã kết nối và thông suốt: $ESP_PORT${NC}"
    else
        echo -e "${RED}[✗] Chưa tìm thấy cổng ESP32 trong WSL2!${NC}"
        echo -e "${YELLOW}════════════════════════════════════════════════════════════════${NC}"
        echo -e "${YELLOW}  HƯỚNG DẪN KẾT NỐI ESP32:${NC}"
        echo -e "  1. Rút cáp USB ESP32 ra cắm lại vào máy tính."
        echo -e "  2. Chạy: ${CYAN}CHUYEN_SANG_HITL_WSL2.bat${NC} (trên Windows) hoặc cắm lại cáp."
        echo -e "  3. Chạy lại: ${CYAN}./run.sh${NC}"
        echo -e "${YELLOW}════════════════════════════════════════════════════════════════${NC}"
        echo ""
        read -p "Nhấn Enter để thử lại hoặc Ctrl+C để thoát..."
        ESP_PORT=$(find_live_esp_port)
        [ -z "$ESP_PORT" ] && ESP_PORT="/dev/ttyUSB0"
    fi
fi

# Khởi động Gazebo
echo -e "${YELLOW}[*] Khởi động Gazebo Sim...${NC}"
export GZ_SIM_RESOURCE_PATH="$SCRIPT_DIR/simulation/models:${HOME}/.gz/models"

# === TỰ ĐỘNG PHÁT HIỆN GPU ===
GPU_INFO=""
D3D12_OK=false

# Kiểm tra GPU Passthrough D3D12 trên WSL2 (chỉ kích hoạt nếu phần cứng và thư viện thực sự hoạt động)
if grep -qi microsoft /proc/version 2>/dev/null && ([ -d "/usr/lib/wsl/lib" ] || [ -d "/dev/dri" ]); then
    sudo chmod 666 /dev/dri/* 2>/dev/null || true
    # Kiểm tra xem D3D12 có tạo được OpenGL context thực tế không (tránh crash Qt)
    if command -v glxinfo &>/dev/null && GALLIUM_DRIVER=d3d12 LD_LIBRARY_PATH="/usr/lib/wsl/lib:$LD_LIBRARY_PATH" glxinfo -B &>/dev/null; then
        export GALLIUM_DRIVER=d3d12
        export LD_LIBRARY_PATH="/usr/lib/wsl/lib:$LD_LIBRARY_PATH"
        export QT_QPA_PLATFORM=xcb
        export LIBGL_DRI3_DISABLE=1
        D3D12_OK=true
        if command -v nvidia-smi &>/dev/null && nvidia-smi &>/dev/null; then
            export MESA_D3D12_DEFAULT_ADAPTER_NAME=NVIDIA
            NV_NAME=$(nvidia-smi --query-gpu=name --format=csv,noheader 2>/dev/null | head -1 | xargs)
            GPU_INFO="WSL2 D3D12 GPU Passthrough ($NV_NAME)"
        else
            GPU_INFO="WSL2 D3D12 GPU Passthrough"
        fi
        echo -e "${GREEN}[✓] GPU: $GPU_INFO (Tăng tốc phần cứng D3D12)${NC}"
    fi
fi

if [ "$D3D12_OK" = false ]; then
    unset GALLIUM_DRIVER
    unset MESA_D3D12_DEFAULT_ADAPTER_NAME
    
    if command -v nvidia-smi &>/dev/null && nvidia-smi &>/dev/null; then
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
        GPU_INFO="Mesa Software Rendering (llvmpipe)"
        export LIBGL_ALWAYS_SOFTWARE=1
        echo -e "${YELLOW}[ℹ] D3D12 passthrough không khả dụng — dùng $GPU_INFO${NC}"
    fi
fi

# Ép gz-transport chạy trên loopback 127.0.0.1 (ngăn rớt gói dynamic_pose trên mạng ảo WSL2)
export GZ_IP=127.0.0.1
export GZ_PARTITION="drone_hitl"

if [ "$1" = "--split" ]; then
    echo -e "${YELLOW}[*] Khởi động Gazebo Server (Headless)...${NC}"
    gz sim -s -r "$WORLD_SDF" &
    GAZEBO_PID=$!
    sleep 2
    echo -e "${YELLOW}[*] Khởi động Gazebo GUI riêng biệt...${NC}"
    gz sim -g &
    GUI_PID=$!
else
    gz sim -r "$WORLD_SDF" &
    GAZEBO_PID=$!
fi

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

# Khởi động Trạm đo Oscilloscope & Telemetry Web (Cổng 8088)
OSC_SERVER="$SCRIPT_DIR/tools/web_oscilloscope/server.py"
[ ! -f "$OSC_SERVER" ] && OSC_SERVER="$SCRIPT_DIR/../tools/web_oscilloscope/server.py"

if [ -f "$OSC_SERVER" ]; then
    pkill -f "web_oscilloscope/server.py" 2>/dev/null || true
    python3 "$OSC_SERVER" --port 8088 &>/dev/null &
    OSC_PID=$!
    echo -e "${GREEN}[✓] Trạm đo Oscilloscope & Telemetry Web đã sẵn sàng: http://localhost:8088${NC}"
    # Tự động mở trình duyệt trên Windows
    if command -v powershell.exe &>/dev/null; then
        powershell.exe -NoProfile -Command "Start-Process 'http://localhost:8088'" 2>/dev/null || true
    fi
fi

# Khởi động Trạm điều khiển mặt đất QGroundControl (QGC)
QGC_WIN_EXE=""
for qpath in \
    "/mnt/c/Program Files/QGroundControl/bin/QGroundControl.exe" \
    "/mnt/c/Program Files/QGroundControl/QGroundControl.exe" \
    "/mnt/c/Program Files (x86)/qgroundcontrol/QGroundControl.exe" \
    "/mnt/c/Program Files (x86)/QGroundControl/QGroundControl.exe" \
    "/mnt/c/Users/ASUS/AppData/Local/Programs/QGroundControl/QGroundControl.exe"; do
    if [ -f "$qpath" ]; then
        QGC_WIN_EXE="$qpath"
        break
    fi
done

if [ -n "$QGC_WIN_EXE" ]; then
    echo -e "${YELLOW}[*] Khởi động Trạm mặt đất QGroundControl trên Windows...${NC}"
    WIN_QGC=$(wslpath -w "$QGC_WIN_EXE")
    ( cmd.exe /c start "" "$WIN_QGC" </dev/null >/dev/null 2>&1 ) &
    echo -e "${GREEN}[✓] QGroundControl đã mở trên màn hình Windows (UDP 14550)!${NC}"
elif command -v qgroundcontrol &>/dev/null; then
    echo -e "${YELLOW}[*] Khởi động QGroundControl Linux...${NC}"
    qgroundcontrol &>/dev/null &
    echo -e "${GREEN}[✓] QGroundControl Linux đã mở!${NC}"
fi

# Đồng bộ binary HITL Bridge mới nhất nếu có trong build/
if [ -f "$SCRIPT_DIR/../build/06_hitl_esp32_bridge" ]; then
    cp -u "$SCRIPT_DIR/../build/06_hitl_esp32_bridge" "$BRIDGE_BIN" 2>/dev/null || true
fi

# Hiển thị hướng dẫn
echo -e "${CYAN}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
echo -e "${BOLD}  📊 TRẠM ĐO SÓNG & ĐIỀU KHIỂN:${NC}"
echo -e "    ${GREEN}✓ Web Oscilloscope:${NC} http://localhost:8088"
echo -e "    ${GREEN}✓ QGroundControl:${NC}   Kết nối tự động qua UDP 14550"
echo -e "${CYAN}────────────────────────────────────────────────────────────${NC}"
echo -e "${BOLD}  🎮 QUY TRÌNH ĐIỀU KHIỂN TAY CẦM XBOX 360:${NC}"
echo -e "    1. Bấm ${BOLD}Y${NC}                  : ARM động cơ (quay nhẹ tại chỗ 200 rad/s)"
echo -e "    2. Đẩy ${BOLD}Stick trái ↑${NC}     : Tăng ga cất cánh mượt mà"
echo -e "    3. Kéo ${BOLD}Stick trái ↓${NC}     : Hạ cánh an toàn (chạm đất tự về Idle)"
echo -e "    4. Giữ ${BOLD}Stick trái ↓ kịch sàn 1.5s${NC} (hoặc bấm Y): DISARM tắt máy"
echo -e "    • Stick phải                 : Di chuyển (Tiến/Lùi/Trái/Phải)"
echo -e "    • Stick trái ↔ hoặc X / B    : Xoay Yaw góc hướng"
echo -e "    • Phím A                     : Hạ cánh tự động (Auto-Land)"
echo -e "    • Phím BACK / SPACE          : Phanh khẩn cấp / DISARM"
echo -e "    • LB + Stick phải            : LỘN NHÀO 360° (ACRO FLIP)"
echo -e "${CYAN}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
echo -e "${YELLOW}  Ctrl+C để tắt tất cả${NC}"
echo ""

# Chạy bridge
"$BRIDGE_BIN" "$ESP_PORT"
cleanup
