#!/bin/bash
echo "======================================================"
echo "  CHUYỂN CHẾ ĐỘ: HITL SIMULATION (WSL2 / LINUX)"
echo "======================================================"

USBIPD="/mnt/c/Program Files/usbipd-win/usbipd.exe"
POWERSHELL="/mnt/c/Windows/System32/WindowsPowerShell/v1.0/powershell.exe"

# Giải phóng cổng serial cũ trước khi kết nối để đảm bảo luôn dùng /dev/ttyUSB0
fuser -k /dev/ttyUSB* /dev/ttyACM* 2>/dev/null || true
pkill -9 -f 06_hitl_esp32_bridge 2>/dev/null || true

# Tìm BUSID của thiết bị CP210x / CH340
BUSID=$("$USBIPD" list 2>/dev/null | grep -iE "10c4:ea60|1a86:7523|0403:6001|CP210|CH340" | awk '{print $1}' | head -n 1)

if [ -z "$BUSID" ]; then
    echo "[!] Không tìm thấy board ESP32 đang cắm. Vui lòng kiểm tra lại dây cáp!"
    exit 1
fi

echo "[*] Tìm thấy ESP32 tại BUSID: $BUSID"
echo "[*] Đang cấp quyền và gắn vào WSL2..."

"$POWERSHELL" -NoProfile -Command "Start-Process 'C:\Program Files\usbipd-win\usbipd.exe' -ArgumentList 'bind --force --busid $BUSID' -Verb RunAs -Wait" 2>/dev/null
"$USBIPD" attach --wsl --busid "$BUSID" 2>/dev/null

sleep 1
sudo chmod 666 /dev/ttyUSB* /dev/ttyACM* 2>/dev/null || true
if [ ! -e "/dev/ttyUSB0" ] && [ -e "/dev/ttyUSB1" ]; then
    sudo ln -sf /dev/ttyUSB1 /dev/ttyUSB0 2>/dev/null || true
fi

if ls /dev/ttyUSB* 1>/dev/null 2>&1; then
    echo "[+] THÀNH CÔNG! Thiết bị đã sẵn sàng tại: $(ls /dev/ttyUSB*)"
else
    echo "[!] Đang kiểm tra lại trạng thái:"
    "$USBIPD" list | grep -iE "$BUSID|10c4:ea60"
fi
