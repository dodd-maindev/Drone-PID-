#!/bin/bash
echo "======================================================"
echo "  CHUYỂN CHẾ ĐỘ: NẠP CODE TỔNG HỢP (WINDOWS HOST)"
echo "======================================================"

USBIPD="/mnt/c/Program Files/usbipd-win/usbipd.exe"
POWERSHELL="/mnt/c/Windows/System32/WindowsPowerShell/v1.0/powershell.exe"

# Tìm BUSID của thiết bị CP210x / CH340
BUSID=$("$USBIPD" list 2>/dev/null | grep -iE "10c4:ea60|1a86:7523|0403:6001|CP210|CH340" | awk '{print $1}' | head -n 1)

if [ -z "$BUSID" ]; then
    echo "[!] Không tìm thấy board ESP32. Vui lòng kiểm tra lại dây cáp!"
    exit 1
fi

echo "[*] Tìm thấy ESP32 tại BUSID: $BUSID"
echo "[*] Đang ngắt kết nối khỏi WSL2 và trả về Windows..."

"$USBIPD" detach --busid "$BUSID" 2>/dev/null
"$POWERSHELL" -NoProfile -Command "Start-Process 'C:\Program Files\usbipd-win\usbipd.exe' -ArgumentList 'unbind --busid $BUSID' -Verb RunAs -Wait" 2>/dev/null

sleep 1
echo "[+] THÀNH CÔNG! Đã trả quyền điều khiển về Windows."
echo "[+] Cổng COM trên Windows hiện tại:"
"$POWERSHELL" -NoProfile -Command "Get-CimInstance Win32_SerialPort | Select-Object DeviceID, Name | Format-Table -AutoSize" 2>/dev/null

