#!/bin/bash
echo "======================================================"
echo "  CHUYỂN CHẾ ĐỘ: NẠP CODE TỔNG HỢP (WINDOWS HOST)"
echo "======================================================"

USBIPD="/mnt/c/Program Files/usbipd-win/usbipd.exe"

# 1. Detach khỏi WSL2
"$USBIPD" detach --hardware-id 10c4:ea60 2>/dev/null || true
"$USBIPD" detach --hardware-id 1a86:7523 2>/dev/null || true

# 2. Tìm BUSID thực tế trong danh sách Connected
BUSID=$("$USBIPD" list 2>/dev/null | grep -E '^[0-9]+-[0-9]+' | grep -iE "10c4:ea60|1a86:7523|0403:6001|CP210|CH340" | awk '{print $1}' | head -n 1)

if [ -n "$BUSID" ]; then
    echo "[*] Tìm thấy ESP32 tại BUSID: $BUSID"
    "$USBIPD" detach --busid "$BUSID" 2>/dev/null || true
fi

echo ""
echo "======================================================"
echo "  HƯỚNG DẪN MỞ KHÓA CỔNG COM TRÊN ARDUINO IDE:"
echo "======================================================"
echo "  Do Windows bảo mật (usbipd giữ quyền Administrator),"
echo "  để Arduino IDE nhận ngay cổng COM, bạn chỉ cần:"
echo ""
echo "  👉 CÁCH NHANH NHẤT (Khuyên dùng):"
echo "     Rút cáp USB của ESP32 ra rồi cắm lại vào máy tính!"
echo "     (Windows sẽ tự động gán lại cổng COM chuẩn ngay lập tức)"
echo ""
echo "  👉 CÁCH 2:"
echo "     Ra ngoài Windows, double-click chạy file:"
echo "     CHUYEN_SANG_ARDUINO_NAP_CODE.bat (chọn Yes khi hỏi Admin)"
echo "======================================================"
