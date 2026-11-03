@echo off
chcp 65001 >nul
:: Tu dong yeu cau quyen Administrator neu chua co
net session >nul 2>&1
if %errorlevel% neq 0 (
    echo [THONG BAO] Dang mo voi quyen Administrator...
    powershell -NoProfile -Command "Start-Process cmd -ArgumentList '/c \"\"%~f0\"\"' -Verb RunAs"
    exit /b
)

title Chuyen che do ESP32 -> Arduino IDE (Windows Host)
echo ======================================================================
echo   CHUYEN CHE DO: NAP CODE ESP32 TREN ARDUINO IDE (WINDOWS HOST)
echo ======================================================================
echo.
echo [1/3] Dang tim kiem ESP32 (CP210x / CH340 / FTDI)...

set FOUND=0
for /f "tokens=1,2,3" %%a in ('usbipd list ^| findstr /i "10c4:ea60 1a86:7523 0403:6001 CP210 CH340"') do (
    set FOUND=1
    echo [2/3] Tim thay ESP32 tai BUSID: %%a (VID:PID: %%b)
    echo       - Dang ngat ket noi khoi WSL2 (detach)...
    usbipd detach --busid %%a >nul 2>&1
    echo       - Dang tra lai quyen dieu khien cho Windows (unbind)...
    usbipd unbind --busid %%a >nul 2>&1
)

if %FOUND%==0 (
    echo [CANH BAO] Khong tim thay board ESP32 dang cam. Vui long cam lai cap USB!
) else (
    echo [3/3] Hoan tat! Cong COM tren Windows:
    echo.
    powershell -NoProfile -Command "Get-CimInstance Win32_SerialPort | Select-Object DeviceID, Name | Format-Table -AutoSize"
)

echo ======================================================================
echo HUONG DAN:
echo 1. Mo Arduino IDE tren Windows.
echo 2. O menu tren cung, nhan vao o chon Port/Board -> Chon cong COM moi (vi du COM6, COM7).
echo 3. Bam Nut Mui Ten [Upload] de nap code binh thuong!
echo ======================================================================
echo.
pause

