@echo off
chcp 65001 >nul
net session >nul 2>&1
if %errorlevel% neq 0 (
    echo [THONG BAO] Dang mo voi quyen Administrator...
    powershell -NoProfile -Command "Start-Process cmd -ArgumentList '/c \"\"%~f0\"\"' -Verb RunAs"
    exit /b
)

title Chuyen che do ESP32 -> Chay HITL tren WSL2
echo ======================================================================
echo   CHUYEN CHE DO: CHAY MO PHONG HITL ESP32 TREN WSL2 (UBUNTU)
echo ======================================================================
echo.
echo [1/3] Dang tim kiem ESP32 (CP210x / CH340 / FTDI)...

set FOUND=0
for /f "tokens=1,2,3" %%a in ('usbipd list ^| findstr /i "10c4:ea60 1a86:7523 0403:6001 CP210 CH340"') do (
    set FOUND=1
    echo [2/3] Tim thay ESP32 tai BUSID: %%a (VID:PID: %%b)
    echo       - Dang bind vao usbipd...
    usbipd bind --force --busid %%a >nul 2>&1
    echo       - Dang attach vao WSL2...
    usbipd attach --wsl --busid %%a
)

if %FOUND%==0 (
    echo [CANH BAO] Khong tim thay board ESP32 dang cam. Vui long cam lai cap USB!
) else (
    echo.
    echo [3/3] Kiem tra trang thai hien tai:
    usbipd list | findstr /i "10c4:ea60 1a86:7523 0403:6001 Attached"
)

echo.
echo ======================================================================
echo HUONG DAN:
echo - Trong terminal WSL2 (Ubuntu), chay lenh: ls /dev/ttyUSB*
echo - ESP32 se xuat hien tai /dev/ttyUSB0.
echo - Gio ban co the chay ./run.sh de khoi dong toan bo he thong!
echo ======================================================================
echo.
pause

