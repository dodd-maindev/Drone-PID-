@echo off
chcp 65001 >nul
title Co Dinh Cong COM3 Cho ESP32
:: Kiem tra quyen Administrator
net session >nul 2>&1
if %errorlevel% neq 0 (
    echo [THONG BAO] Dang yeu cau quyen Administrator de can thiep Registry Windows...
    powershell -NoProfile -Command "Start-Process cmd -ArgumentList '/c \"\"%~f0\"\"' -Verb RunAs"
    exit /b
)

echo ======================================================================
echo   KHOA CHET CONG COM3 CHO ESP32 (XOA SACH PHANTOM PORTS TRONG REGISTRY)
echo ======================================================================
echo.

echo [1/3] Dang xoa sach bo dem ComDB Arbiter (COM3..COM255 in-use)...
powershell -NoProfile -Command "$db = New-Object byte[] 32; Set-ItemProperty -Path 'HKLM:\SYSTEM\CurrentControlSet\Control\COM Name Arbiter' -Name ComDB -Value $db -Force; if (Test-Path 'HKLM:\SYSTEM\CurrentControlSet\Control\COM Name Arbiter\Devices') { Remove-Item 'HKLM:\SYSTEM\CurrentControlSet\Control\COM Name Arbiter\Devices' -Recurse -Force }"

echo [2/3] Dang gan chet PortName = COM3 cho chip CP210x...
powershell -NoProfile -Command "$key = 'HKLM:\SYSTEM\CurrentControlSet\Enum\USB\VID_10C4&PID_EA60\0001'; if (Test-Path $key) { Set-ItemProperty -Path \"$key\Device Parameters\" -Name PortName -Value 'COM3' -Force; Set-ItemProperty -Path $key -Name FriendlyName -Value 'Silicon Labs CP210x USB to UART Bridge (COM3)' -Force }"

echo [3/3] Dang khoi dong lai driver CP210x tren Windows...
pnputil /restart-device "USB\VID_10C4&PID_EA60\0001" >nul 2>&1

echo.
echo ======================================================================
echo   DA KHOA THANH CONG! CONG HIEN TAI CUA ESP32:
echo ======================================================================
powershell -NoProfile -Command "Get-CimInstance Win32_SerialPort | Select-Object DeviceID, Name | Format-Table -AutoSize"
echo.
echo Tu nay tro di, ESP32 se duoc dinh danh vinh vien la COM3!
echo.
pause

