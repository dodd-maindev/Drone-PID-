# Hướng dẫn Nạp Firmware và Chạy Hardware-In-The-Loop (HITL) với ESP32

Thư mục này chứa toàn bộ mã nguồn firmware lõi điều khiển bay (Flight Controller) độc lập, chạy trên vi điều khiển **ESP32**, giao tiếp thời gian thực với **Gazebo Sim** trên máy tính qua cáp USB Serial ở tốc độ cao **921600 baud**.

---

## 1. Cấu trúc thư mục

```text
firmware_esp32/
├── firmware_esp32.ino          # File chính cho Arduino IDE (Mở và nạp ngay)
├── platformio.ini              # File cấu hình PlatformIO / VS Code
├── src/
│   └── main.cpp                # Mã nguồn điều khiển FreeRTOS Dual-Core (200Hz)
└── include/                    # Toàn bộ header lõi C++ thuần (Zero-Dependency)
    ├── telemetry_packet.hpp    # Cấu trúc gói tin nhị phân nén HITL
    ├── math_utils.hpp          # Biến đổi Quaternion, Euler, chuẩn hóa góc
    ├── pid.hpp                 # Lớp PID thuần toán học
    ├── altitude_controller.hpp # Bộ điều khiển độ cao (PI + Velocity Damping)
    ├── position_controller.hpp # Bộ điều khiển vị trí (Active Braking & Stop Anchor)
    └── motor_mixer.hpp         # Bộ trộn động cơ quadrotor chữ X
```

---

## 2. Hướng dẫn nạp code vào ESP32

Bạn có thể chọn **Cách 1 (Arduino IDE)** hoặc **Cách 2 (PlatformIO)**:

### Cách 1: Nạp bằng Arduino IDE (Đơn giản nhất)
1. Mở **Arduino IDE**.
2. Cài đặt package board ESP32 (nếu chưa có):
   - Vào `File` -> `Preferences` -> Thêm URL: `https://raw.githubusercontent.com/espressif/arduino-esp32/gh-pages/package_esp32_index.json` vào ô *Additional Boards Manager URLs*.
   - Vào `Tools` -> `Board` -> `Boards Manager...` -> Tìm **esp32** và nhấn **Install**.
3. Mở file `firmware_esp32/firmware_esp32.ino`.
4. Vào `Tools`:
   - **Board:** Chọn `ESP32 Dev Module` (hoặc `NodeMCU-32S`).
   - **Upload Speed:** `921600`.
   - **Port:** Chọn đúng cổng COM của ESP32 (ví dụ `COM3`, `COM4` trên Windows hoặc `/dev/ttyUSB0` trên Linux).
5. Nhấn nút **Upload (Mũi tên tròn)** để nạp code vào ESP32!

### Cách 2: Nạp bằng PlatformIO (VS Code)
1. Mở thư mục `firmware_esp32` trong **VS Code** có cài extension **PlatformIO IDE**.
2. Cắm cáp USB ESP32 vào máy tính.
3. Nhấn biểu tượng **PlatformIO Upload** (mũi tên ở thanh dưới cùng của VS Code) để biên dịch và nạp code.

---

## 3. Chạy Mô phỏng HITL trên Máy tính

### Bước 1: Khởi động thế giới vật lý Gazebo Sim
Mở Terminal 1 trên máy tính (WSL2/Linux) và chạy:
```bash
cd /home/do/drone_control_cpp
gz sim simulation/worlds/drone_world.sdf
```
*(Bấm nút Play màu cam ở góc dưới bên trái của Gazebo Sim để khởi động đồng hồ vật lý)*

### Bước 2: Kết nối ESP32 với WSL2 (Nếu dùng WSL2 trên Windows)
Mở PowerShell trên Windows với quyền Administrator và chạy:
```powershell
# Liệt kê các thiết bị USB
usbipd list

# Gắn cổng USB của ESP32 vào WSL (thay <busid> bằng ID của ESP32, ví dụ 1-4)
usbipd attach --wsl --busid <busid>
```
Kiểm tra trên Terminal WSL2:
```bash
ls -l /dev/ttyUSB0
# Nếu thấy thiết bị xuất hiện là đã kết nối thành công!
```

### Bước 3: Khởi động chương trình Cầu nối PC Bridge
Mở Terminal 2 và chạy:
```bash
cd /home/do/drone_control_cpp
./build/06_hitl_esp32_bridge /dev/ttyUSB0
```

---

## 4. Bàn phím điều khiển chuyến bay

Khi cầu nối hoạt động, bạn điều khiển máy bay trực tiếp bằng bàn phím (lệnh sẽ được chuyển sang ESP32 tính toán):

| Phím | Chức năng điều khiển |
| :---: | :--- |
| **`Q`** | **Arm động cơ & Tự động cất cánh lên 2.0m** |
| **`A`** | **Kích hoạt hạ cánh an toàn (Safe Land)** |
| **`Mũi tên` / `I K J L`** | Lái Tiến / Lùi / Trái / Phải (Nhả ra ESP32 tự phanh dứt khoát) |
| **`W` / `S`** | Tăng / Giảm độ cao |
| **`Z` / `C`** | Xoay đầu (Yaw) Trái / Phải |
| **`SPACE`** | Phanh khẩn cấp / Disarm |
| **`X`** | Thoát chương trình |
