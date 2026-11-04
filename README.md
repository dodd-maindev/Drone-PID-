# Drone HITL Simulator (ESP32 + Gazebo + QGroundControl)

Hệ thống mô phỏng Hardware-In-The-Loop (HITL) thời gian thực dành cho máy bay 4 cánh (Quadrotor). Hệ thống cho phép nạp và chạy trực tiếp thuật toán điều khiển (PID, ITSMC) trên vi điều khiển thực tế **ESP32**, giao tiếp hai chiều với môi trường mô phỏng vật lý 3D **Gazebo Sim (Harmonic)** và trạm mặt đất **QGroundControl**.

---

## 🚀 Đặc tính nổi bật

* **Phần cứng thật (HITL):** Thuật toán điều khiển chạy 100% trên chip vật lý ESP32 (dual-core 240MHz, FPU phần cứng).
* **Mô phỏng vật lý chân thực:** Mô phỏng khí động học và cảm biến (IMU, Odometry, Barometer) trong Gazebo Sim Harmonic.
* **Giao tiếp tốc độ cao:** Serial nhị phân CRC16 tốc độ 921,600 baud (250Hz - 500Hz loop rate).
* **Điều khiển trực quan:** Hỗ trợ tay cầm Xbox 360 / Xbox One mượt mà trên Windows (XInput độ trễ cực thấp) hoặc phím tắt bàn phím.
* **Trạm đo sóng Web Oscilloscope:** Giao diện web thời gian thực hiển thị PFD (Artificial Horizon), biểu đồ sóng và thanh công suất 4 động cơ.
* **Tự động hóa hoàn toàn (1-Click Run):** Tự động phát hiện cổng ESP32, mở Gazebo Sim, Web Oscilloscope và QGroundControl.

---

## 📂 Cấu trúc thư mục dự án

```
Drone-PID-/
├── CMakeLists.txt                         # File cấu hình build CMake cho C++ Bridge
├── package.sh                             # Script tự động đóng gói trọn gói DroneHITL
├── run.sh                                 # Khởi chạy toàn bộ hệ thống bằng 1 lệnh duy nhất
│
├── CHUYEN_SANG_ARDUINO_NAP_CODE.bat       # [Windows] Phím tắt chuyển ESP32 sang nạp code Arduino
├── CHUYEN_SANG_HITL_WSL2.bat              # [Windows] Phím tắt chuyển ESP32 vào WSL2 để mô phỏng
├── chuyen_sang_arduino.sh                 # [WSL/Linux] Script chuyển chế độ nạp code
├── chuyen_sang_hitl.sh                    # [WSL/Linux] Script chuyển chế độ bay HITL
├── HUONG_DAN_FIX_LOI_ESP32_ARDUINO_IDE.md # Hướng dẫn sửa lỗi kết nối nạp code ESP32
│
├── firmware_esp32/                        # Mã nguồn Firmware nạp lên board ESP32
│   ├── firmware_esp32.ino                 # Code chính Arduino (Vòng lặp điều khiển, State Machine)
│   ├── flight_config.h                    # Thông số PID, ITSMC, giới hạn góc & gia tốc động cơ
│   ├── itsmc_controller.hpp               # Bộ điều khiển Integral Terminal Sliding Mode
│   └── mavlink_telemetry.hpp              # Giao thức truyền tin MAVLink
│
├── experiments/                           # Cầu nối C++ kết nối Gazebo <-> ESP32
│   └── 06_hitl_esp32_bridge.cpp           # Mã nguồn C++ HITL Bridge chính
│
├── simulation/                            # Môi trường mô phỏng 3D Gazebo
│   ├── worlds/drone_world.sdf             # Bản đồ mô phỏng 3D
│   ├── models/                            # 3D models (drone x500, tòa nhà, bãi đỗ...)
│   └── audio/                             # Âm thanh động cơ drone
│
├── tools/                                 # Các công cụ hỗ trợ
│   ├── web_oscilloscope/                  # Trạm đo sóng dao động Web (HTML5/JS/Canvas)
│   └── xbox_bridge/                       # Cầu nối tay cầm Xbox (C# XInput cho Windows)
│
├── matlab/                                # Các thuật toán mô phỏng lý thuyết
│   ├── itsmc.m                            # Mô phỏng thuật toán ITSMC trên MATLAB
│   └── on_critic_only.m                   # Mô phỏng học tăng cường Actor-Critic
│
├── scripts/                               # Script launcher gốc
│   └── run_hitl_launcher.sh               # Launcher script nguồn
│
├── DroneHITL/                             # Gói sản phẩm độc lập (chạy ngay không cần compile)
└── build/                                 # Thư mục build CMake
```

---

## 🛠️ Hướng dẫn sử dụng

### 1. Nạp code cho ESP32
- Chạy `CHUYEN_SANG_ARDUINO_NAP_CODE.bat` (trên Windows) hoặc `./chuyen_sang_arduino.sh` (trên WSL2).
- Mở **Arduino IDE**, mở file `firmware_esp32/firmware_esp32.ino`.
- Chọn board **ESP32 Dev Module** và cổng COM tương ứng, nhấn **Upload**.

### 2. Chạy mô phỏng HITL
Chỉ cần mở Terminal và chạy:
```bash
./run.sh
```
Hệ thống sẽ tự động:
1. Gắn cổng ESP32 vào môi trường mô phỏng.
2. Khởi chạy 3D Simulation Gazebo Harmonic.
3. Mở trạm đo sóng **Web Oscilloscope** tại `http://localhost:8088`.
4. Mở trạm mặt đất **QGroundControl** trên Windows.
5. Kích hoạt cầu nối tay cầm Xbox.

### 3. Quy trình điều khiển (Tay cầm Xbox 360 / One)
* **Bấm phím Y:** `ARM` động cơ (cánh quạt quay nhẹ tại chỗ 200 rad/s ở trạng thái chờ).
* **Đẩy cần trái (Throttle) ↑:** Tăng ga cất cánh mượt mà lên độ cao mong muốn.
* **Kéo cần trái (Throttle) ↓:** Hạ cánh an toàn (chạm đất tự động về tốc độ chờ Idle).
* **Giữ cần trái ↓ 1.5s (hoặc bấm Y):** `DISARM` tắt hẳn động cơ.
* **Cần phải:** Điều khiển tiến / lùi / trái / phải.
* **Phím A:** Tự động hạ cánh (Auto-Land).
* **Phím BACK / SPACE:** Phanh khẩn cấp & ngắt động cơ.
