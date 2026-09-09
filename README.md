# Drone-PID-

Bộ điều khiển máy bay 4 cánh (Quadrotor) thuần C++ từ con số 0, giao tiếp trực tiếp với môi trường mô phỏng vật lý **Gazebo Sim** qua thư viện `gz-transport` và `gz-msgs`.

---

## 🚀 Đặc tính nổi bật

* **Zero PX4 Dependency:** Bỏ qua hoàn toàn Autopilot trung gian (PX4/ArduPilot), giao tiếp trực tiếp bằng IPC / Shared Memory với Gazebo Sim.
* **Mô hình Pub / Subscribe:** 
  * Lắng nghe trạng thái Odometry (`/model/x500/odometry`).
  * Phát lệnh vận tốc góc cho 4 cánh quạt (`/x500/command/motor_speed`).
* **Cấu trúc điều khiển phân tầng (Cascaded Loop):**
  * **Vòng trong (Attitude Loop - 200Hz):** Cân bằng tư thế Roll/Pitch/Yaw bằng thuật toán PD dập tắt dao động nhanh.
  * **Vòng ngoài (Altitude Loop):** Bám độ cao đặt bằng PID kết hợp phanh vận tốc thẳng đứng (Velocity Damping) chống vọt lố.
  * **Motor Mixer & Desaturation:** Phối hợp tốc độ 4 cánh quạt khung chữ X theo hệ trục FLU, tự động bảo vệ mô-men thăng bằng chống lật úp.

---

## 📂 Cấu trúc thư mục

```
drone_control_cpp/
├── CMakeLists.txt          # File cấu hình build với gz-transport13 & gz-msgs10
├── BAO_CAO_TIEN_DO.md      # Tài liệu báo cáo chi tiết về thuật toán
├── include/                # Header files
│   ├── comm/               # Cầu nối Gazebo (GzBridge)
│   ├── controllers/        # Bộ điều khiển PID, Độ cao & Motor Mixer
│   ├── core/               # Vehicle Commander & Vehicle State
│   └── math/               # Toán học Quaternion, Euler, chuẩn hóa góc
├── src/                    # Source code triển khai C++
├── simulation/             # Tài nguyên mô phỏng độc lập
│   ├── models/             # Model x500 và x500_base
│   └── worlds/             # File thế giới drone_world.sdf
└── experiments/            # Các bài thí nghiệm điều khiển
    ├── 01_open_loop_thrust.cpp
    ├── 02_pid_altitude_hold.cpp
    ├── 03_attitude_maneuver.cpp
    └── 04_keyboard_teleop.cpp
```

---

## 🛠️ Hướng dẫn cài đặt & Chạy mô phỏng

### 1. Yêu cầu hệ thống
* Ubuntu 22.04 / 24.04 (hoặc WSL2)
* Gazebo Sim Harmonic (v8.x)
* Thư viện: `libgz-transport13-dev`, `libgz-msgs10-dev`
* Trình biên dịch: `cmake`, `g++` (C++17)

### 2. Biên dịch dự án
```bash
mkdir -p build && cd build
cmake ..
make -j$(nproc)
```

### 3. Khởi chạy
* **Terminal 1: Mở mô phỏng Gazebo**
  ```bash
  export GZ_SIM_RESOURCE_PATH=$(pwd)/simulation/models
  gz sim -r simulation/worlds/drone_world.sdf
  ```

* **Terminal 2: Chạy bài điều khiển C++**
  ```bash
  # Bài thí nghiệm giữ độ cao PID (cất cánh 2.0m -> lên 3.5m -> hạ cánh):
  ./build/02_pid_altitude_hold

  # Hoặc lái Drone bằng bàn phím (WASD / IJKL):
  ./build/04_keyboard_teleop
  ```
