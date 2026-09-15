# TÀI LIỆU GIẢI THÍCH CHI TIẾT TOÀN BỘ KIẾN TRÚC DỰ ÁN DRONE_CONTROL_CPP

> **Mục tiêu dự án:** Phát triển hệ thống Flight Control thuần C++ (C++17) điều khiển trực tiếp mô hình quadrotor `x500` trong môi trường mô phỏng **Gazebo Sim Harmonic (8.15.0)** thông qua giao thức **GZ-Transport IPC**.  
> Hệ thống hoạt động **độc lập 100%**, không phụ thuộc vào PX4-Autopilot, ArduPilot, ROS hay MAVLink, tối ưu hóa hiệu năng thời gian thực và tự chủ thuật toán điều khiển từ gốc.

---

## MỤC LỤC
1. [Sơ đồ cấu trúc tổng thể thư mục (Project Tree)](#1-sơ-đồ-cấu-trúc-tổng-thể-thư-mục)
2. [Kiến trúc phân tầng điều khiển (Control Architecture)](#2-kiến-trúc-phân-tầng-điều-khiển)
3. [Giải thích chi tiết từng thư mục, từng file và từng hàm](#3-giải-thích-chi-tiết-từng-thư-mục-từng-file-và-từng-hàm)
   - [Thư mục gốc (Root)](#a-thư-mục-gốc-root)
   - [Thư mục `include/` & `src/` (Thư viện lõi `drone_lib`)](#b-thư-mục-include-và-src-thư-viện-lõi-drone_lib)
     - [comm/ (Giao tiếp Gazebo)](#b1-thư-mục-comm-giao-tiếp-mô-phỏng)
     - [core/ (Quản lý phương tiện & Trạng thái)](#b2-thư-mục-core-quản-lý-phương-tiện--trạng-thái)
     - [controllers/ (Thuật toán điều khiển PID & Mixer)](#b3-thư-mục-controllers-thuật-toán-điều-khiển)
     - [math/ (Toán học động học & Động lực học)](#b4-thư-mục-math-toán-học-hàng-không)
     - [utils/ (Ghi log dữ liệu Blackbox)](#b5-thư-mục-utils-ghi-nhận-dữ-liệu-hộp-đen)
   - [Thư mục `experiments/` (Các bài thí nghiệm thực thi)](#c-thư-mục-experiments-các-chương-trình-thí-nghiệm-chính)
   - [Thư mục `simulation/` (Mô hình và thế giới Gazebo)](#d-thư-mục-simulation-mô-hình--thế-giới-mô-phỏng)
   - [Thư mục `log/` (Phân tích dữ liệu chuyến bay)](#e-thư-mục-log-dữ-liệu-và-công-cụ-vẽ-đồ-thị)
4. [Bảng tần số các vòng lặp trong hệ thống](#4-bảng-tần-số-các-vòng-lặp-trong-hệ-thống)

---

## 1. SƠ ĐỒ CẤU TRÚC TỔNG THỂ THƯ MỤC

```text
drone_control_cpp/
├── CMakeLists.txt                  # Cấu hình biên dịch CMake C++17 và liên kết thư viện Gazebo
├── README.md                       # Giới thiệu tổng quan dự án
├── EXPLAIN.md                      # [Tài liệu này] Bách khoa toàn thư giải thích toàn bộ mã nguồn
│
├── experiments/                    # 3 bài thí nghiệm bay chính (File thực thi C++)
│   ├── 02_pid_altitude_hold.cpp   # Bài 2: Giữ độ cao PID tự động (2.0m -> 3.5m -> Hạ cánh khóa vị trí)
│   ├── 04_keyboard_teleop.cpp     # Bài 4: Lái phím ảo (Phím A hạ cánh, Q bay lên, khóa mỏ neo ảo)
│   └── 05_yaw_rotation_control.cpp# Bài 5: Tự xoay 360°/720° quanh trục đứng Yaw có phanh pha trước
│
├── include/                        # Header files (Khai báo class, struct, hàm)
│   ├── comm/
│   │   └── gz_bridge.hpp           # Header cầu nối Gazebo Transport
│   ├── controllers/
│   │   ├── altitude_controller.hpp # Header bộ điều khiển độ cao
│   │   ├── motor_mixer.hpp         # Header bộ trộn công suất 4 cánh quạt (Motor Mixer chữ X)
│   │   └── pid.hpp                 # Header lớp PID thuần C++
│   ├── core/
│   │   ├── vehicle_commander.hpp   # Header lớp điều phối tổng thể drone
│   │   └── vehicle_state.hpp       # Struct lưu trữ toàn bộ dữ liệu cảm biến đa luồng (Atomic)
│   ├── math/
│   │   └── math_utils.hpp          # Header toán học hàng không (Euler <-> Quaternion, Clamp, Normalize)
│   └── utils/
│       └── data_logger.hpp         # Header ghi log dữ liệu Blackbox ra CSV
│
├── src/                            # Source files (Hiện thực hóa logic các class)
│   ├── comm/
│   │   └── gz_bridge.cpp           # Hiện thực hóa giao tiếp Gz-Transport (Sub Odom, Pub Motor)
│   ├── controllers/
│   │   └── altitude_controller.cpp # Hiện thực hóa giải thuật PID độ cao + Velocity Damping
│   ├── core/
│   │   └── vehicle_commander.cpp   # Vòng lặp điều khiển tư thế 200Hz, tính mô-men, gửi motor
│   └── utils/
│       └── data_logger.cpp         # Hiện thực hóa việc ghi log thread-safe sang file CSV
│
├── simulation/                     # Tài nguyên mô phỏng độc lập
│   ├── models/
│   │   ├── x500/                   # Định nghĩa SDF của x500 (gồm 4 plugin motor + 1 plugin odometry)
│   │   └── x500_base/              # Khung vỏ, vật liệu, mesh 3D và ma trận quán tính
│   └── worlds/
│       └── drone_world.sdf         # Môi trường thế giới Gazebo (trọng trường 9.8m/s², vật lý ODE 250Hz)
│
└── log/                            # Lưu trữ và phân tích dữ liệu bay
    ├── latest_flight.csv           # File log chuyến bay gần nhất
    ├── flight_telemetry_*.csv      # Lịch sử các chuyến bay trước
    └── plot_flight.py              # Script Python tự động vẽ đồ thị từ file CSV
```

---

## 2. KIẾN TRÚC PHÂN TẦNG ĐIỀU KHIỂN (CASCADED MULTI-RATE ARCHITECTURE)

Hệ thống được thiết kế theo mô hình điều khiển phân tầng chuẩn ngành hàng không:

```
[Người dùng / Kịch bản bay] (Tần số: ~30 - 33 Hz)
   |
   +---> Đặt mục tiêu: Độ cao Z_target, Góc nghiêng Roll/Pitch_target, Góc xoay Yaw_target
   |
[Vòng lặp bên ngoài - Outer Loop] (Tần số: ~33 Hz - Thư mục `experiments/`)
   |---> Tính PID Độ cao: Z_error -> Thrust_base
   |---> Bù góc nghiêng: Total_Thrust = Thrust_base / cos(tilt)
   |---> Phanh vị trí mỏ neo ảo: V_x, V_y -> Roll_correction, Pitch_correction
   |
[Vòng lặp bên trong - Inner Loop] (Tần số: ~200 Hz - File `vehicle_commander.cpp`)
   |---> Đọc Roll, Pitch, Yaw, p, q, r từ `VehicleState`
   |---> Bộ điều khiển tư thế (Attitude PID): Sai số góc -> Mô-men yêu cầu (tau_roll, tau_pitch, tau_yaw)
   |---> Bộ trộn động cơ (Motor Mixer chữ X): Thrust + Tau -> Vận tốc 4 cánh quạt (rad/s)
   |---> Kỹ thuật Mixer Desaturation: Chống bão hòa công suất, ưu tiên giữ thăng bằng
   |
[Cầu nối truyền thông GzBridge] (Topic: `/x500/command/motor_speed`)
   |
[Mô phỏng vật lý Gazebo Sim] (Tần số giải tích ODE: 250 Hz - Topic: `/model/x500/odometry`)
```

---

## 3. GIẢI THÍCH CHI TIẾT TỪNG THƯ MỤC, TỪNG FILE VÀ TỪNG HÀM

### A. THƯ MỤC GỐC (ROOT)

#### 1. `CMakeLists.txt`
* **Chức năng:** Kịch bản cấu hình build hệ thống bằng CMake.
* **Các thành phần quan trọng:**
  * `find_package(gz-transport13 REQUIRED)` & `find_package(gz-msgs10 REQUIRED)`: Tìm kiếm thư viện giao tiếp liên tiến trình của Gazebo Harmonic.
  * `add_library(drone_lib STATIC ...)`: Đóng gói 4 file lõi (`gz_bridge.cpp`, `altitude_controller.cpp`, `vehicle_commander.cpp`, `data_logger.cpp`) thành thư viện tĩnh `libdrone_lib.a`.
  * `add_executable(...)`: Liên kết thư viện `drone_lib` để tạo ra 3 file thực thi trong thư mục `build/`:
    * `02_pid_altitude_hold`
    * `04_keyboard_teleop`
    * `05_yaw_rotation_control`

---

### B. THƯ MỤC `include/` VÀ `src/` (THƯ VIỆN LÕI `drone_lib`)

#### B.1. Thư mục `comm/` (Giao tiếp mô phỏng)

##### File `include/comm/gz_bridge.hpp` & `src/comm/gz_bridge.cpp`
* **Chức năng:** Đóng vai trò là cầu nối truyền thông thời gian thực giữa mã C++ và mô phỏng Gazebo Sim thông qua IPC (Inter-Process Communication).
* **Các hàm thành viên:**
  * `GzBridge()`: Khởi tạo con trỏ `node_` của thư viện `gz::transport::Node`.
  * `~GzBridge()`: Gọi `shutdown()` để ngắt kết nối an toàn.
  * `bool init(VehicleState* state_ptr, const std::string& odom_topic, const std::string& motor_topic)`:
    * Lưu con trỏ trạng thái `state_ptr`.
    * Đăng ký nhận dữ liệu (Subscribe) từ topic Odometry (`/model/x500/odometry`). Khi có dữ liệu, Gazebo sẽ tự động kích hoạt callback `on_odometry`.
    * Đăng ký kênh phát lệnh (Advertise) mảng vận tốc động cơ kiểu `gz::msgs::Actuators` vào topic `/x500/command/motor_speed`.
  * `void on_odometry(const gz::msgs::Odometry& msg)`:
    * **Hàm callback tự động gọi khi Gazebo gửi dữ liệu**:
    * Trích xuất tọa độ 3D: $X, Y, Z$ (`msg.pose().position()`).
    * Trích xuất vận tốc tịnh tiến: $V_x, V_y, V_z$ (`msg.twist().linear()`).
    * Trích xuất Quaternion $[w, x, y, z]$, gọi `DroneMath::quaternion_to_euler` để chuyển hóa sang góc Euler $Roll, Pitch, Yaw$.
    * Trích xuất vận tốc góc: $p, q, r$ (`msg.twist().angular()`).
    * Ghi toàn bộ dữ liệu này vào struct `VehicleState` một cách thread-safe. Đánh dấu cờ `is_connected = true`.
  * `void send_motor_velocities(const std::array<float, 4>& speeds)`:
    * Đóng gói vận tốc quay $\omega_0, \omega_1, \omega_2, \omega_3$ (rad/s) vào protobuf `gz::msgs::Actuators`.
    * Bơm gói tin sang Gazebo để điều khiển trực tiếp 4 khớp cánh quạt.
  * `void shutdown()`: Gửi mảng $\{0, 0, 0, 0\}$ để dừng hẳn 4 motor khi thoát chương trình và hủy node.

---

#### B.2. Thư mục `core/` (Quản lý phương tiện & Trạng thái)

##### File `include/core/vehicle_state.hpp`
* **Chức năng:** Định nghĩa struct `VehicleState` lưu trữ toàn bộ trạng thái cảm biến (Telemetry) và chỉ số điều khiển của drone.
* **Đặc điểm kỹ thuật:** Toàn bộ các biến thành viên đều sử dụng `std::atomic<float>` hoặc `std::atomic<bool>`:
  * Cho phép nhiều luồng (Luồng mạng GzBridge, Luồng điều khiển 200Hz, Luồng ứng dụng người dùng) đọc và ghi dữ liệu đồng thời mà **không bị Race Condition, không bị khóa luồng (Lock-free)**.
* **Các trường dữ liệu chính:**
  * `pos_x, pos_y, altitude`: Tọa độ vị trí 3D (mét).
  * `vx, vy, vz`: Vận tốc tịnh tiến 3 trục (m/s).
  * `roll, pitch, yaw`: Góc nghiêng tư thế (radian).
  * `p, q, r`: Tốc độ quay góc thân máy bay quanh 3 trục (rad/s).
  * `motor_speed_0, 1, 2, 3`: Vận tốc thực tế gửi đến 4 động cơ (rad/s).
  * `current_thrust, tau_roll, tau_pitch, tau_yaw`: Lực nâng và các mô-men điều khiển đầu ra.
  * `is_armed, is_connected`: Trạng thái mở khóa động cơ và trạng thái kết nối.

##### File `include/core/vehicle_commander.hpp` & `src/core/vehicle_commander.cpp`
* **Chức năng:** Trái tim điều phối của toàn bộ drone. Quản lý trạng thái Arm/Disarm, nhận lệnh góc/lực nâng từ người dùng, chạy luồng tính toán tư thế 200Hz và phân bổ động cơ.
* **Các hàm thành viên:**
  * `VehicleCommander()`: Khởi tạo các đối tượng con, thiết lập các giá trị ban đầu và khởi chạy luồng nền `control_thread_` chạy hàm `control_worker()`.
  * `~VehicleCommander()`: Tắt cờ `running_`, đợi luồng kết thúc (`join()`) và dọn dẹp tài nguyên.
  * `bool start(int dummy_port = 0)`: Khởi tạo cầu nối `gz_bridge_.init(&state_)`.
  * `void stop()`: Tắt cầu nối và dừng hoạt động.
  * `bool wait_for_connection(int timeout_seconds = 10)`: Chờ tối đa `timeout_seconds` cho đến khi `state_.is_connected` bật `true`.
  * `void arm(bool force = true)`: Kích hoạt cờ `is_armed = true`, cho phép động cơ quay.
  * `void disarm()`: Tắt cờ `is_armed = false`, gửi ngay lập tức lệnh dừng 4 động cơ về 0.
  * `void set_offboard_mode()`: Xác nhận chuyển sang chế độ tự chủ điều khiển trực tiếp (Direct Control).
  * `void send_attitude_target(float roll, float pitch, float yaw, float thrust)`:
    * Nhận góc mong muốn (Roll, Pitch, Yaw tính bằng rad) và Lực nâng tổng $Thrust \in [0.0, 1.0]$.
    * Lưu nguyên tử vào các biến `target_roll_`, `target_pitch_`, `target_yaw_`, `target_thrust_`.
  * `const VehicleState& state() const`: Trả về tham chiếu trạng thái để các bài thí nghiệm đọc dữ liệu.
  * `DataLogger& logger()`: Trả về tham chiếu đối tượng ghi log.
  * `void control_worker()`:
    * **Vòng lặp điều khiển tư thế chạy ngầm ở tần số 200Hz (chu kỳ 5ms)**:
    * Nếu `!is_armed` hoặc `target_thrust <= 0.02`: Reset tích phân I, gửi 0 rad/s cho 4 motor.
    * Tính sai số góc nghiêng: $e_{roll} = Roll_{target} - Roll_{actual}$, $e_{pitch} = Pitch_{target} - Pitch_{actual}$.
    * Tích lũy tích phân góc $I_{roll}, I_{pitch}$ kèm bộ kẹp chống bão hòa (Anti-windup clamp $\pm 0.05$).
    * Tính mô-men phản hồi góc theo công thức:
      $$\tau_{roll} = K_{p} e_{roll} + K_{i} \int e_{roll} dt - K_{d} \cdot p$$
      $$\tau_{pitch} = K_{p} e_{pitch} + K_{i} \int e_{pitch} dt - K_{d} \cdot q$$
      $$\tau_{yaw} = K_{p,yaw} (Yaw_{target} - Yaw_{actual}) - K_{d,yaw} \cdot r$$
    * Đưa $Thrust, \tau_{roll}, \tau_{pitch}, \tau_{yaw}$ qua bộ trộn `mixer_.compute_motor_speeds(...)` để sinh ra vận tốc 4 cánh quạt $\omega_0, \omega_1, \omega_2, \omega_3$.
    * Bơm mảng vận tốc này sang Gazebo qua `gz_bridge_.send_motor_velocities(...)`.
    * Định kỳ mỗi $20\text{ms}$ (tần số 50Hz), ghi một bản ghi vào `logger_`.

---

#### B.3. Thư mục `controllers/` (Thuật toán điều khiển PID & Mixer)

##### File `include/controllers/pid.hpp`
* **Chức năng:** Lớp điều khiển PID 1 chiều tổng quát, thuần C++, có giới hạn đầu ra và giới hạn tích phân.
* **Các hàm thành viên:**
  * `PIDController(kp, ki, kd, output_min, output_max, integral_max)`: Cấu hình các hệ số khuếch đại và ngưỡng bão hòa.
  * `void reset()`: Đặt lại tích phân `integral_ = 0.0` và cờ khởi động ban đầu.
  * `double update(double setpoint, double measurement, double dt)`:
    * Tính sai số $e = Setpoint - Measurement$.
    * Tính thành phần P: $P = K_p \cdot e$.
    * Tính thành phần I: $I = I_{old} + K_i \cdot e \cdot dt$, kẹp $I \in [-integral\_max, integral\_max]$ (Anti-windup).
    * Tính thành phần D: $D = K_d \cdot \frac{e - e_{prev}}{dt}$.
    * Tổng hợp $Output = \text{clamp}(P + I + D, output\_min, output\_max)$.

##### File `include/controllers/altitude_controller.hpp` & `src/controllers/altitude_controller.cpp`
* **Chức năng:** Bộ điều khiển độ cao chuyên biệt cho Quadcopter x500 trong môi trường mô phỏng Gazebo.
* **Đặc điểm thiết kế:** Điểm cân bằng tĩnh lý thuyết của x500 ($m=2.06\text{kg}$, $g=9.8$, 4 motor có $k_m = 8.55 \times 10^{-6}$) nằm ở mức **Hover Thrust = 0.59**.
* **Các hàm thành viên:**
  * `AltitudeController(hover_thrust, kp, ki, kd)`: Khởi tạo điểm hover (0.59) và PID bên trong.
  * `void reset()`: Gọi `pid_.reset()` để xóa tích phân cũ.
  * `float compute_thrust(double target_alt, double current_alt, double vz, double dt)`:
    * Tính lượng bù ga: $\Delta Thrust = \text{PID}(target\_alt, current\_alt, dt)$.
    * **Cơ chế phanh vi phân vận tốc rơi (Velocity Damping):** Trừ bớt $K_v \cdot V_z$ ($K_v = 0.12$). Khi drone leo lên quá nhanh ($V_z > 0$), ga sẽ tự động hãm lại trước khi chạm đích $\to$ **Triệt tiêu hoàn toàn hiện tượng vọt lố (Zero Overshoot)**.
    * Tổng lực nâng: $Thrust = Hover + \Delta Thrust - K_v \cdot V_z$.
    * Giới hạn an toàn: Kẹp trong khoảng $[0.10, 0.85]$ để luôn chừa lại ít nhất $15\%$ công suất động cơ phục vụ cho việc cân bằng góc nghiêng.
  * `float compute_thrust(double target_alt, double current_alt, double dt)`: Overload khi không có vận tốc $V_z$.
  * `void set_hover_thrust(double hover_thrust)`: Cho phép thay đổi lực nâng tĩnh khi tải trọng drone thay đổi.

##### File `include/controllers/motor_mixer.hpp`
* **Chức năng:** Bộ trộn động cơ (Motor Mixer) cho cấu hình quadrotor khung chữ X (Quad X).
* **Quy ước vị trí động cơ x500 (theo hệ trục FLU của Gazebo):**
  * Motor 0: Phía trước - Bên phải (+X, -Y) $\to$ Quay ngược chiều kim đồng hồ (CCW).
  * Motor 1: Phía sau - Bên trái (-X, +Y) $\to$ Quay ngược chiều kim đồng hồ (CCW).
  * Motor 2: Phía trước - Bên trái (+X, +Y) $\to$ Quay cùng chiều kim đồng hồ (CW).
  * Motor 3: Phía sau - Bên phải (-X, -Y) $\to$ Quay cùng chiều kim đồng hồ (CW).
* **Các hàm thành viên:**
  * `MotorMixer(max_rot_velocity = 1000.0f)`: Thiết lập tốc độ quay trần của động cơ x500 ($1000\text{ rad/s}$).
  * `std::array<float, 4> compute_motor_speeds(thrust, roll, pitch, yaw, is_armed)`:
    * Tính toán lực nâng chuẩn hóa $u_i \in [0.0, 1.0]$ cho từng động cơ:
      $$u_0 = Thrust - Roll - Pitch - Yaw \quad (\text{Front-Right CCW})$$
      $$u_1 = Thrust + Roll + Pitch - Yaw \quad (\text{Rear-Left CCW})$$
      $$u_2 = Thrust + Roll - Pitch + Yaw \quad (\text{Front-Left CW})$$
      $$u_3 = Thrust - Roll + Pitch + Yaw \quad (\text{Rear-Right CW})$$
    * **Kỹ thuật chống bão hòa công suất (Mixer Desaturation):** Nếu có một động cơ bị yêu cầu vượt quá $1.0$, thuật toán sẽ trừ đều phần thừa trên cả 4 động cơ. Điều này giúp độ cao có thể suy giảm nhẹ nhưng **tuyệt đối không bị mất điều khiển thăng bằng góc**.
    * **Chuyển đổi lực thành tốc độ quay:** Vì lực nâng tỉ lệ với bình phương tốc độ quay ($F = k_m \cdot \omega^2$), nên:
      $$\omega_i = \omega_{max} \cdot \sqrt{\text{clamp}(u_i, 0, 1)}$$

---

#### B.4. Thư mục `math/` (Toán học hàng không)

##### File `include/math/math_utils.hpp`
* **Chức năng:** Cung cấp các công thức chuyển đổi hình học tọa độ và góc xoay hàng không.
* **Các hàm nội tuyến (inline functions):**
  * `euler_to_quaternion(roll, pitch, yaw)`: Chuyển 3 góc Euler (radian) sang Quaternion $[w, x, y, z]$.
  * `quaternion_to_euler(w, x, y, z, roll, pitch, yaw)`: Chuyển Quaternion từ cảm biến Odometry thành 3 góc Euler Roll, Pitch, Yaw.
  * `normalize_angle(angle)`: Đưa mọi góc xoay bất kỳ về khoảng chuẩn $[-\pi, +\pi]$ (loại bỏ vấn đề bước nhảy từ $+180^\circ$ sang $-180^\circ$).
  * `deg2rad(deg)`: Đổi từ độ sang radian.
  * `rad2deg(rad)`: Đổi từ radian sang độ.
  * `clamp(val, min_val, max_val)`: Giới hạn giá trị nằm trong ngưỡng an toàn.

---

#### B.5. Thư mục `utils/` (Ghi nhận dữ liệu hộp đen)

##### File `include/utils/data_logger.hpp` & `src/utils/data_logger.cpp`
* **Chức năng:** Ghi lại toàn bộ dữ liệu điều khiển và cảm biến thời gian thực của chuyến bay ra định dạng CSV để phục vụ phân tích.
* **Struct `FlightLogEntry`:** Lưu trữ 1 dòng dữ liệu đầy đủ gồm: thời gian (`time_s`), lực ga (`target_thrust`), góc đặt/thực tế (`roll, pitch, yaw`), vận tốc rơi (`vz`), tốc độ góc (`p, q, r`), mô-men (`tau_roll, tau_pitch, tau_yaw`) và tốc độ 4 cánh quạt (`w0, w1, w2, w3`).
* **Các hàm trong class `DataLogger`:**
  * `bool start(log_dir, prefix)`: Tạo thư mục log nếu chưa có, sinh tên file dạng `flight_log_YYYYMMDD_HHMMSS.csv`, đồng thời mở file `latest_flight.csv` để ghi đè dữ liệu mới nhất.
  * `void log(const FlightLogEntry& entry)`: Ghi 1 mẫu dữ liệu vào file (được bảo vệ bởi `std::lock_guard<std::mutex>` để chống xung đột luồng).
  * `void stop()`: Đóng file an toàn khi kết thúc chuyến bay.
  * `get_current_log_path()`, `get_latest_log_path()`, `get_records_count()`: Các hàm truy vấn đường dẫn và số lượng mẫu đã ghi.

---

### C. THƯ MỤC `experiments/` (CÁC CHƯƠNG TRÌNH THÍ NGHIỆM CHÍNH)

#### 1. File `experiments/02_pid_altitude_hold.cpp`
* **Tên bài:** Thí nghiệm Giữ độ cao PID và Khóa cứng vị trí X-Y.
* **Mục tiêu:** Cất cánh êm như thang máy lên $2.0\text{ m}$, nâng tiếp lên $3.5\text{ m}$, sau đó tự động hạ cánh thẳng tắp về mặt đất mà không bị trôi dạt tọa độ.
* **Các hàm và khối logic:**
  * `compute_position_lock_tilt(corr_roll, corr_pitch)`: Thuật toán PD vị trí:
    * Tính sai số tọa độ: $e_x = X_0 - X_{actual}$, $e_y = Y_0 - Y_{actual}$.
    * Bù góc nghiêng ngược chiều vận tốc trôi:
      $$Pitch_{corr} = 1.2 \cdot e_x - 1.5 \cdot V_x$$
      $$Roll_{corr} = -1.2 \cdot e_y + 1.5 \cdot V_y$$
    * Khóa chặt góc bù trong phạm vi $\pm 2.5^\circ$ để drone giữ vững tâm tọa độ ban đầu.
  * `main()`:
    * **Khởi tạo:** Kết nối Gazebo, gửi mồi 30Hz, Arm động cơ.
    * **Bước 1 (Leo lên 2.0m):** Ramping mục tiêu tăng dần với tốc độ giới hạn $0.75\text{ m/s}$ trong 6 giây.
    * **Bước 2 (Nâng lên 3.5m):** Tiếp tục ramp mượt mà từ $2.0\text{ m}$ lên $3.5\text{ m}$, giữ lơ lửng ổn định không vọt lố.
    * **Bước 3 (Hạ cánh thẳng đứng):** Hạ dần mục tiêu về $0.08\text{ m}$, liên tục khóa tọa độ X-Y.
    * **Bước 4 (Chạm đất):** Xả ga về 0 và gọi `drone.disarm()` tắt động cơ an toàn.

#### 2. File `experiments/04_keyboard_teleop.cpp`
* **Tên bài:** Bàn phím điều khiển thời gian thực (Virtual RC Teleop) với cơ chế Mỏ neo ảo, Phím A (Hạ cánh) và Phím Q (Bay lên tiếp).
* **Các hàm và thành phần:**
  * `set_nonblocking_terminal(bool enable)`: Sử dụng thư viện `termios.h` và `fcntl.h` của Linux để chuyển STDIN sang chế độ non-blocking (nhận phím tức thời mà không cần bấm Enter).
  * `char read_key()`: Đọc phím bấm. Giải mã chuỗi escape sequence của các phím mũi tên (`ESC [ A/B/C/D`) thành các mã ký tự `U` (Lên), `N` (Xuống), `R` (Phải), `L` (Trái).
  * `enum class FlightState`: Máy trạng thái quản lý các chế độ bay:
    * `FLYING`: Đang bay và nhận lệnh lái từ phi công.
    * `LANDING`: Đang thực hiện chu trình hạ cánh an toàn.
    * `LANDED`: Đã tiếp đất an toàn, động cơ tắt hoàn toàn, chờ lệnh tiếp theo.
    * `TAKEOFF`: Đang tự động cất cánh leo lên $2.0\text{ m}$ để tiếp tục phiên bay.
  * **Cơ chế Mỏ neo ảo (Virtual Anchor & Position Hold):**
    * Khi người dùng nhả phím mũi tên quá $120\text{ms}$:
    * Giai đoạn 1: Drone tự động nghiêng ngược chiều vận tốc để phanh gấp ($V_x \to 0, V_y \to 0$).
    * Giai đoạn 2: Khi vận tốc đã triệt tiêu ($<0.08\text{ m/s}$), tự động thả mỏ neo tại vị trí $(X_{anchor}, Y_{anchor})$. Nếu có gió hay quán tính đẩy lệch, drone tự động nghiêng kéo ngược về tâm!
  * **Cơ chế Phím A & Phím Q:**
    * Nhấn **`A`**: Chuyển sang `LANDING`, khóa trục thẳng đứng, giảm độ cao êm ái $0.75\text{ m/s}$, chạm đất tự Disarm ngắt động cơ và chuyển sang `LANDED` (chương trình vẫn chạy).
    * Nhấn **`Q`**: Khi đang ở `LANDED`, tự động Arm động cơ lại, cất cánh thẳng đứng lên $2.0\text{ m}$, sau đó chuyển lại sang `FLYING` để người dùng tiếp tục lái.

#### 3. File `experiments/05_yaw_rotation_control.cpp`
* **Tên bài:** Thí nghiệm Xoay tròn quanh trục đứng Yaw (Spin Control).
* **Mục tiêu:** Xoay tròn 360° hoặc 720° quanh trục thẳng đứng Z mà không bị trôi tâm tọa độ, dừng lại đúng góc chỉ định mà không bị vọt lố.
* **Các hàm:**
  * `compute_position_hold(corr_roll, corr_pitch)`: Giữ drone đứng yên tuyệt đối tại tâm $(X_0, Y_0)$ trong suốt quá trình quay tròn.
  * `main(argc, argv)`:
    * Nhận tham số góc xoay từ dòng lệnh (mặc định $360^\circ$). Hỗ trợ xoay âm (quay phải), xoay nhiều vòng ($720^\circ$).
    * **Thuật toán phanh hãm pha trước (Phase-Lead Braking):** Khi còn cách góc đích $25^\circ$, tốc độ xoay mục tiêu sẽ được giảm tốc tuyến tính kết hợp với hệ số giảm chấn $K_{d,yaw} = 0.15$ để triệt tiêu đà quán tính xoay, khóa cứng đầu drone đúng góc đặt.

---

### D. THƯ MỤC `simulation/` (MÔ HÌNH & THẾ GIỚI MÔ PHỎNG)

#### 1. Thư mục `simulation/models/x500/`
* Chứa file mô hình SDF của quadrotor x500.
* Tích hợp 4 plugin động cơ `gz-sim-multicopter-motor-model-system` tương ứng với 4 khớp cánh quạt (`rotor_0_joint` đến `rotor_3_joint`), nhận lệnh từ topic `/x500/command/motor_speed`.
* Tích hợp plugin phát dữ liệu hành trình `gz-sim-odometry-publisher-system` phát dữ liệu tư thế và vận tốc thời gian thực qua topic `/model/x500/odometry` ở tần số **100 Hz**.

#### 2. Thư mục `simulation/models/x500_base/`
* Chứa toàn bộ hình học 3D, file lưới Collada (`.dae`), khối lượng chuẩn ($2.06\text{ kg}$) và tensor ma trận quán tính xoay ($I_{xx}, I_{yy}, I_{zz}$) của khung drone x500.

#### 3. File `simulation/worlds/drone_world.sdf`
* Định nghĩa môi trường thế giới vật lý của Gazebo Sim:
  * Trọng lực: gia tốc $g = -9.8\text{ m/s}^2$ theo phương thẳng đứng.
  * Động cơ vật lý ODE được cấu hình bước nhảy `max_step_size = 0.004s` với `real_time_update_rate = 250Hz` (đảm bảo độ chính xác vi phân cực cao).
  * Mặt phẳng mặt đất tĩnh có tính chất va chạm và đàn hồi.

---

### E. THƯ MỤC `log/` (DỮ LIỆU VÀ CÔNG CỤ VẼ ĐỒ THỊ)

#### 1. File `log/latest_flight.csv`
* File dữ liệu dạng bảng chứa toàn bộ thông số chi tiết của chuyến bay gần nhất: thời gian, độ cao đặt/thực tế, góc nghiêng Roll/Pitch/Yaw đặt/thực tế, mô-men điều khiển và tốc độ quay của từng cánh quạt.

#### 2. File `log/plot_flight.py`
* **Chức năng:** Công cụ vẽ đồ thị phân tích tự động viết bằng Python (dùng thư viện `pandas` và `matplotlib`).
* **Hàm `plot_flight(csv_path)`:**
  * Đọc file CSV và tự động xuất ra file ảnh `latest_flight_plot.png` chia làm 3 biểu đồ trực quan:
    1. **Biểu đồ 1:** Độ cao thực tế vs vận tốc rơi thẳng đứng $V_z$.
    2. **Biểu đồ 2:** Độ bám góc nghiêng Roll & Pitch (Mục tiêu nét đứt vs Thực tế nét liền).
    3. **Biểu đồ 3:** Tốc độ quay của 4 cánh quạt $\omega_0, \omega_1, \omega_2, \omega_3$ (rad/s).

---

## 4. BẢNG TẦN SỐ CÁC VÒNG LẶP TRONG HỆ THỐNG

| Thành phần | Vị trí mã nguồn | Chu kỳ | Tần số | Nhiệm vụ chính |
| :--- | :--- | :--- | :--- | :--- |
| **Vòng lặp trong (Inner Loop)** | `VehicleCommander::control_worker()` | $5\text{ ms}$ | **~200 Hz** | Tính PID tư thế góc, tốc độ góc vi phân, trộn động cơ chữ X, bơm lệnh motor sang Gazebo |
| **Vòng lặp ngoài (Outer Loop)** | `02_pid_altitude_hold.cpp`<br>`04_keyboard_teleop.cpp`<br>`05_yaw_rotation_control.cpp` | $30 - 33\text{ ms}$ | **~30 – 33 Hz** | Nhận phím điều khiển, tính PID độ cao, khóa mỏ neo vị trí ảo $X-Y$, in HUD ra Terminal |
| **Ghi Blackbox Logger** | `VehicleCommander::control_worker()` | $20\text{ ms}$ | **50 Hz** | Ghi các mẫu dữ liệu bay chi tiết ra file CSV |
| **Cảm biến Odometry** | `simulation/models/x500/model.sdf` | $10\text{ ms}$ | **100 Hz** | Gazebo xuất dữ liệu vị trí, vận tốc, góc Euler sang topic `/model/x500/odometry` |
| **Giải tích vật lý ODE** | `simulation/worlds/drone_world.sdf` | $4\text{ ms}$ | **250 Hz** | Gazebo mô phỏng tương tác lực nâng, quán tính và va chạm |

---
*Tài liệu này được biên soạn đầy đủ, chính xác và đồng bộ 100% với mã nguồn hiện hành của dự án `drone_control_cpp`.*
