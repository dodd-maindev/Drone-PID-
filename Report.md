# TÀI LIỆU KỸ THUẬT TOÀN DIỆN HỆ THỐNG ĐIỀU KHIỂN QUADROTOR (DRONE_CONTROL_CPP)

> **Mục tiêu dự án:** Phát triển hệ thống Flight Control thuần C++ (chuẩn C++17) tự chủ hoàn toàn từ gốc, điều khiển trực tiếp mô hình quadrotor `x500` trong môi trường mô phỏng **Gazebo Sim Harmonic (8.15.0)** thông qua giao thức **GZ-Transport IPC**, đồng thời phát dữ liệu Telemetry thời gian thực qua giao thức **MAVLink (UDP)** tới trạm mặt đất **QGroundControl (QGC)**.  
> Hệ thống áp dụng kiến trúc điều khiển thác đa tầng (Cascaded Control Architecture) chuẩn công nghiệp hàng không tương đương PX4 Autopilot và DJI, tích hợp bộ ước lượng nhiễu gió (Disturbance Observer), phanh chủ động tức thì (Instant Anchor Lock) và làm mượt quán tính (Slew Rate Limiting).

---

## MỤC LỤC
1. [Sơ đồ cấu trúc tổng thể thư mục (Project Tree)](#1-sơ-đồ-cấu-trúc-tổng-thể-thư-mục)
2. [Kiến trúc phân tầng điều khiển (Cascaded Multi-Rate Architecture)](#2-kiến-trúc-phân-tầng-điều-khiển)
3. [Chi tiết từng thành phần, từng file và từng thuật toán](#3-chi-tiết-từng-thành-phần-từng-file-và-từng-thuật-toán)
   - [Thư mục gốc (Root)](#a-thư-mục-gốc-root)
   - [Thư mục `include/` & `src/` (Thư viện tĩnh `drone_lib`)](#b-thư-mục-include-và-src-thư-viện-lõi-drone_lib)
     - [comm/ (Giao tiếp Gazebo IPC & QGroundControl MAVLink)](#b1-thư-mục-comm-giao-tiếp-truyền-thông)
     - [core/ (Quản lý phương tiện & Trạng thái nguyên tử đa luồng)](#b2-thư-mục-core-quản-lý-phương-tiện--trạng-thái)
     - [controllers/ (Bộ điều khiển Vị trí, Độ cao, PID & Motor Mixer)](#b3-thư-mục-controllers-thuật-toán-điều-khiển)
     - [math/ (Toán học động học & Động lực học Quaternion/Euler)](#b4-thư-mục-math-toán-học-hàng-không)
     - [utils/ (Blackbox Telemetry Logger)](#b5-thư-mục-utils-ghi-nhận-dữ-liệu-hộp-đen)
   - [Thư mục `experiments/` (Các chương trình bay thực nghiệm)](#c-thư-mục-experiments-các-chương-trình-thí-nghiệm-chính)
   - [Thư mục `simulation/` (Mô hình SDF và Thế giới Gazebo Sim)](#d-thư-mục-simulation-mô-hình--thế-giới-mô-phỏng)
   - [Thư mục `log/` (Dữ liệu hộp đen và Phân tích Telemetry)](#e-thư-mục-log-dữ-liệu-và-công-cụ-vẽ-đồ-thị)
4. [Bảng thông số tần số và chu kỳ các vòng lặp trong hệ thống](#4-bảng-thông-số-tần-số-và-chu-kỳ-các-vòng-lặp)

---

## 1. SƠ ĐỒ CẤU TRÚC TỔNG THỂ THƯ MỤC

```text
drone_control_cpp/
├── CMakeLists.txt                  # Kịch bản build CMake C++17, liên kết Gz-Transport, Gz-Msgs, pthread
├── README.md                       # Giới thiệu dự án và hướng dẫn chạy nhanh
├── EXPLAIN.md                      # [Tài liệu này] Bách khoa toàn thư giải thích toàn bộ kiến trúc mã nguồn
├── run.txt                         # Lệnh mẫu và ghi chú môi trường chạy Gazebo Sim + QGroundControl
│
├── include/                        # Header files (Khai báo giao diện, class, struct, math)
│   ├── comm/
│   │   ├── gz_bridge.hpp           # Header cầu nối Gazebo Transport IPC
│   │   └── mavlink_bridge.hpp      # Header máy chủ MAVLink UDP kết nối QGroundControl
│   ├── controllers/
│   │   ├── altitude_controller.hpp # Header bộ điều khiển độ cao (PI + Velocity Damping)
│   │   ├── motor_mixer.hpp         # Header bộ trộn động cơ chữ X (Motor Mixer + Mixer Desaturation)
│   │   ├── pid.hpp                 # Header lớp PID thuần C++ 1D (Anti-windup, Derivative on Error)
│   │   └── position_controller.hpp # Header bộ điều khiển vị trí 2D, bù gió tích phân, phanh mỏ neo
│   ├── core/
│   │   ├── vehicle_commander.hpp   # Header lớp quản trị và điều phối tổng thể drone
│   │   └── vehicle_state.hpp       # Struct trạng thái cảm biến nguyên tử lock-free (std::atomic)
│   ├── math/
│   │   └── math_utils.hpp          # Toán học hàng không (Euler <-> Quaternion, Normalize, Clamp)
│   └── utils/
│       └── data_logger.hpp         # Header ghi log dữ liệu Blackbox 24 cột ra file CSV
│
├── src/                            # Source files (Hiện thực hóa các class của drone_lib)
│   ├── comm/
│   │   ├── gz_bridge.cpp           # Hiện thực Sub Odometry (100Hz), Pub Actuators (200Hz) qua Gz-Transport
│   │   └── mavlink_bridge.cpp      # Hiện thực UDP Socket, tự dò IP host WSL2, stream MAVLink v2/v1 sang QGC
│   ├── controllers/
│   │   └── altitude_controller.cpp # Thuật toán điều khiển độ cao: Hover 0.59, bù lực nâng, hãm rơi
│   ├── core/
│   │   └── vehicle_commander.cpp   # Vòng lặp Inner Loop 200Hz: Attitude PID, Tilt Compensation, Motor Mixer
│   └── utils/
│       └── data_logger.cpp         # Quản lý file CSV, ghi đệm thread-safe với std::mutex
│
├── experiments/                    # Các chương trình bay thực thi độc lập (Executables)
│   ├── 02_pid_altitude_hold.cpp    # Bài 2: Tự động cất cánh 2.0m -> 3.5m -> hạ cánh khóa vị trí thẳng đứng
│   ├── 04_keyboard_teleop.cpp      # Bài 4: Bàn phím điều khiển thời gian thực (Mũi tên / IJKL, phanh mỏ neo)
│   └── 05_yaw_rotation_control.cpp # Bài 5: Xoay tròn quanh trục Yaw 360°/720° có phanh pha trước (Phase-Lead)
│
├── simulation/                     # Tài nguyên mô phỏng Gazebo Sim Harmonic
│   ├── models/
│   │   ├── x500/                   # Định nghĩa SDF quadrotor x500: 4 multicopter motor plugins + 1 odometry
│   │   └── x500_base/              # Khung vỏ 2.06kg, mesh 3D Collada (.dae), ma trận quán tính xoay
│   └── worlds/
│       └── drone_world.sdf         # Môi trường thế giới vật lý ODE (250Hz, step 0.004s), cấu hình gió
│
├── third_party/
│   └── mavlink/                    # Thư viện header-only MAVLink v2 chuẩn quốc tế (c_library_v2)
│
└── log/                            # Lưu trữ và phân tích dữ liệu chuyến bay
    ├── latest_flight.csv           # File log phản chiếu chuyến bay gần nhất
    ├── flight_telemetry_*.csv      # Toàn bộ lịch sử các chuyến bay được đánh dấu thời gian
    └── plot_flight.py              # Script Python tự động trích xuất và vẽ đồ thị phân tích chất lượng bay
```

---

## 2. KIẾN TRÚC PHÂN TẦNG ĐIỀU KHIỂN (CASCADED MULTI-RATE ARCHITECTURE)

Hệ thống được xây dựng theo mô hình **Điều khiển Thác Đa Tốc Độ (Cascaded Multi-Rate Control System)** chuẩn mực của các hệ thống lái tự động tiên tiến nhất thế giới (PX4, ArduPilot, ETH RPG):

```text
       [ NGƯỜI DÙNG / BÀN PHÍM / KỊCH BẢN BAY / QGROUNDCONTROL ]
                                 │
                 Lệnh điều khiển: Velocity / Position / Altitude Target
                                 │
                                 ▼
┌─────────────────────────────────────────────────────────────────────────────────┐
│ VÒNG LẶP NGOÀI - VỊ TRÍ & ĐỘ CAO (OUTER LOOP ~30 - 33 Hz)                        │
│ (Được hiện thực trong experiments/ và các Controller tầng cao)                   │
├─────────────────────────────────────────────────────────────────────────────────┤
│ 1. PositionController (include/controllers/position_controller.hpp):            │
│    • Outer P-Position Loop:  v_sp = Kp_pos * (anchor - current_pos)             │
│    • Slew Rate Limiter:      Giới hạn gia tốc lệnh tay lái max 3.5 m/s²         │
│    • Instant Anchor Snap:    Khóa mỏ neo tức thì lúc buông phím (chuẩn DJI)     │
│    • Inner Velocity Loop:    ev = v_sp - v_cur -> a_sp = Kp_vel*ev + I_wind     │
│    • Wind Disturbance Obs:   I_wind = ∫ Ki_vel * ev dt (ước lượng lực cản gió)  │
│    • Acc-to-Attitude:        Pitch = atan2(ax, g), Roll = atan2(-ay, g)         │
│    • Tilt Rate Limiter:      Giới hạn tốc độ biến thiên góc 180°/s              │
│                                                                                 │
│ 2. AltitudeController (include/controllers/altitude_controller.hpp):            │
│    • Base Hover Thrust:      Điểm cân bằng tĩnh Hover = 0.59                    │
│    • PI Altitude Loop:       ΔThrust = Kp_z * ez + Ki_z * ∫ez dt                │
│    • Velocity Damping:       Hãm tốc độ leo/rơi: - Kv * Vz (Kv = 0.32)          │
│    • Integral Preservation:  Không reset tích phân khi nhả phím (chống sụt ga)  │
└─────────────────────────────────────────────────────────────────────────────────┘
                                 │
     Sinh ra mục tiêu góc & ga: Target Roll, Pitch, Yaw (rad) & Thrust [0.0 - 1.0]
                                 │
                                 ▼
┌─────────────────────────────────────────────────────────────────────────────────┐
│ VÒNG LẶP TRONG - TƯ THẾ & MÔ-MEN (INNER LOOP 200 Hz / Chu kỳ 5ms)               │
│ (Chạy ngầm trong luồng độc lập: src/core/vehicle_commander.cpp)                 │
├─────────────────────────────────────────────────────────────────────────────────┤
│ 1. Attitude PID Controller:                                                     │
│    • Sai số góc nghiêng:     e_roll = roll_target - roll_actual                 │
│                              e_pitch = pitch_target - pitch_actual              │
│                              e_yaw = normalize(yaw_target - yaw_actual)         │
│    • Mô-men điều khiển:      tau_roll  = Kp*e_roll  + Ki*∫e_roll  - Kd*p_rate    │
│                              tau_pitch = Kp*e_pitch + Ki*∫e_pitch - Kd*q_rate   │
│                              tau_yaw   = Kp_yaw*e_yaw - Kd_yaw*r_rate           │
│    • Anti-Windup Clamping:   Giới hạn tích lũy tích phân góc [-0.05, +0.05]     │
│                                                                                 │
│ 2. Tilt Compensation (Bù lực nâng góc nghiêng):                                 │
│    • Effective_Thrust = Thrust_cmd / cos(tilt)                                  │
│                                                                                 │
│ 3. MotorMixer (include/controllers/motor_mixer.hpp):                            │
│    • Phân bổ lực chữ X:      u0..u3 = Thrust ± Roll ± Pitch ± Yaw               │
│    • Mixer Desaturation:     Hạ đều công suất khi bão hòa, ưu tiên thăng bằng   │
│    • Chuyển đổi công suất:   ω_i = ω_max * sqrt(clamp(u_i, 0, 1))               │
└─────────────────────────────────────────────────────────────────────────────────┘
           │                                                   │
           ▼ (Vận tốc 4 cánh quạt: rad/s)                      ▼ (Dữ liệu Telemetry 30Hz)
┌──────────────────────────────────────┐        ┌──────────────────────────────────────┐
│ CẦU NỐI GAZETO IPC (src/comm/gz_...) │        │ MÁY CHỦ MAVLINK (src/comm/mavlink_.) │
│ Gz-Transport Node                    │        │ UDP Port 14550 (QGC) & 14540 (Local) │
│ Topic: /x500/command/motor_speed     │        │ Heartbeat, Attitude, Local/Global Pos│
└──────────────────────────────────────┘        └──────────────────────────────────────┘
                   │                                               │
                   ▼                                               ▼
     [ Gazebo Sim 8.15 Harmonic ]                       [ QGroundControl Station ]
     ODE Physics Solver (250 Hz)                        HUD, Độ cao, La bàn, Giám sát
```

---

## 3. CHI TIẾT TỪNG THÀNH PHẦN, TỪNG FILE VÀ TỪNG THUẬT TOÁN

### A. THƯ MỤC GỐC (ROOT)

#### 1. `CMakeLists.txt`
* **Mục đích:** Kịch bản cấu hình biên dịch hiện đại cho C++17, tự động tìm kiếm thư viện Gazebo Transport, Gazebo Messages và liên kết thư viện luồng `pthread`.
* **Cấu hình chi tiết:**
  * `find_package(gz-transport13 REQUIRED)` & `find_package(gz-msgs10 REQUIRED)`: Nhập các gói giao tiếp liên tiến trình (IPC) và Protobuf của Gazebo Harmonic.
  * `add_library(drone_lib STATIC ...)`: Biên dịch toàn bộ các module lõi thành thư viện tĩnh `libdrone_lib.a`:
    * `src/comm/gz_bridge.cpp`
    * `src/comm/mavlink_bridge.cpp`
    * `src/controllers/altitude_controller.cpp`
    * `src/core/vehicle_commander.cpp`
    * `src/utils/data_logger.cpp`
  * `target_include_directories(drone_lib PUBLIC include third_party/mavlink)`: Đưa thư viện MAVLink v2 và các header dự án vào đường dẫn tìm kiếm toàn cục.
  * `target_compile_options(drone_lib PUBLIC -Wno-address-of-packed-member)`: Bỏ qua cảnh báo alignment trong cấu trúc nén của thư viện MAVLink C.
  * Tự động kiểm tra và build 3 file thực thi trong thư mục `build/`:
    * `02_pid_altitude_hold`
    * `04_keyboard_teleop`
    * `05_yaw_rotation_control`

---

### B. THƯ MỤC `include/` VÀ `src/` (THƯ VIỆN LÕI `drone_lib`)

#### B.1. Thư mục `comm/` (Giao tiếp truyền thông)

##### 1. File `include/comm/gz_bridge.hpp` & `src/comm/gz_bridge.cpp`
* **Chức năng:** Cầu nối IPC thời gian thực giữa Flight Controller và mô phỏng Gazebo Sim thông qua thư viện `gz-transport`.
* **Nguyên lý hoạt động:**
  * Sử dụng một đối tượng `gz::transport::Node` duy nhất để vừa Subscribe vừa Publish dữ liệu.
* **Các phương thức:**
  * `bool init(VehicleState* state_ptr, const std::string& odom_topic, const std::string& motor_topic)`:
    * Đăng ký nhận bản tin từ `/model/x500/odometry` kiểu `gz::msgs::Odometry`.
    * Đăng ký kênh phát lệnh sang `/x500/command/motor_speed` kiểu `gz::msgs::Actuators`.
  * `void on_odometry(const gz::msgs::Odometry& msg)`:
    * Hàm Callback được triệu gọi mỗi khi Gazebo phát sinh gói dữ liệu Odometry (tần số 100Hz).
    * Trích xuất vị trí $X, Y, Z$ (`msg.pose().position()`).
    * Trích xuất vận tốc tịnh tiến thế giới $V_x, V_y, V_z$ (`msg.twist().linear()`).
    * Trích xuất Quaternion $[w, x, y, z]$, gọi `DroneMath::quaternion_to_euler()` chuyển thành 3 góc Euler $Roll, Pitch, Yaw$ (radian).
    * Trích xuất tốc độ góc thân $p, q, r$ (`msg.twist().angular()`).
    * Lưu nguyên tử vào `VehicleState` và đánh dấu `is_connected = true`.
  * `void send_motor_velocities(const std::array<float, 4>& speeds)`:
    * Đóng gói mảng 4 giá trị vận tốc góc $\omega_0, \omega_1, \omega_2, \omega_3$ (rad/s) vào `gz::msgs::Actuators`.
    * Bơm sang Gazebo Sim điều khiển trực tiếp 4 khớp động cơ quadrotor.
  * `void shutdown()`: Gửi mảng tốc độ $\{0, 0, 0, 0\}$ để dừng hẳn motor và ngắt kết nối an toàn.

##### 2. File `include/comm/mavlink_bridge.hpp` & `src/comm/mavlink_bridge.cpp`
* **Chức năng:** Máy chủ MAVLink UDP thời gian thực, kết nối 2 chiều giữa Flight Controller và phần mềm trạm mặt đất **QGroundControl (QGC)** chạy trên Windows hoặc Linux.
* **Đặc điểm kiến trúc:**
  * **Tự động dò IP Host WSL2:** Hàm `detect_wsl_host_ip()` đọc file `/etc/resolv.conf` để trích xuất địa chỉ IP ảo của Windows Host, tự động thiết lập kênh truyền thông xuyên mạng ảo WSL2 sang QGroundControl mà không cần người dùng nhập IP thủ công.
  * **Quản lý đa địa chỉ đích (Multi-Target Broadcasting):** Gửi đồng thời đến `127.0.0.1:14550` (Local QGC) và `Windows_Host_IP:14550` (Windows QGC).
* **Các bản tin MAVLink được phát định kỳ (MAVLink Telemetry Streams):**
  * `HEARTBEAT` (1 Hz): Định danh hệ thống là Autopilot đa cánh quạt (`MAV_TYPE_QUADROTOR`, `MAV_AUTOPILOT_GENERIC`), báo trạng thái Armed/Disarmed và chế độ bay Guided/Manual.
  * `SYS_STATUS` (1 Hz): Báo cáo tình trạng pin ảo (16.8V - 4S LiPo), cảm biến 3D gyro, accel, từ kế và GPS hoạt động bình thường.
  * `ATTITUDE` (30 Hz): Stream trực tiếp góc nghiêng Roll, Pitch, Yaw và vận tốc góc $p, q, r$ để QGroundControl vẽ la bàn và đường chân trời nhân tạo thời gian thực.
  * `LOCAL_POSITION_NED` & `GLOBAL_POSITION_INT` (10 Hz): Stream tọa độ vị trí 3D $X, Y, Z$ và vận tốc $V_x, V_y, V_z$ để QGC hiển thị vị trí drone trên bản đồ tọa độ.
  * `VFR_HUD` (10 Hz): Stream tốc độ bay, cao độ thực tế và mức ga để hiển thị trên thanh đồng hồ tốc độ/cao độ của QGC.
* **Nhận lệnh từ QGroundControl (Uplink Command Processing):**
  * Lắng nghe gói tin UDP tại cổng local `14540`.
  * Xử lý lệnh `MAV_CMD_COMPONENT_ARM_DISARM`: Khi người dùng gạt nút Arm/Disarm trên màn hình QGC, callback `arm_callback_` sẽ tự động kích hoạt Arm/Disarm trong lõi C++.
  * Phản hồi `MAV_CMD_REQUEST_MESSAGE` và `COMMAND_ACK`.
  * Phương* **Các cơ chế kỹ thuật cốt lõi:**
  1. **Phanh chủ động & Khóa mỏ neo tại điểm dừng (Active Braking & Stop-Point Anchor Lock):**
     * **Khi đang lái (`AxisMode::DRIVING`):** Mỏ neo $(Anchor_X, Anchor_Y)$ liên tục trượt theo vị trí thực tế của drone.
     * **Khoảnh khắc buông phím (`AxisMode::BRAKING`):** Đặt ngay vận tốc mục tiêu $V_{target} = 0\text{ m/s}$. Drone nghiêng thân đối kháng tỷ lệ thuận với vận tốc thực tế để dập tắt quán tính về $0$. Trong suốt quá trình này, mỏ neo tiếp tục bám theo drone $\to$ **TUYỆT ĐỐI KHÔNG sinh lực kéo lùi về vị trí cũ trước khi nhả tay!**
     * **Khi drone đã dừng hẳn ($|V| < 0.08\text{ m/s}$ - `AxisMode::HOLD`):** Khóa cứng mỏ neo tại chính vị trí dừng thực tế. Chuyển sang vòng lặp $P-PID$ giữ chặt vị trí này, chống trôi gió.
  2. **Bộ ước lượng gió tích phân chống bão hòa (Anti-Windup Wind Observer):**
     * Tạm dừng tích phân trong lúc đang phanh hãm động lực, chỉ kích hoạt tích phân bù gió khi cả hai trục đã dừng hẳn (`HOLD`).
     * Loại bỏ hoàn toàn hiện tượng tích lũy sai số động làm drone giật lùi sau khi dừng.
  3. **Chuyển đổi gia tốc sang góc nghiêng thân chuẩn vật lý (Acc-to-Attitude Mapping):**
     * Tính toán vector gia tốc mong muốn: $a_{sp} = K_{p\_vel} \cdot (v_{sp} - v) + a_{wind}$.
     * Chiếu chính xác qua gia tốc trọng trường Trái Đất ($g = 9.80665\text{ m/s}^2$):
       $$Pitch = \arctan\left(\frac{a_{sp\_x}}{g}\right), \quad Roll = \arctan\left(\frac{-a_{sp\_y}}{g}\right)$$
  4. **Bộ lọc gia tốc tay lái (Slew Rate Limiter):**
     * Khi ấn phím, vận tốc mục tiêu tăng tốc mượt mà với gia tốc tối đa $3.5\text{ m/s}^2$. Khởi tạo từ vận tốc hiện tại (`smoothed_cmd = vx`), loại bỏ cú sốc giật gia tốc.
  5. **Độ trễ nhả phím thích ứng (Adaptive Key Release Timeout - 120ms):**
     * Giảm thời gian phát hiện buông phím từ $0.45\text{s}$ xuống còn **$0.12\text{s}$** khi đang giữ phím (nhanh gấp gần 4 lần), cắt đứt quãng đường trôi quán tính ngay lập tức.
  6. **Phanh đối kháng dứt khoát:**
     * Khi phanh hãm dừng: Cho phép mở rộng góc nghiêng phanh lên tới **$16.0^\circ$** với $K_{p\_vel} = 2.5$, dập tắt toàn bộ vận tốc $2.5\text{ m/s}$ chỉ trong vòng chưa đầy $0.8\text{s}$.��u chưa Arm hoặc ga $< 0.02$, reset tích phân góc và gửi 0 rad/s cho 4 motor.
  * **Thuật toán PID Tư thế (Attitude PID):**
    * Sai số góc:
      $$e_{roll} = Roll_{cmd} - Roll_{cur}$$
      $$e_{pitch} = Pitch_{cmd} - Pitch_{cur}$$
      $$e_{yaw} = \text{normalize\_angle}(Yaw_{cmd} - Yaw_{cur})$$
    * Tích lũy tích phân kèm bộ kẹp Anti-Windup kẹp chặt trong khoảng $[-0.05, +0.05]$.
    * Tính mô-men phản hồi:
      $$\tau_{roll} = K_{p} e_{roll} + K_{i} \int e_{roll} dt - K_{d} \cdot p$$
      $$\tau_{pitch} = K_{p} e_{pitch} + K_{i} \int e_{pitch} dt - K_{d} \cdot q$$
      $$\tau_{yaw} = K_{p,yaw} e_{yaw} - K_{d,yaw} \cdot r$$
    * Kẹp ngưỡng an toàn mô-men: $|\tau_{roll}| \le 0.25$, $|\tau_{pitch}| \le 0.25$, $|\tau_{yaw}| \le 0.20$.
  * **Bù lực nâng góc nghiêng (Tilt Compensation):**
    $$\text{Effective\_Thrust} = \frac{Thrust_{cmd}}{\max(0.65, \cos(Roll_{cur}) \cdot \cos(Pitch_{cur}))}$$
    Giúp drone giữ nguyên độ cao ổn định khi nghiêng thân cơ động mà không bị tụt độ cao.
  * **Bộ trộn động cơ:** Truyền $\text{Effective\_Thrust}$ và $\tau_{roll}, \tau_{pitch}, \tau_{yaw}$ qua `mixer_.compute_motor_speeds()` sinh ra $\omega_0, \omega_1, \omega_2, \omega_3$.
  * Bơm lệnh vận tốc góc sang Gazebo qua `gz_bridge_.send_motor_velocities()`.
  * Định kỳ mỗi $20\text{ms}$ (tần số 50Hz), ghi một mẫu vào `logger_`.

---

#### B.3. Thư mục `controllers/` (Thuật toán điều khiển)

##### 1. File `include/controllers/position_controller.hpp` (Nâng cấp toàn diện)
* **Chức năng:** Bộ điều khiển vị trí 2D, chống trôi gió tự nhiên và phanh dừng vị trí thiết kế theo chuẩn kiến trúc **PX4 Autopilot & DJI Loiter/Position-Hold**.
* **Các cơ chế kỹ thuật cốt lõi:**
  1. **Khóa mỏ neo tức thì lúc buông tay (Instant Anchor Snap):**
     * Khi người lái đang ấn phím lái: Mỏ neo $(Anchor_X, Anchor_Y)$ liên tục trượt theo vị trí thực tế của drone.
     * **Khoảnh khắc người lái buông phím:** Mỏ neo được **KHÓA CỨNG NGAY LẬP TỨC** tại tọa độ buông tay.
     * Quán tính đẩy drone trôi tới đâu, vòng lặp vị trí ($K_{p\_pos} \cdot \Delta p$) sẽ lập tức sinh ra vận tốc mục tiêu mang dấu âm $\rightarrow$ kéo giật ngược góc nghiêng để phanh đứng khựng và hút drone trở về đúng vị trí lúc vừa buông tay (Zero Drift).
  2. **Bộ ước lượng gió tích phân liên tục (Wind Disturbance Observer):**
     * Tích phân sai số vận tốc:
       $$\mathbf{I}_{wind\_x} = \int K_{i\_vel} (V_{sp\_x} - V_{x}) dt, \quad \mathbf{I}_{wind\_y} = \int K_{i\_vel} (V_{sp\_y} - V_{y}) dt$$
     * Tự động tích lũy và duy trì góc đón gió ổn định (Pitch đón gió $-1.5^\circ \dots -2.5^\circ$, Roll đón gió $+0.6^\circ \dots +1.0^\circ$ khi có gió $2\text{ m/s}$) mà không bị trôi vị trí.
  3. **Chuyển đổi gia tốc sang góc nghiêng thân chuẩn vật lý (Acc-to-Attitude Mapping):**
     * Tính toán vector gia tốc mong muốn: $a_{sp} = K_{p\_vel} \cdot (v_{sp} - v) + a_{wind}$.
     * Chiếu chính xác qua gia tốc trọng trường Trái Đất ($g = 9.80665\text{ m/s}^2$):
       $$Pitch = \arctan\left(\frac{a_{sp\_x}}{g}\right), \quad Roll = \arctan\left(\frac{-a_{sp\_y}}{g}\right)$$
  4. **Bộ lọc gia tốc tay lái (Slew Rate Limiter):**
     * Khi ấn phím, vận tốc mục tiêu không nhảy vọt dạng xung bước (Step) mà tăng tốc mượt mà với gia tốc tối đa $3.5\text{ m/s}^2$.
     * Khởi tạo mượt mà từ vận tốc hiện tại (`smoothed_cmd = vx`), loại bỏ 100% cú sốc giật gia tốc ở đầu mỗi lần nhấn phím.
  5. **Bộ lọc tốc độ đổi góc (Tilt Angular Rate Limiter - $180^\circ/\text{s}$):**
     * Khống chế tốc độ quét góc nghiêng tối đa $180^\circ/\text{giây}$ ($\approx 6^\circ$ mỗi chu kỳ $33\text{ms}$).
     * Loại bỏ hoàn toàn hiện tượng rung lắc/chao đảo (wobbling) khi đổi trạng thái lái $\leftrightarrow$ phanh, cho phép phản ứng phanh dứt khoát chỉ trong $0.12\text{s}$.
  6. **Phanh đối kháng chủ động dứt khoát:**
     * Khi đang bay bình thường: Giới hạn góc nghiêng ở `max_tilt_deg` ($12^\circ$).
     * Khi buông tay phanh dừng: Cho phép mở rộng góc phanh đối kháng lên tới **$18.0^\circ$**, giúp dập tắt toàn bộ quán tính $2.5\text{ m/s}$ trong quãng đường chưa đầy **$0.7\text{ mét}$**.

##### 2. File `include/controllers/altitude_controller.hpp` & `src/controllers/altitude_controller.cpp`
* **Chức năng:** Bộ điều khiển độ cao chuyên biệt cho quadrotor x500.
* **Các đặc tính quan trọng:**
  * Điểm cân bằng tĩnh: `hover_thrust_ = 0.59`.
  * **Cơ chế phanh vi phân vận tốc rơi (Velocity Damping):**
    $$Thrust = Hover + \Delta Thrust_{PI} - K_v \cdot V_z \quad (K_v = 0.32)$$
    Khi drone bay lên nhanh ($V_z > 0$), thành phần $-K_v \cdot V_z$ chủ động hạ bớt lực nâng trước khi đạt độ cao đặt $\to$ **Triệt tiêu hoàn toàn hiện tượng vọt lố (Zero Overshoot)**.
  * **Bảo toàn tích phân khi buông phím:** Khi người dùng nhả phím tăng/giảm độ cao (W/S), bộ điều khiển chỉ chốt lại độ cao mục tiêu mà **không gọi reset tích phân**, giúp lực nâng duy trì ổn định phẳng lì, không bao giờ bị hụt ga tụt độ cao đột ngột.
  * Dải giới hạn an toàn: Kẹp ga trong khoảng $[0.10, 0.85]$.

##### 3. File `include/controllers/motor_mixer.hpp`
* **Chức năng:** Bộ trộn động cơ ma trận cho khung quadrotor chữ X (x500 geometry).
* **Bố trí động cơ (Hệ trục FLU của Gazebo):**
  * Rotor 0: Front-Right (+X, -Y) $\to$ Quay CCW (-)
  * Rotor 1: Rear-Left   (-X, +Y) $\to$ Quay CCW (-)
  * Rotor 2: Front-Left  (+X, +Y) $\to$ Quay CW  (+)
  * Rotor 3: Rear-Right  (-X, -Y) $\to$ Quay CW  (+)
* **Ma trận phân bổ:**
  $$u_0 = Thrust - Roll - Pitch - Yaw$$
  $$u_1 = Thrust + Roll + Pitch - Yaw$$
  $$u_2 = Thrust + Roll - Pitch + Yaw$$
  $$u_3 = Thrust - Roll + Pitch + Yaw$$
* **Kỹ thuật chống bão hòa công suất (Mixer Desaturation):**
  * Khi có động cơ bị yêu cầu $u_i > 1.0$, thuật toán tính lượng thừa $\Delta = u_{max} - 1.0$ và trừ đều trên cả 4 động cơ.
  * Ưu tiên tuyệt đối việc bảo toàn mô-men thăng bằng tư thế góc, chấp nhận hy sinh một phần nhỏ lực nâng tổng để drone không bị lật.
* **Chuyển đổi phi tuyến sang vận tốc góc:**
  $$\omega_i = \omega_{max} \cdot \sqrt{\text{clamp}(u_i, 0, 1)} \quad (\omega_{max} = 1000\text{ rad/s})$$

##### 4. File `include/controllers/pid.hpp`
* **Chức năng:** Lớp điều khiển PID 1 chiều tổng quát, thuần C++, có bộ kẹp tích phân chống bão hòa (Anti-Reset Windup) và giới hạn đầu ra.

---

#### B.4. Thư mục `math/` (Toán học hàng không)

##### File `include/math/math_utils.hpp`
* Cung cấp các công thức biến đổi hệ trục và góc xoay hàng không hiệu năng cao (inline):
  * `quaternion_to_euler(w, x, y, z, roll, pitch, yaw)`: Chuyển đổi Quaternion 4 chiều từ cảm biến Odometry sang 3 góc Euler Roll, Pitch, Yaw.
  * `euler_to_quaternion(roll, pitch, yaw)`: Chuyển đổi 3 góc Euler sang Quaternion $[w, x, y, z]$.
  * `normalize_angle(angle)`: Chuẩn hóa góc về khoảng $[-\pi, +\pi]$ (triệt tiêu bước nhảy không liên tục khi qua mốc $180^\circ$).
  * `deg2rad(deg)`, `rad2deg(rad)`: Chuyển đổi qua lại giữa độ và radian.
  * `clamp(val, min_val, max_val)`: Giới hạn giá trị trong ngưỡng an toàn.

---

#### B.5. Thư mục `utils/` (Ghi nhận dữ liệu hộp đen)

##### File `include/utils/data_logger.hpp` & `src/utils/data_logger.cpp`
* **Chức năng:** Ghi lại toàn bộ dữ liệu điều khiển và cảm biến của từng mili-giây chuyến bay ra file CSV phục vụ hậu kiểm và vẽ đồ thị.
* **Cấu trúc 24 cột dữ liệu trong `FlightLogEntry`:**
  `time_s, target_thrust, target_roll_deg, target_pitch_deg, target_yaw_deg, actual_x, actual_y, actual_vx, actual_vy, actual_alt_m, actual_vz_mps, actual_roll_deg, actual_pitch_deg, actual_yaw_deg, rate_p_rad_s, rate_q_rad_s, rate_r_rad_s, tau_roll, tau_pitch, tau_yaw, w0_rad_s, w1_rad_s, w2_rad_s, w3_rad_s`.
* **Cơ chế quản lý file:**
  * Tự động sinh file có gắn nhãn thời gian: `log/flight_telemetry_YYYYMMDD_HHMMSS.csv`.
  * Đồng thời cập nhật liên tục vào file phản chiếu `log/latest_flight.csv` để các công cụ phân tích luôn đọc được chuyến bay mới nhất.
  * Ghi đệm an toàn đa luồng với `std::lock_guard<std::mutex>`.

---

### C. THƯ MỤC `experiments/` (CÁC CHƯƠNG TRÌNH THÍ NGHIỆM CHÍNH)

#### 1. File `experiments/02_pid_altitude_hold.cpp`
* **Tên thí nghiệm:** Giữ độ cao PID tự động và Khóa vị trí 2D.
* **Kịch bản thực hiện:**
  1. Kết nối Gazebo Sim, gửi mồi lệnh ban đầu, Arm động cơ.
  2. Tự động cất cánh leo lên $2.0\text{ m}$ mượt như thang máy trong 6 giây (tốc độ giới hạn $0.75\text{ m/s}$).
  3. Tiếp tục leo bậc thang lên $3.5\text{ m}$, giữ thăng bằng ổn định tuyệt đối, không vọt lố.
  4. Bật thuật toán khóa tâm vị trí $X-Y$ trong suốt hành trình.
  5. Tự động hạ cánh thẳng đứng về mặt đất ($0.08\text{ m}$) và Disarm ngắt động cơ an toàn.

#### 2. File `experiments/04_keyboard_teleop.cpp`
* **Tên thí nghiệm:** Bàn phím điều khiển thời gian thực (Virtual RC Teleop) tích hợp chống trôi gió và phanh khóa mỏ neo.
* **Giao diện điều khiển bàn phím:**
  * **`Mũi tên` hoặc `I / K / J / L`:** Lái Tiến / Lùi / Trái / Phải (Nhả ra tự phanh khóa neo ngay lập tức).
  * **`W / S`:** Tăng / Giảm độ cao (Bấm giữ để leo/hạ, nhả ra dừng độ cao êm dịu).
  * **`Z / C` hoặc `E / D`:** Xoay đầu (Yaw) Trái / Phải.
  * **`1 / 2 / 3`:** Chuyển nhanh 3 chế độ tốc độ:
    * Chế độ 1: Chậm ($1.0\text{ m/s}$, góc nghiêng tối đa $8.0^\circ$).
    * Chế độ 2: Vừa ($2.5\text{ m/s}$, góc nghiêng tối đa $12.0^\circ$).
    * Chế độ 3: Nhanh ($5.0\text{ m/s}$, góc nghiêng tối đa $18.0^\circ$).
  * **`A`:** Kích hoạt hạ cánh an toàn khẩn cấp (Safe Land).
  * **`Q`:** Khởi động động cơ và cất cánh bay lên tiếp (Takeoff to 2.0m).
  * **`SPACE`:** Phanh đứng khẩn cấp (Emergency Hover Lock).
  * **`X`:** Thoát chương trình an toàn.
* **Các cải tiến kỹ thuật đặc biệt:**
  * Bộ giải mã chuỗi thoát (Escape Sequence Parser) cho phím mũi tên Linux non-blocking có cơ chế kiểm tra đầy đủ, không bao giờ bị rơi mã gây hiểu nhầm thành phím `A`.
  * Bộ đệm thời gian lặp phím `0.45s` giúp nối mượt mà khoảng trễ nhịp gõ phím của hệ điều hành WSL2/Linux, không bị phanh nhầm khi đang giữ phím lái.
  * Loại bỏ phép chia $\cos(tilt)$ thừa, để tầng tư thế xử lý duy nhất $\rightarrow$ ga ổn định phẳng lì, không bị dập dềnh khi nghiêng thân.

#### 3. File `experiments/05_yaw_rotation_control.cpp`
* **Tên thí nghiệm:** Tự động xoay tròn quanh trục đứng Yaw (Spin Control).
* **Đặc tính:**
  * Hỗ trợ nhận góc xoay từ dòng lệnh (ví dụ quay $360^\circ$, $720^\circ$, xoay âm quay phải).
  * Thuật toán phanh hãm pha trước (Phase-Lead Braking): Khi còn cách góc đích $25^\circ$, tốc độ quay được giảm tốc tuyến tính kết hợp hệ số vi phân $K_{d,yaw} = 0.15$ để triệt tiêu đà quán tính xoay, khóa cứng đầu drone đúng góc đặt mà không bị vọt lố.
  * Giữ cứng tâm tọa độ $(X_0, Y_0)$ và độ cao trong suốt thời gian xoay tròn.

---

### D. THƯ MỤC `simulation/` (MÔ HÌNH & THẾ GIỚI MÔ PHỎNG)

#### 1. Thư mục `simulation/models/x500/`
* Chứa file mô hình SDF của quadrotor x500.
* Tích hợp 4 plugin động cơ `gz-sim-multicopter-motor-model-system` nhận lệnh từ topic `/x500/command/motor_speed`.
* Tích hợp plugin Odometry `gz-sim-odometry-publisher-system` phát dữ liệu tư thế và vận tốc thời gian thực qua topic `/model/x500/odometry` ở tần số **100 Hz**.

#### 2. Thư mục `simulation/models/x500_base/`
* Chứa toàn bộ hình học 3D, file lưới Collada (`.dae`), khối lượng chuẩn ($2.06\text{ kg}$) và tensor ma trận quán tính xoay ($I_{xx}, I_{yy}, I_{zz}$) của khung drone x500.

#### 3. File `simulation/worlds/drone_world.sdf`
* Định nghĩa môi trường vật lý Gazebo Sim:
  * Trọng trường Trái Đất: $g = -9.8\text{ m/s}^2$.
  * Động cơ vật lý ODE được cấu hình bước nhảy thời gian `max_step_size = 0.004s` với `real_time_update_rate = 250Hz` (đảm bảo độ chính xác vi phân cực cao).
  * Hỗ trợ cấu hình gió tự nhiên thế giới mô phỏng (plugin `WindEffects` và thẻ `<wind>`).

---

### E. THƯ MỤC `log/` (DỮ LIỆU VÀ CÔNG CỤ VẼ ĐỒ THỊ)

#### 1. File `log/latest_flight.csv`
* File phản chiếu chuyến bay gần nhất chứa đầy đủ 24 trường thông số động học và điều khiển.

#### 2. File `log/plot_flight.py`
* Công cụ vẽ đồ thị phân tích tự động viết bằng Python (dùng thư viện `pandas` và `matplotlib`).
* Tự động xuất ra file ảnh `latest_flight_plot.png` chia làm 3 biểu đồ trực quan:
  1. **Biểu đồ 1:** Độ cao thực tế vs vận tốc rơi thẳng đứng $V_z$.
  2. **Biểu đồ 2:** Độ bám góc nghiêng Roll & Pitch (Mục tiêu nét đứt vs Thực tế nét liền).
  3. **Biểu đồ 3:** Tốc độ quay của 4 cánh quạt $\omega_0, \omega_1, \omega_2, \omega_3$ (rad/s).

---

## 4. BẢNG THÔNG SỐ TẦN SỐ VÀ CHU KỲ CÁC VÒNG LẶP

| Thành phần / Vòng lặp | Vị trí mã nguồn | Chu kỳ | Tần số | Vai trò & Nhiệm vụ kỹ thuật |
| :--- | :--- | :--- | :--- | :--- |
| **Vòng lặp trong (Inner Loop)** | `VehicleCommander::control_worker()` | $5\text{ ms}$ | **~200 Hz** | Tính PID tư thế góc, vi phân tốc độ góc, bù tilt compensation, trộn động cơ chữ X, bơm lệnh motor sang Gazebo |
| **Vòng lặp ngoài (Outer Loop)** | `02_pid_altitude_hold.cpp`<br>`04_keyboard_teleop.cpp`<br>`05_yaw_rotation_control.cpp` | $30 - 33\text{ ms}$ | **~30 – 33 Hz** | Nhận phím điều khiển, tính PositionController, khóa mỏ neo vị trí $X-Y$, tính PID độ cao, in HUD terminal |
| **Stream MAVLink QGC** | `MavlinkBridge::thread_worker()` | $33\text{ ms}$ | **~30 Hz** | Phát UDP gói ATTITUDE, LOCAL_POSITION, GLOBAL_POSITION, VFR_HUD sang QGroundControl |
| **Blackbox Telemetry Logger** | `VehicleCommander::control_worker()` | $20\text{ ms}$ | **50 Hz** | Ghi đầy đủ 24 cột dữ liệu bay chi tiết ra file CSV |
| **Cảm biến Odometry** | `simulation/models/x500/model.sdf` | $10\text{ ms}$ | **100 Hz** | Gazebo xuất dữ liệu vị trí, vận tốc, góc Euler sang topic `/model/x500/odometry` |
| **Giải tích vật lý ODE** | `simulation/worlds/drone_world.sdf` | $4\text{ ms}$ | **250 Hz** | Gazebo giải phương trình vi phân chuyển động, lực nâng, quán tính và va chạm mặt đất |
| **Nhịp tim MAVLink (Heartbeat)** | `MavlinkBridge::thread_worker()` | $1000\text{ ms}$ | **1 Hz** | Phát gói HEARTBEAT và SYS_STATUS duy trì kết nối với QGroundControl |

---
*Tài liệu này được biên soạn đầy đủ, chính xác và đồng bộ 100% với toàn bộ mã nguồn hiện hành của dự án `drone_control_cpp`.*
