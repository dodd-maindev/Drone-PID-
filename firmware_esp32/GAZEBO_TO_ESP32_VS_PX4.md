# Phân tích luồng dữ liệu Gazebo → PC Bridge → ESP32 và So sánh với PX4

Tài liệu này làm rõ 3 câu hỏi cốt lõi về kiến trúc HITL hiện tại của dự án:
1. **ESP32 nhận được những dữ liệu gì từ Gazebo?**
2. **Dữ liệu đó lấy từ đâu (cảm biến thô hay ground-truth)?**
3. **Cơ chế này đã giống PX4 chưa, giống và khác ở điểm nào?**

---

## 1. ESP32 nhận được những gì từ Gazebo?

ESP32 **không** giao tiếp trực tiếp với Gazebo, mà nhận dữ liệu thông qua cầu nối PC ([`06_hitl_esp32_bridge.cpp`](../experiments/06_hitl_esp32_bridge.cpp)) qua **USB Serial (UART 921600 baud)** ở tần số **100 Hz** (`10 ms/gói`).

Dữ liệu gửi xuống ESP32 là struct nhị phân **`SensorPacket` (69 bytes)** định nghĩa tại [`firmware_esp32/telemetry_packet.hpp`](./telemetry_packet.hpp):

| Nhóm dữ liệu | Các trường trong `SensorPacket` | Đơn vị / Hệ quy chiếu | Ý nghĩa |
|---|---|---|---|
| **Header** | `header[2]` (`0xAA, 0x55`) | - | Byte đồng bộ đầu gói |
| **Vị trí 3D** | `x`, `y`, `z` | mét (m), World ENU | Tọa độ ước lượng theo Đông–Bắc–Lên |
| **Vận tốc** | `vx`, `vy`, `vz` | m/s | `vx, vy`: hệ **Body** (Tiến / Trái); `vz`: hệ **World ENU** (Lên) |
| **Góc thái độ** | `roll`, `pitch`, `yaw` | radian (rad) | Góc Euler ZYX ước lượng |
| **Vận tốc góc** | `p`, `q`, `r` | rad/s | Tốc độ xoay quanh 3 trục thân (Gyro) |
| **Lệnh bay** | `cmd_vx`, `cmd_vy`, `cmd_alt_vel`, `cmd_yaw_rate` | m/s, rad/s | Lệnh từ `WaypointNavigator` (`--auto`) hoặc tay cầm/bàn phím |
| **Cờ & Chế độ** | `flags`, `algo_mode` | bitmask / enum | `ARMED`, `DRIVING_X/Y`, `ALT_ACTIVE`, `TAKEOFF`, `LAND`, `algo_mode` (`0=ITSMC`, `1=RL`, `2=PID`) |
| **Checksum** | `checksum` | XOR 1 byte | Kiểm tra toàn vẹn gói tin |

Ngược lại, sau khi tính toán bộ điều khiển ở **200 Hz**, ESP32 gửi trả PC gói **`ActuatorPacket` (28 bytes)** gồm: `w0, w1, w2, w3` (tốc độ góc 4 cánh quạt, rad/s), `target_pitch_deg`, `target_roll_deg`, và `status_flags`. PC đẩy thẳng `w0..w3` lên topic `/x500/command/motor_speed` của Gazebo.

---

## 2. Dữ liệu đó nhận được từ đâu? (Cảm biến thật trong Gazebo hay Ground-Truth?)

**Nhận từ 4 cảm biến thô có nhiễu vật lý trong Gazebo — KHÔNG phải đọc cheat Ground-Truth Odometry.**

Cụ thể, trong [`simulation/models/x500_base/model.sdf`](../simulation/models/x500_base/model.sdf), mô hình X500 gắn 4 cảm biến mô phỏng theo phần cứng thật của PX4:

| Cảm biến Gazebo | Topic GZ-Transport | Tần số | Nhiễu mô phỏng (Gaussian) | Phần cứng tham chiếu |
|---|---|---:|---|---|
| **IMU** (`imu_sensor`) | `/world/default/model/x500/link/base_link/sensor/imu_sensor/imu` | 250 Hz | Gyro `0.000873 rad/s`<br>Accel `0.00637 / 0.00686 m/s²` | TDK IIM42653 |
| **GPS / NavSat** (`navsat_sensor`) | `/world/default/model/x500/link/base_link/sensor/navsat_sensor/navsat` | 30 Hz | `lat, lon, alt`, `vel_E/N/U` | GNSS Receiver |
| **Barometer** (`air_pressure_sensor`) | `/world/default/model/x500/link/base_link/sensor/air_pressure_sensor/air_pressure` | 50 Hz | `stddev = 3 Pa` | Bosch BMP390 |
| **Magnetometer** (`magnetometer_sensor`) | `/world/default/model/x500/link/base_link/sensor/magnetometer_sensor/magnetometer` | 100 Hz | `stddev = 5e-7 T` (5 mgauss) | ST IIS2MDC |

### Quá trình gộp cảm biến (Sensor Fusion) diễn ra như thế nào?

[`06_hitl_esp32_bridge.cpp`](../experiments/06_hitl_esp32_bridge.cpp) (dòng 617–620) subscribe đủ 4 topic cảm biến trên và đưa vào lớp **`DroneEstimation::StateEstimator`** ([`src/estimation/state_estimator.cpp`](../src/estimation/state_estimator.cpp)):

1. **Ước lượng góc (`roll, pitch, yaw`):**
   - Tích phân Gyro (`gx, gy, gz`) theo động học Euler ở **250 Hz**.
   - Dùng **Complementary Filter** từ gia tốc kế (`ax, ay, az`) với hệ số nhỏ (`alpha = 0.0016` khi gần tĩnh, `0.0002` khi đang gia tốc mạnh) để khử trôi Roll/Pitch.
   - Dùng **Magnetometer bù nghiêng (Tilt-compensated)** để hiệu chỉnh trôi Yaw ở **100 Hz** (tự động lấy trung bình 20 mẫu đầu trên mặt đất làm chuẩn `yaw = 0`).
2. **Ước lượng vị trí & vận tốc (`x, y, z, vx, vy, vz`):**
   - Xoay gia tốc IMU từ hệ Body sang World ENU, trừ trọng lực `g = 9.80665 m/s²`.
   - Chạy **Kalman Filter 2 trạng thái** (`[pos, vel]`) độc lập cho từng trục `X, Y, Z`:
     - **Predict (250 Hz):** từ gia tốc IMU.
     - **Update GPS (30 Hz):** đổi `lat/lon` sang ENU (mét) và cập nhật vị trí + vận tốc `X, Y, Z`.
     - **Update Barometer (50 Hz):** quy đổi áp suất khí quyển theo công thức ICAO (calib `P0` từ 15 mẫu đầu) để cập nhật chính xác độ cao `Z`.
3. Sau khi `StateEstimator` gộp xong, PC lấy trạng thái đã ước lượng đóng gói vào `SensorPacket` gửi qua Serial xuống ESP32.

*(Lưu ý: File `src/comm/gz_bridge.cpp` trước đây có hàm `on_odometry` đọc `/model/x500/odometry`, nhưng bridge HITL `06_hitl_esp32_bridge.cpp` hiện tại **không** dùng `GzBridge` mà dùng trực tiếp 4 cảm biến thô + `StateEstimator`.)*

---

## 3. Đã giống như bên PX4 chưa?

### Những điểm ĐÃ GIỐNG PX4
1. **Đầu vào cảm biến chuẩn PX4 X500:** Dùng đúng 4 cảm biến (IMU 250Hz, GPS 30Hz, Baro 50Hz, Mag 100Hz) với cùng thông số nhiễu phần cứng (IIM42653, BMP390, IIS2MDC) từ `x500_base/model.sdf` của PX4.
2. **Không dùng Ground-Truth để điều khiển:** ESP32 điều khiển dựa trên trạng thái đã qua bộ lọc Kalman + nhiễu cảm biến thực tế, không dùng tọa độ tuyệt đối lý tưởng của Gazebo.
3. **Chu trình khép kín tương đương:** `Cảm biến Gazebo → Sensor Fusion → Bộ điều khiển (Vị trí + Thái độ) → Motor Mixer → Tốc độ Rotor (rad/s) → Gazebo`, kèm giao tiếp **MAVLink** với QGroundControl.

### Những điểm KHÁC với PX4

| Tiêu chí | PX4 Autopilot (SITL / HITL chuẩn) | Hệ thống hiện tại (ESP32 HITL) |
|---|---|---|
| **Nơi chạy EKF (Sensor Fusion)** | **Chạy ngay trên Flight Controller (MCU)** — nhận cảm biến thô (hoặc `HIL_SENSOR` qua MAVLink) và tự chạy EKF2 bên trong firmware. | **Chạy trên PC** (`StateEstimator` trong `06_hitl_esp32_bridge.cpp`). PC gộp xong mới gửi trạng thái (`x, y, z, roll, pitch, yaw...`) xuống ESP32. |
| **Độ phức tạp của EKF** | **PX4 EKF2 (24 trạng thái):** Error-state EKF đầy đủ, ước lượng cả Gyro/Accel bias, Mag bias, vận tốc gió, và có bộ đệm trễ (fusion horizon) cho GPS/Baro. | **Estimator gọn nhẹ:** Complementary Filter cho góc + 3 bộ Kalman tuyến tính 2-state cho `X, Y, Z`; chưa ước lượng IMU bias hay bù trễ phép đo GPS trong EKF. |
| **Giao thức PC ↔ MCU** | **MAVLink** (`HIL_SENSOR` → `HIL_ACTUATOR_CONTROLS` chuẩn hóa `[-1, 1]` hoặc `[0, 1]`). | **Binary struct riêng** (`SensorPacket` 69B → `ActuatorPacket` 28B) qua UART 921600 baud (PC gửi 100Hz, ESP32 chạy control 200Hz), xuất thẳng `w0..w3` (rad/s). |
| **Tách biệt phần mềm** | Firmware PX4 đảm nhận cả Estimation + Control + Navigation + MAVLink. | PC đảm nhận **Estimation (EKF) + WaypointNavigator + MAVLink/QGC**; ESP32 tập trung toàn bộ tài nguyên cho **Control (ITSMC / RL Critic+ESO / PID) + Motor Mixer**. |

---

## Tóm tắt ngắn gọn để báo cáo

- **ESP32 nhận gì?** Nhận **trạng thái ước lượng** (vị trí `x, y, z`, vận tốc `vx, vy, vz`, góc `roll, pitch, yaw`, tốc độ góc `p, q, r`) + **lệnh bay & cờ chế độ** qua `SensorPacket` (69 bytes, 100Hz).
- **Nhận từ đâu?** Từ **4 cảm biến thô có nhiễu thực tế của Gazebo** (`IMU 250Hz`, `GPS 30Hz`, `Barometer 50Hz`, `Magnetometer 100Hz`) sau khi được gộp qua bộ lọc **EKF/Kalman (`StateEstimator`)** trên PC — **không** đọc ground-truth odometry.
- **Giống PX4 chưa?** **Giống về mặt nguồn cảm biến đầu vào và vòng kín vật lý**, nhưng **khác ở chỗ bộ gộp EKF hiện đặt trên PC** (để giảm tải UART/CPU cho ESP32, còn ESP32 chuyên chạy thuật toán điều khiển 200Hz), và dùng bộ Kalman gọn nhẹ thay vì EKF2 24-state của PX4.
