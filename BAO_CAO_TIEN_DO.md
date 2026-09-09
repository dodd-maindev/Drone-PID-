# BÁO CÁO TIẾN ĐỘ: HỆ THỐNG ĐIỀU KHIỂN QUADROTOR THUẦN C++ GIAO TIẾP GAZEBO SIM

**Người báo cáo:** Học viên / Sinh viên thực hiện  
**Dự án:** Điều khiển Drone từ con số 0 (Drone Control From Scratch)  
**Môi trường:** Ubuntu 24.04 / WSL2, Gazebo Sim Harmonic (v8.15), C++17  

---

## 1. Kiến trúc kết nối phần mềm (C++ $\leftrightarrow$ Gazebo Sim)

Thay vì phụ thuộc vào Autopilot có sẵn (PX4) với giao thức MAVLink qua mạng UDP phức tạp và cồng kềnh, hệ thống đã được tái cấu trúc để giao tiếp trực tiếp với môi trường vật lý của **Gazebo Sim** thông qua thư viện gốc **`gz-transport`** và định dạng dữ liệu **`gz-msgs`** theo mô hình **Publish / Subscribe**:

* **Chiều nhận (Subscribe):**
  * Lắng nghe dữ liệu Odometry từ topic `/model/x500/odometry` do plugin `OdometryPublisher` của Gazebo phát ra.
  * Tần số nhận: 100Hz – 250Hz (theo chu kỳ mô phỏng vật lý).
  * Thu nhận các trạng thái: Tọa độ vị trí $(x, y, z)$, vận tốc tịnh tiến $(v_x, v_y, v_z)$, hướng Quaternion $(w, x, y, z)$ và vận tốc góc con quay hồi chuyển $(p, q, r)$.
* **Chiều gửi (Publish):**
  * Tạo Publisher phát dữ liệu vào topic `/x500/command/motor_speed` kiểu thông điệp `gz::msgs::Actuators`.
  * Gửi mảng vận tốc góc $\omega_0, \omega_1, \omega_2, \omega_3$ (đơn vị rad/s) trực tiếp tới 4 động cơ của mô hình drone `x500`.
* **Đặc tính kỹ thuật tầng dưới:**
  * Sử dụng cơ chế **Bộ nhớ chia sẻ (Shared Memory / IPC)** và ZeroMQ nội bộ của Linux.
  * Độ trễ đạt mức micro-giây ($\mu s$), loại bỏ hoàn toàn hiện tượng trễ mạng hoặc xung đột cổng mạng UDP.

---

## 2. Thuật toán điều khiển cân bằng trên cao (Cascaded Flight Control)

Hệ thống sử dụng cấu trúc điều khiển phân tầng (Cascaded Control Loop) gồm 2 vòng lặp phản hồi chính và 1 tầng phối hợp động cơ:

```
[Mục tiêu: Z_target, Roll=0, Pitch=0, Yaw_target]
         │
         ▼
┌────────────────────────────────────────────────────────┐
│ 1. VÒNG NGOÀI: ĐIỀU KHIỂN ĐỘ CAO (Altitude PID)        │
│    - Bù điểm lơ lửng lý thuyết (Hover Thrust = 0.59)    │
│    - PID sai số độ cao                                 │
│    - Phanh vận tốc thẳng đứng (Velocity Damping)       │
└───────────────────────────────────┬────────────────────┘
                                    │ Sinh ra: Lực nâng tổng (Thrust)
                                    ▼
┌────────────────────────────────────────────────────────┐
│ 2. VÒNG TRONG: CÂN BẰNG TƯ THẾ (Attitude PD - 200Hz)   │
│    - Chuyển đổi Quaternion -> Euler (Roll, Pitch, Yaw) │
│    - PD sai số góc & đạo hàm con quay hồi chuyển (p,q,r)│
└───────────────────────────────────┬────────────────────┘
                                    │ Sinh ra: Mô-men 3 trục (tau_roll, tau_pitch, tau_yaw)
                                    ▼
┌────────────────────────────────────────────────────────┐
│ 3. BỘ TRỘN ĐỘNG CƠ (Motor Mixer & Desaturation)        │
│    - Ánh xạ hình học Quadrotor chữ X (FLU frame)       │
│    - Khử bão hòa công suất (Mixer Desaturation)        │
│    - Đổi sang rad/s: w = w_max * sqrt(u)               │
└───────────────────────────────────┬────────────────────┘
                                    │ Bơm vào Gazebo Sim
                                    ▼
                         [4 Động cơ Drone x500]
```

---

### Khối 1: Vòng điều khiển độ cao (Vòng ngoài - Altitude Loop)

Mục tiêu đưa drone đạt và duy trì độ cao $Z_{target}$ ổn định.

1. **Lực nâng cơ sở lơ lửng (Feedforward Hover Thrust):**
   * Khối lượng drone x500: $M \approx 2.064\text{ kg}$.
   * Trọng lượng cần cân bằng: $W = M \cdot g \approx 2.064 \times 9.8 = 20.23\text{ N}$.
   * Lực đẩy của 4 rotor: $F_{total} = 4 \times k_m \cdot \omega^2$ (với $k_m = 8.54858 \times 10^{-6}$).
   * Vận tốc góc cần để lơ lửng:
     $$\omega_{hover} = \sqrt{\frac{W}{4 \cdot k_m}} \approx 769.1\text{ rad/s}$$
   * Giá trị lực nâng chuẩn hóa $[0.0, 1.0]$ với $\omega_{max} = 1000\text{ rad/s}$:
     $$Thrust_{hover} = \left(\frac{769.1}{1000.0}\right)^2 \approx \mathbf{0.59}$$

2. **Bù sai số độ cao bằng PID:**
   * Sai số: $e_z(t) = Z_{target} - Z_{actual}(t)$.
   * Đầu ra bù ga: $\Delta Thrust_{PID} = K_p \cdot e_z + K_i \int e_z dt + K_d \frac{de_z}{dt}$.

3. **Phanh vận tốc dập tắt vọt lố (Velocity Damping):**
   * Để chống hiện tượng quán tính kéo máy bay bay vọt qua mục tiêu (overshoot), hệ thống áp dụng cơ chế phanh phản hồi trực tiếp từ vận tốc thẳng đứng $v_z$:
     $$Thrust_{total} = Thrust_{hover} + \Delta Thrust_{PID} - K_v \cdot v_z$$
   * Khi drone dâng lên nhanh ($v_z > 0$), ga tự động giảm hãm lại.
   * Trần ga được giới hạn ở mức **0.85** nhằm dành sẵn ít nhất 15% công suất dự phòng cho vòng điều khiển tư thế.

---

### Khối 2: Vòng điều khiển tư thế góc nghiêng (Vòng trong - Attitude Loop)

Chạy độc lập trên một thread C++ thời gian thực với tần số **200Hz (chu kỳ 5ms)** để đảm bảo máy bay luôn giữ thăng bằng tuyệt đối.

1. **Thu nhận góc Euler từ Quaternion:**
   Hàm chuyển đổi chuẩn từ Quaternion $(w, x, y, z)$ sang góc Euler $(\phi, \theta, \psi)$:
   $$\begin{cases}
   \phi\text{ (Roll)} = \text{atan2}\left(2(wx + yz), 1 - 2(x^2 + y^2)\right) \\
   \theta\text{ (Pitch)} = \text{asin}\left(2(wy - zx)\right) \\
   \psi\text{ (Yaw)} = \text{atan2}\left(2(wz + xy), 1 - 2(y^2 + z^2)\right)
   \end{cases}$$

2. **Thuật toán điều khiển cân bằng PD:**
   Để máy bay thăng bằng phẳng, góc đặt mục tiêu được giữ ở mức $0$ ($\phi_{cmd} = 0, \theta_{cmd} = 0$):
   $$\begin{cases}
   \tau_{roll} = K_{p,att} \cdot (0 - \phi) - K_{d,att} \cdot p \\
   \tau_{pitch} = K_{p,att} \cdot (0 - \theta) - K_{d,att} \cdot q \\
   \tau_{yaw} = K_{p,yaw} \cdot \text{normalize}(\psi_{cmd} - \psi) - K_{d,yaw} \cdot r
   \end{cases}$$
   *Thành phần $p, q, r$ là vận tốc góc đo trực tiếp từ cảm biến con quay hồi chuyển, có tác dụng dập tắt tức thì mọi rung lắc xoay.*

---

### Khối 3: Bộ trộn động cơ (Motor Mixer) & Khử bão hòa (Mixer Desaturation)

1. **Phương trình phối hợp động cơ Quadrotor khung chữ X:**
   Xét trong hệ tọa độ chuẩn của Gazebo (FLU: $+X$ Tiến, $+Y$ Sang trái, $+Z$ Lên trời):
   * Rotor 0: Trước - Phải ($+X, -Y$), chiều quay CCW.
   * Rotor 1: Sau - Trái   ($-X, +Y$), chiều quay CCW.
   * Rotor 2: Trước - Trái ($+X, +Y$), chiều quay CW.
   * Rotor 3: Sau - Phải  ($-X, -Y$), chiều quay CW.

   Công thức phân bổ công suất chuẩn xác:
   $$\begin{aligned}
   u_0 &= Thrust - \tau_{roll} - \tau_{pitch} + \tau_{yaw} \\
   u_1 &= Thrust + \tau_{roll} + \tau_{pitch} + \tau_{yaw} \\
   u_2 &= Thrust + \tau_{roll} - \tau_{pitch} - \tau_{yaw} \\
   u_3 &= Thrust - \tau_{roll} + \tau_{pitch} - \tau_{yaw}
   \end{aligned}$$

2. **Quy đổi ra vận tốc góc thực tế (rad/s):**
   Do lực đẩy khí động học tỉ lệ với bình phương vận tốc góc ($F \sim \omega^2$):
   $$\omega_i = \omega_{max} \cdot \sqrt{\text{clamp}(u_i, 0.0, 1.0)} \quad (\text{với } \omega_{max} = 1000\text{ rad/s})$$

3. **Thuật toán chống bão hòa công suất (Mixer Desaturation):**
   * Khi lực nâng $Thrust$ lớn, nếu cộng thêm mô-men nghiêng làm một trong các động cơ chạm ngưỡng $u_i > 1.0$ (100% công suất), nếu chỉ cắt ngọn (clamping) thì chênh lệch lực giữa các cánh sẽ bị triệt tiêu $\to$ máy bay mất khả năng thăng bằng và lật nhào.
   * **Giải pháp:** Nếu $\max(u_i) > 1.0$, thuật toán tự động tính độ vượt $\Delta = \max(u_i) - 1.0$ và trừ đều vào toàn bộ 4 cánh:
     $$u_i \leftarrow u_i - \Delta$$
   * Cơ chế này đảm bảo **ưu tiên tuyệt đối cho mô-men giữ thăng bằng tư thế**, drone không bao giờ bị lật úp khi cất cánh đột ngột.

---

## 3. Kết quả thực nghiệm

1. **Kiểm thử cất cánh và bám độ cao (`02_pid_altitude_hold`):**
   * Drone cất cánh thẳng đứng từ mặt đất, không bị rung lắc hay chúi đầu.
   * Bám mức độ cao 2.0m ổn định với sai số $\pm 0.03\text{ m}$.
   * Chuyển mốc độ cao lên 3.5m mượt mà và thực hiện hạ cánh mềm chạm đất an toàn.
2. **Khả năng độc lập phần mềm:**
   * Hoàn toàn không cần cài đặt hoặc khởi động PX4.
   * Bộ điều khiển C++ biên dịch thành file thực thi độc lập, tự bắt kết nối khi thế giới mô phỏng Gazebo khởi chạy.

---

## 4. Kế hoạch nghiên cứu tiếp theo

1. **Bộ điều khiển vị trí mặt phẳng ngang (Position Controller $X - Y$):**
   * Nhận sai số vị trí $e_x, e_y$ để sinh ra góc nghiêng đặt $Roll_{cmd}, Pitch_{cmd}$ tương ứng.
   * Cho phép drone giữ nguyên tọa độ 3D cố định trong không gian (Station Keeping) hoặc bay bám theo chuỗi điểm mốc (Waypoint Tracking).
2. **Nghiên cứu nâng cấp thuật toán điều khiển:**
   * So sánh hiệu quả giữa bộ PID hiện tại với bộ điều khiển phản hồi trạng thái LQR (Linear Quadratic Regulator) hoặc bộ điều khiển dự báo mô hình MPC (Model Predictive Control).
