# Hiểu nhanh cơ chế 2 thuật toán điều khiển mới trên ESP32: ITSMC & RL Critic-Only + ESO

Cả hai thuật toán đều chạy thời gian thực ngay trên chip ESP32 ở tần số **200 Hz (cứ 5 ms tính một lần)** trong [`firmware_esp32.ino`](./firmware_esp32.ino), giúp drone giữ thăng bằng và bám quỹ đạo dưới tác động của gió và độ trễ phần cứng.

---

## Thuật toán 1: ITSMC (Điều khiển trượt tích phân đầu cuối)
**Code:** [`itsmc_controller.hpp`](./itsmc_controller.hpp) | **Tham số:** `FlightConfig::ITSMC` trong [`flight_config.h`](./flight_config.h)

### Ý tưởng cốt lõi: "Ép trượt về đúng mục tiêu, bất chấp gió đẩy"

Nếu PID giống như một cái lò xo kéo drone về điểm đặt (khi gió thổi ngang dễ bị lệch hoặc phải chờ tích phân `Ki` kéo từ từ), thì **ITSMC** hoạt động giống như việc **vẽ sẵn một đường ray (mặt trượt `s`)** dẫn thẳng về trạng thái cân bằng, rồi dùng lực mạnh để ép drone trượt dọc theo đường ray đó.

### Cách hoạt động trong mỗi chu kỳ 5 ms:

1. **Vừa nhìn độ lệch, vừa cộng dồn sai số kiểu phi tuyến (`sqrt(e)`):**
   - Với sai số lớn, lực kéo tỉ lệ theo căn bậc hai `sqrt(|e|)` kết hợp với `e` và tốc độ thay đổi `de`, giúp drone **kéo về mục tiêu nhanh hơn** so với PID thông thường.
   - Đồng thời nó tích lũy thêm thành phần tích phân `integral_sign_` (có giới hạn `max_integral` chống bão hòa) để **xóa sạch sai số tĩnh** khi drone bị lệch trọng tâm hoặc gió thổi liên tục.

2. **Khử lỗi "rung giật" khi đứng yên (2 mẹo kỹ thuật quan trọng trên ESP32):**
   - **Tuyến tính hóa vùng gần 0 (`eps_lin`):** Hàm `sqrt(|e|)` có nhược điểm là độ dốc cực lớn khi `e` tiến gần 0 (dễ làm drone rung lắc liên tục quanh tâm). Code tự chuyển sang đường thẳng mượt khi `|e| < eps_lin` để drone đứng im phăng phắc.
   - **Làm trơn lớp biên (`tanh(s / phi_b)`):** Điều khiển trượt cổ điển hay bị "chattering" (cánh quạt rít giật liên hồi vì đóng/ngắt đột ngột). Code thay hàm nhảy bậc bằng hàm `tanh` trơn qua bề dày `phi_b` nên motor chạy êm.

3. **Áp dụng trên drone:**
   - Vòng ngoài XY dùng bộ điều khiển vị trí (`PositionController`) để tính góc nghiêng mong muốn.
   - **ITSMC** trực tiếp điều khiển **độ cao Z** (`ITSMCAltitudeController`) và **3 trục góc Roll, Pitch, Yaw** (`ITSMCController`).

---

## Thuật toán 2: RL Critic-Only + ESO (Học tăng cường trực tuyến + Bù nhiễu tức thời)
**Code:** [`rl_critic_controller.hpp`](./rl_critic_controller.hpp) | **Tham số:** `FlightConfig::RLCritic` trong [`flight_config.h`](./flight_config.h)

### Ý tưởng cốt lõi: "Tự chấm điểm, tự chỉnh trọng số mỗi 5 ms — và nhờ ESO lo phần chống gió"

Đây **không phải** kiểu RL cổ điển (như DQN/PPO) phải bay sai, rơi hàng nghìn lần rồi mới học được. Cơ chế cốt lõi của nó là **tự tính toán, tự chấm điểm và tự căn chỉnh trọng số theo thời gian thực ở tần số 200 Hz (mỗi 5 ms một lần ngay trên ESP32)**.

Để bay mượt ngay từ giây đầu tiên mà không bị rơi trong lúc học, nó kết hợp **2 cơ chế chạy song song** trong mỗi chu kỳ 5 ms:

### Cơ chế 1: Tự căn chỉnh trọng số (Online RL Critic-Only)

Thay vì dùng một bộ hệ số $K_P, K_D$ cố định mãi mãi như PID, thuật toán biến các hệ số điều khiển thành trọng số `wp_` (vị trí 3D) và `we_` (góc Roll/Pitch/Yaw) của một mạng Critic, rồi tự chỉnh liên tục theo 3 bước:

1. **Xuất phát thông minh (không học mù):**
   - Nó không khởi tạo trọng số ngẫu nhiên (sẽ làm drone lật ngay), mà đặt sẵn tại **~92% nghiệm tối ưu Riccati** (`kWpInit`, `kWeInit`). Nhờ vậy giây đầu tiên cất cánh đã bay vững.

2. **Tự chấm điểm sau mỗi 5 ms (Bellman Error `eB`):**
   - Ở mỗi tick 5 ms, nó tự kiểm tra: *"Với độ lệch hiện tại và lực vừa phát ra, chi phí thực tế vừa phải trả có khớp với tốc độ giảm chi phí mà mạng Critic dự đoán không?"*
   - Việc chấm điểm và học được **tách riêng từng trục** (X, Y, Z, Roll, Pitch, Yaw) để sai số lúc cất cánh thẳng đứng trục Z không làm hỏng trọng số bay ngang X/Y.

3. **Tự sửa trọng số ngay lập tức:**
   - Nếu chưa khớp (`eB != 0`), nó dịch nhẹ các trọng số `wp_`, `we_` ngay trong tick đó để ép `eB -> 0` (tức là dần tiến sát tới luật điều khiển tối ưu thực tế của con drone).
   - Nếu phát hiện sai số có xu hướng phình ra mất ổn định, số hạng ổn định Lyapunov (`KP2_STAB`, `KE2_STAB`) lập tức kích hoạt để kéo trọng số về vùng an toàn (và luôn kẹp trong biên cho phép quanh khởi tạo Riccati).
   - Vì mạng chỉ gồm **9 trọng số `float`** với vài chục phép nhân cộng (không phải mạng neural hàng triệu tham số), ESP32 tính xong trong vài chục micro-giây, hoàn toàn đáp ứng thời gian thực 200 Hz.

### Cơ chế 2: Cảm nhận & triệt tiêu nhiễu tức thời (ESO bậc 3)

Đây là mảnh ghép cực quan trọng đi kèm với RL:

- Nếu chỉ có RL chỉnh trọng số, khi gặp gió giật mạnh (như gió `[1.5, 0.5, 0] m/s` trong Gazebo), trọng số sẽ phải gồng lên thay đổi liên tục và dễ bị dao động.
- **ESO** giải quyết việc này bằng cách chạy một **"bản sao mô hình ảo"** song song với drone thật (đã tính cả độ trễ motor & truyền thông `ESO_ACT_LAG_*`).
- Cứ mỗi 5 ms, ESO lấy **(vị trí/góc thực tế đo được) − (vị trí/góc mô hình dự đoán)**. Phần chênh lệch đó chính là **tác động của gió và sai số mô hình** $\rightarrow$ ESO ước lượng thành `dis_p_hat_` (lực gió) và `dis_eta_hat_` (mô-men nhiễu) rồi **bù ngược lại thẳng vào lực đẩy/mô-men** ngay ở chu kỳ kế tiếp.

Nhờ ESO đã "dọn sạch" phần nhiễu gió bên ngoài, mạng RL Critic chỉ còn việc tinh chỉnh trọng số `wp_`, `we_` sao cho drone bay êm và tối ưu nhất.

---

## Tóm tắt so sánh nhanh

| Tiêu chí | 1. ITSMC | 2. RL Critic-Only + ESO |
|---|---|---|
| **Triết lý điều khiển** | Ép drone trượt theo đường ray định sẵn (`s`) | Tự chấm điểm (`eB`) & tự nắn trọng số tối ưu mỗi 5 ms |
| **Cách chống gió / nhiễu** | Dùng lực chuyển mạch `tanh` + tích phân phi tuyến | ESO chạy mô hình ảo để đo gió và bù trừ trực tiếp |
| **Hệ số điều khiển** | Cố định (`k11, k12, k2, k3` trong `flight_config.h`) | Tự tinh chỉnh online (`wp_[9]`, `we_[9]`) quanh nghiệm Riccati |
| **Phạm vi áp dụng** | Điều khiển độ cao Z & góc Roll/Pitch/Yaw | Điều khiển toàn bộ cả vòng ngoài 3D (XYZ) lẫn vòng trong (Roll/Pitch/Yaw) |
| **Khi bay HITL thực tế** | Bám tốt, hơi vọt lố nhẹ lúc lên độ cao và gợn nhẹ ở góc cua | Lên độ cao nhanh hơn, không vọt lố, bám cua mượt và tổng sai số thấp hơn |
