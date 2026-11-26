# Nhật ký Kỹ thuật: Chi tiết các Lỗi đã gặp & Cách Khắc phục (Gazebo Harmonic + WSL2 + ESP32 HITL + 4-Drone Formation)

Tài liệu này tổng hợp toàn bộ **7 lỗi thực tế** đã được phát hiện, đo đạc bằng dữ liệu thật và xử lý triệt để trong quá trình triển khai hệ thống **Drone HITL Simulator + Đội hình 4 Drone phân tán** trên môi trường **Windows 11 / WSL2 (Ubuntu) + Gazebo Harmonic 8.15.0 + ESP32**.

---

## Bảng tóm tắt nhanh (Quick Reference)

| # | Triệu chứng thực tế | Nguyên nhân gốc rễ | Nơi sửa / Lệnh xử lý | Trạng thái |
|---|---|---|---|---|
| 1 | Gazebo chạy 5s lại khựng 1 lúc, rồi nhảy cóc | Xung đột đồng hồ `systemd-timesyncd` (NTP) và Hyper-V Windows (`Time jumped backwards` ~15.2s mỗi 5s) | `sudo systemctl disable --now systemd-timesyncd` + đồng bộ giờ Windows | Đã xử lý triệt để |
| 2 | Màn hình Gazebo sọc xanh/đen, GUI rò RAM (`4 GB` sau 5 phút) | `LIBGL_DRI3_DISABLE=1` + `vblank_mode=0` trên WSLg Mesa D3D12 + 59 model nặng @ 60Hz | Bỏ 2 biến env trên ở `scripts/run_hitl_launcher.sh`, đặt `<state_hertz>30</state_hertz>` & gọn ngoại cảnh | Đã xử lý triệt để |
| 3 | Màn hình Gazebo trắng tinh khi mở GUI | `--render-engine ogre` (Ogre 1) xung đột với `<engine>ogre2</engine>` trong `~/.gz/sim/8/gui.config` | Trả mặc định về `Ogre 2` trong `scripts/run_hitl_launcher.sh` | Đã xử lý triệt để |
| 4 | Drone 1 (ESP32) nằm im `0 rad/s` trong khi Drone 2–4 bay bình thường | Tính XOR checksum sai offset 2 byte header (`0xAA ^ 0x55 = 0xFF`), đọc 1 lần UART RX, binary cũ & thiếu đồng bộ `g_target_alt` | Sửa `experiments/07_hitl_formation_bridge.cpp` + rebuild & sync ra `bin/` và `DroneHITL/bin/` | Đã xác minh (`177` gói OK, `615 rad/s`) |
| 5 | Drone 1 (ESP32) cất cánh đẹp tới `t = 12.6s` rồi rung lật rơi xuống đất | Chế độ đội hình ép `RL Critic + ESO` rất nhạy nhiễu GPS/IMU + hệ số ngang `kp_xy = 1.4` gắt | Đổi mặc định sang `ITSMC`, giảm `kp_xy = 0.85` (`±1.6 m/s`), thêm `STANDBY` chờ phím `1` | Đã bay `238.7s` liên tục ở `z = 4.997m` |
| 6 | Cả 4 Drone đứng im hoặc vừa lên đã nghiêng lật ngửa mỗi con một góc | Topic motor thiếu `/x500_k/command/motor_speed`, `<odom_topic>` fix cứng, chạy đồng thuận ngang ngay dưới đất `t = 0s`, gain SMC/HGO quá gắt | Publish cả 2 topic motor, bỏ fix cứng `<odom_topic>`, cất cánh thẳng `0..4s` (`z < 0.45m` giữ phẳng), làm dịu gain SMC | Đã xử lý (`D1..D4` bay đội hình kim cương ở `z = 5m`) |
| 7 | Tọa độ GPS trên QGroundControl nhảy vọt hàng chục km | Gazebo `navsat` lấy nhiễu `<horizontal><noise><stddev>` theo đơn vị **độ** (`1 deg ≈ 111,320 m`), không phải mét | Đổi `stddev` từ `0.60` → `4.5e-6` độ (`≈ 0.50 m`) trong `simulation/models/x500_base/model.sdf` | Đã xử lý (`Valid.`) |

---

## Chi tiết từng lỗi & cách khắc phục

### 1. Gazebo bị khựng theo chu kỳ cứ 5 giây/lần trên WSL2 (Chạy một xíu → Đứng → Chạy tiếp)

- **Triệu chứng:**
  - Hình ảnh trên Gazebo bị đứng yên vài giây rồi drone nhảy vọt một đoạn, lặp đi lặp lại đều đặn.
  - Tuy nhiên tốc độ giải vật lý nội tại (`gz topic -e -t /stats`) vẫn giữ `real_time_factor ≈ 1.000` và RAM của `gz sim server` ổn định (~251 MB).
- **Chẩn đoán & Bằng chứng đo được:**
  - Đo luồng `/world/default/dynamic_pose/info` thấy cứ khoảng **5.01 giây** lại xuất hiện khoảng trống bất thường:
    ```text
    t=8.08s  msg#398 GAP=15.229s
    t=13.09s msg#650 GAP=15.231s
    ```
  - Kiểm tra log nhân Linux (`dmesg`) phát hiện xung đột thời gian liên tục đúng chu kỳ 5 giây:
    ```text
    [ 7343.962272] systemd-journald[40]: Time jumped backwards, rotating.
    [ 7348.972914] systemd-journald[40]: Time jumped backwards, rotating.
    [ 7353.999494] systemd-journald[40]: Time jumped backwards, rotating.
    ```
- **Nguyên nhân:**
  - Trên WSL2 tồn tại 2 cơ chế chỉnh đồng hồ chạy song song: **Hyper-V Host Sync** từ Windows đẩy giờ sang Linux mỗi ~5 giây và dịch vụ **`systemd-timesyncd`** trong Ubuntu tự đồng bộ NTP qua mạng (`ntp.ubuntu.com`).
  - Khi đồng hồ Windows lệch so với NTP (hoặc sai ngày/giờ hệ thống), 2 bên giằng co kéo đồng hồ hệ thống WSL2 nhảy lùi ~15 giây liên tục. Mọi `QTimer` / bộ điều tiết khung hình trong Gazebo GUI & `SceneBroadcaster` dựa trên đồng hồ tường bị treo theo mỗi cú nhảy.
- **Cách khắc phục:**
  1. Tắt `systemd-timesyncd` trong Ubuntu WSL2 để chỉ dùng duy nhất một nguồn giờ thống nhất từ Windows:
     ```bash
     sudo systemctl disable --now systemd-timesyncd
     ```
  2. Trên Windows, vào **Settings → Time & language → Date & time → Sync now** để đồng bộ ngày giờ Windows chuẩn với Internet.
- **Cách kiểm chứng sau khi sửa:**
  ```bash
  dmesg | grep "Time jumped" | tail -n 5
  ```
  Đợi 10–15 giây chạy lại không còn dòng mới xuất hiện; Gazebo chạy mượt liên tục không còn chu kỳ khựng 5 giây.

---

### 2. Gazebo sọc màn hình (xanh/đen), vỡ bộ đệm & ngốn RAM/VRAM (`WSLg + D3D12`)

- **Triệu chứng:**
  - Sau khi mở vài phút, khung hình bị sọc ngang màu xanh/nhiễu toàn bộ nửa trên, nửa dưới đen kịt.
  - Tiến trình `gz sim gui` ăn CPU ~70–78% và phình RAM rất nhanh (`1.38 GB` sau 78s → `4.01 GB` sau 4 phút 55s, chạm ngưỡng `4 GB VRAM` của RTX 3050 Ti Laptop GPU).
- **Nguyên nhân:**
  1. `scripts/run_hitl_launcher.sh` đặt `export LIBGL_DRI3_DISABLE=1` cùng `GALLIUM_DRIVER=d3d12`, làm tắt chia sẻ GPU buffer zero-copy với Weston/XWayland, ép copy từng khung hình qua CPU và làm rách/vỡ framebuffer khi đầy VRAM.
  2. Đặt `export vblank_mode=0` khiến vòng lặp render đẩy khung hình không giới hạn vào hàng đợi XWayland.
  3. Thế giới `drone_formation_world.sdf` ban đầu chứa tới 59 model (25 cây thông, 11 cây sồi dùng material script cũ, 7 ngôi nhà, 5 xe, nhà kho 8.9 MB, 4 sân đáp) và `SceneBroadcaster` phát ở tần số mặc định 60 Hz.
- **Cách khắc phục:**
  - Trong `scripts/run_hitl_launcher.sh` (và đồng bộ sang `DroneHITL/run.sh`):
    - **Xóa** `export LIBGL_DRI3_DISABLE=1` và **không dùng** `export vblank_mode=0`.
    - Giữ tăng tốc phần cứng sạch cho RTX 3050 Ti: `GALLIUM_DRIVER=d3d12`, `MESA_D3D12_DEFAULT_ADAPTER_NAME=NVIDIA`, `QT_QPA_PLATFORM=xcb`.
  - Trong `simulation/worlds/drone_formation_world.sdf` (và đồng bộ sang `DroneHITL/simulation/worlds/drone_formation_world.sdf`):
    - Đặt `<state_hertz>30</state_hertz>` trong plugin `SceneBroadcaster` để giảm một nửa lưu lượng đồng bộ pose sang GUI mà vẫn mượt.
    - Rút gọn ngoại cảnh ở xa không nằm trong tầm nhìn (giữ bãi đáp ISILAB trung tâm, đường nhựa, billboard chính, 2 nhà, 2 xe và 4 cây gần) nhưng **giữ nguyên 100% mô hình 4 drone `model://x500`** theo yêu cầu giữ ngoại hình gốc.

---

### 3. Màn hình Gazebo trắng tinh khi chuyển sang Ogre 1 (`--render-engine ogre`)

- **Triệu chứng:** Toàn bộ khung nhìn 3D của Gazebo Sim hiển thị một màu trắng hoàn toàn (trong khi thanh công cụ và cây Entity bên phải vẫn hiện chữ `Render Engine Gui Plugin: ogre`).
- **Nguyên nhân:** Giao diện mặc định của Gazebo Harmonic 8.15.0 dùng cấu hình `~/.gz/sim/8/gui.config` với plugin `MinimalScene` cấu hình `<engine>ogre2</engine>`. Việc truyền cờ `--render-engine ogre` xung đột với thiết lập Ogre 2 của `MinimalScene` trên WSLg D3D12 dẫn tới không vẽ được khung 3D.
- **Cách khắc phục:**
  - Trả cấu hình mặc định trong `scripts/run_hitl_launcher.sh` (và `DroneHITL/run.sh`) về **`Ogre 2`** (không tự động chèn `--render-engine ogre`).
  - Vẫn giữ các cờ tùy chọn `--ogre1`, `--ogre2`, `--sw-render` dành riêng cho trường hợp cần chẩn đoán thủ công.

---

### 4. Drone 1 (chạy ESP32 HITL) nằm im trên mặt đất trong chế độ 4 Drone

- **Triệu chứng:** Khi chạy đội hình 4 drone, Drone 2, 3, 4 cất cánh bình thường nhưng Drone 1 (`x500_1` nối với ESP32 qua `/dev/ttyUSB0`) đứng yên ở `(3.0, 2.0, 0.0)` với tốc độ quạt `0 rad/s`.
- **Nguyên nhân:**
  1. **Sai offset Checksum (lỗi chính):** Firmware trên ESP32 (`firmware_esp32/firmware_esp32.ino`, dòng 498, 771, 952) và cầu nối 1 drone (`experiments/06_hitl_esp32_bridge.cpp`, dòng 1653, 1681) tính XOR checksum **chỉ trên phần payload** (bỏ 2 byte header đồng bộ `0xAA 0x55` / `0x55 0xAA`: con trỏ `+ 2`, độ dài `sizeof - 3`). Ngược lại, `experiments/07_hitl_formation_bridge.cpp` trước đó gọi `calc_checksum(&pkt, sizeof - 1)` tính từ byte `0`. Do `0xAA ^ 0x55 = 0xFF`, toàn bộ checksum bị đảo bit, khiến **ESP32 loại bỏ 100% gói `SensorPacket`** và giữ nguyên trạng thái `DISARMED`.
  2. **Đọc UART RX chưa vét sạch buffer & trượt đồng bộ sai:** Mỗi vòng lặp 10ms chỉ gọi `read()` một lần thay vì `while ((nr = read(...)) > 0)` (ESP32 gửi ở 200Hz), và khi sai checksum lại xóa cả `sizeof(ActuatorPacket)` byte thay vì dịch từng `1` byte.
  3. **Chưa đồng bộ file nhị phân mới nhất** ra `bin/07_hitl_formation_bridge` và `DroneHITL/bin/07_hitl_formation_bridge`.
  4. **Trình tích phân độ cao trên ESP32 (`g_target_alt`):** Firmware cộng dồn `g_target_alt += s.cmd_alt_vel * DT` ở 200Hz; nếu gửi thẳng bộ điều khiển vị trí Z mà không bám điểm đặt ảo, dễ gây tích phân quá đà (double-integrator windup).
- **Cách khắc phục (không cần nạp lại code ESP32):**
  - Trong `experiments/07_hitl_formation_bridge.cpp`:
    - Xóa `calc_checksum`, dùng `compute_packet_checksum(reinterpret_cast<const uint8_t*>(&pkt) + 2, sizeof(SensorPacket) - 3)` cho cả TX và RX.
    - Đọc hết bộ đệm RX bằng vòng `while ((nr = read(...)) > 0)` và dịch từng `1` byte khi mất đồng bộ hoặc sai checksum.
    - Đồng bộ chuỗi chuyển trạng thái sạch trên ESP32 (`0..0.25s`: `DISARMED`; `0.25..0.60s`: `CTRL_FLAG_ARMED | CTRL_FLAG_ALT_ACTIVE | CTRL_FLAG_TAKEOFF`; `>= 0.60s`: `CTRL_FLAG_ARMED | CTRL_FLAG_ALT_ACTIVE` kèm biến bám `esp_virtual_alt`).
  - Biên dịch lại và đồng bộ đầy đủ ra `build/07_hitl_formation_bridge`, `bin/07_hitl_formation_bridge` và `DroneHITL/bin/07_hitl_formation_bridge`.
- **Cách kiểm chứng:**
  - Test bắt tay trực tiếp qua `/dev/ttyUSB0` @ `921600 baud`: nhận **`177` gói `ActuatorPacket` hợp lệ**, lúc `DISARMED` trả `(0, 0, 0, 0)` (`status_flags = 20`), lúc `ARM + TAKEOFF` trả `(615.0, 615.0, 615.0, 615.0) rad/s` (`status_flags = 21`).

---

### 5. Drone 1 (ESP32) bay lên tới `5m` rồi bị rung lật rơi xuống đất ở `t ≈ 12.8s` + Nhu cầu đợi bấm phím `1`

- **Triệu chứng:**
  - Theo log `logs/formation/trajectory/latest_formation.csv`, Drone 1 (ESP32) cất cánh chuẩn và bám tới đỉnh `(4.0, 0.0, 5.0)` tại `t = 10..12.6s` (`D1=(5.19, -0.32, 5.14)`).
  - Tuy nhiên từ `t = 12.8s → 13.9s`, nó nghiêng dao động và rơi từ `5.14m` xuống `0.18m` rồi trượt trên sân.
  - Ngoài ra, người dùng cần khi mới khởi động mô phỏng thì cả 4 drone **nằm yên trên mặt đất (`STANDBY`)**, chỉ cất cánh khi bấm phím **`1`**.
- **Nguyên nhân:**
  - `scripts/run_hitl_launcher.sh` (dòng 152) và `07_hitl_formation_bridge.cpp` trước đó ép Drone 1 chạy **`RL Critic + ESO`** (`--algo rl`). Dưới tải 4 drone + nhiễu cảm biến `StateEstimator` + trễ UART/Gazebo, bộ quan sát bậc 3 ESO tích lũy sai lệch (`dis_eta_hat`, `dis_p_hat`) làm lật thân drone.
  - Hệ số lái ngang gửi cho ESP32 trong `compute_esp32_velocity_cmds()` ban đầu khá gắt (`kp_xy = 1.4f`, giới hạn `±2.5 m/s` trong khi vận tốc quỹ đạo chỉ `0.60 m/s`).
- **Cách khắc phục:**
  - Đổi thuật toán mặc định cho ESP32 trong chế độ đội hình sang **`ITSMC`** (`ALGO_ARG="itsmc"` ở `run_hitl_launcher.sh`, `selected_algo = ALGO_ITSMC` ở `07_hitl_formation_bridge.cpp`) — bộ trượt tích phân có giới hạn, miễn nhiễm trôi ESO dưới nhiễu cảm biến.
  - Làm dịu lệnh vận tốc trong `include/formation/formation_controller.hpp` (`kp_xy: 1.4 → 0.85`, giới hạn `±1.6 m/s`) và `alt_trim` (`0.18 * err_z - 0.12 * vz_w`, giới hạn `±0.25`) trong `07_hitl_formation_bridge.cpp`.
  - Thêm máy trạng thái **`FlightMode { STANDBY, FLYING, LANDING }`** cùng hàm đọc bàn phím không chặn (`TerminalRawGuard` + `select`): mới mở nằm yên ở `STANDBY` (vẫn gửi gói giữ kết nối ESP32 mỗi 10ms), bấm **`1`** cất cánh (gọi `sync_start_positions()`), bấm **`0`** hạ cánh (`step_landing()`).
- **Kết quả xác minh:**
  - Đo thực tế trong log bay sau khi sửa: Drone 1 (ESP32 với `ITSMC`) bay liên tục **238.7 giây** cùng cả 4 drone, ở giây `238.73s` giữ độ cao `D1.z = 4.9968 m`, `D2.z = 5.0005 m`.

---

### 6. Lỗi ban đầu khiến cả 4 Drone đứng im hoặc vừa nhấc lên đã lật ngửa dưới đất

- **Triệu chứng:**
  - Lần chạy đầu tiên cả 4 drone không quay cánh quạt.
  - Sau khi mở khóa topic motor, vừa nhấc lên khỏi đất cả 4 con bị nghiêng mạnh, va xuống bãi đáp và mỗi con lăn một góc.
- **Nguyên nhân:**
  1. Plugin `MulticopterMotorModel` của Gazebo Harmonic lắng nghe topic `/<model_name>/command/motor_speed` (không có tiền tố `/model/`), trong khi bridge ban đầu chỉ publish `/model/x500_k/command/motor_speed`.
  2. File `simulation/models/x500/model.sdf` cũ ghi cứng `<odom_topic>/model/x500/odometry</odom_topic>`, làm cả 4 drone đè chung một topic odometry thay vì `/model/x500_1..4/odometry`.
  3. Đồ thị đồng thuận (Consensus) chạy sai lệch ngang ngay tại `t = 0s` khi các drone còn ở trên mặt đất ở `(±3, ±2)` cách đỉnh kim cương `(±4, 0)` hơn 2m → bộ SMC nghiêng thân tới 25° ngay trên bãi đáp làm càng đáp quệt đất và lật ngửa.
  4. Hệ số SMC ngang quá gắt + đạo hàm bậc 2 `ddeta_des` của HGO khuếch đại nhiễu ở tần số 100Hz + thiếu bù lực nâng `1 / cos(tilt)`.
- **Cách khắc phục:**
  - Publish đồng thời cả `/<model_name>/command/motor_speed` và `/model/<model_name>/command/motor_speed` trong `experiments/07_hitl_formation_bridge.cpp`.
  - Xóa `<odom_topic>` cố định khỏi `simulation/models/x500/model.sdf` (và đồng bộ sang `DroneHITL/`).
  - Trong `include/formation/formation_controller.hpp`:
    - Chia pha bay: `0..4s` cất cánh thẳng đứng lên `z = 4.0m` (`dp_des.x = dp_des.y = 0`), từ `4..10s` mới chạy đồng thuận xếp đội hình kim cương lên `z = 5.0m`, từ `>= 10s` bay quỹ đạo tròn bán kính `4m`.
    - Khóa phẳng khi sát đất (`if (st.z < 0.45f) ux = uy = 0;`).
    - Giảm quá tắt dần cho trục ngang (`alpha_pxy = 0.65`, `kpi1_xy = 1.40`, `kpi2_xy = 0.25`, giới hạn góc nghiêng `15°`), bù lực nâng `uz / cos_tilt` (`min 0.85`), và tắt nhiễu `ddeta_des` trong vòng thái độ SMC.

---

### 7. GPS trên QGroundControl nhảy vọt hàng chục km (sang Bắc Ninh / Hưng Yên)

- **Triệu chứng:** Khi bật nhiễu cảm biến ngoài trời trong `x500_base/model.sdf`, biểu tượng drone trên bản đồ QGroundControl nhảy loạn xạ khỏi khu vực bãi bay hàng chục km.
- **Nguyên nhân:** Trong thẻ `<navsat><position_sensing><horizontal><noise>` của Gazebo Harmonic, tham số `<stddev>` được tính theo đơn vị **độ kinh/vĩ (degrees)** (`1° ≈ 111,320 m`), không phải mét. Giá trị `0.60` bị hiểu thành `0.60° ≈ 66.8 km`!
- **Cách khắc phục:** Đặt lại `<stddev>4.5e-6</stddev>` (`4.5e-6 × 111,320 m ≈ 0.50 m`) trong `simulation/models/x500_base/model.sdf` (và đồng bộ sang `DroneHITL/simulation/models/x500_base/model.sdf`), kiểm tra hợp lệ bằng `gz sdf -k`.

---

## Checklist tự kiểm tra nhanh khi gặp sự cố

1. **Kiểm tra nhảy đồng hồ WSL2 (gây khựng Gazebo mỗi 5s):**
   ```bash
   dmesg | grep "Time jumped" | tail -n 5
   systemctl is-active systemd-timesyncd   # Phải là: inactive
   ```
2. **Kiểm tra tốc độ vật lý Gazebo & luồng truyền pose:**
   ```bash
   GZ_IP=127.0.0.1 GZ_PARTITION=drone_hitl gz topic -e -t /stats -n 2 | grep real_time_factor
   ```
3. **Kiểm tra GPU D3D12 trên WSL2:**
   ```bash
   GALLIUM_DRIVER=d3d12 MESA_D3D12_DEFAULT_ADAPTER_NAME=NVIDIA LD_LIBRARY_PATH="/usr/lib/wsl/lib:$LD_LIBRARY_PATH" glxinfo -B | grep -E "Device|Accelerated"
   ```
4. **Kiểm tra cổng Serial ESP32 & đồng bộ binary:**
   ```bash
   ls -la /dev/ttyUSB* /dev/ttyACM*
   ls -la build/07_hitl_formation_bridge bin/07_hitl_formation_bridge DroneHITL/bin/07_hitl_formation_bridge
   ```
