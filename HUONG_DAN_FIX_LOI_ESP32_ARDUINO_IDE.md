# HƯỚNG DẪN XỬ LÝ LỖI ARDUINO IDE KHÔNG NHẬN ESP32 (COM PORT [NOT CONNECTED])

Tài liệu này ghi lại chi tiết **nguyên nhân**, **cách khắc phục ngay lập tức**, và **quy trình chuyển đổi qua lại giữa Nạp Code (Windows) và Chạy Mô Phỏng HITL (WSL2)**.

---

## 1. Dấu hiệu nhận biết lỗi

Khi mở **Arduino IDE 2.x** trên Windows:
- Góc dưới bên phải hiển thị: `ESP32 Dev Module on COMx [not connected]`.
- Nút chọn Board/Port phía trên chỉ hiện: `Select other board and port...` (không bấm chọn nạp được).
- Thanh màu vàng xuất hiện phía dưới Serial Monitor:  
  `Not connected. Select a board and a port to connect automatically.`
- Trong **Device Manager** của Windows: Mục `Ports (COM & LPT)` bị mất hoặc thiết bị bị nhận diện là `USBIP Shared Device`.

---

## 2. Nguyên nhân cốt lõi (Tại sao bị lỗi?)

Dự án này sử dụng kiến trúc kết hợp giữa **Windows** (để nạp code, chạy QGroundControl) và **WSL2 Ubuntu** (để chạy mô phỏng HITL, Gazebo, C++ Bridge):

1. **Khi chạy mô phỏng HITL trên WSL2**:
   - Công cụ `usbipd-win` được sử dụng để "bắt cóc" (forward) cổng USB của ESP32 từ Windows vào thẳng môi trường ảo Ubuntu (đổi thành `/dev/ttyUSB0`).
   - Để làm được việc này, Windows **buộc phải giải phóng (unload) driver cổng COM tiêu chuẩn (CP210x / CH340)** và gán thiết bị vào driver `USBIP Stub`.

2. **Hậu quả khi nạp code**:
   - Khi ESP32 vẫn còn bị `usbipd` chiếm giữ (`Shared (forced)` hoặc `Attached`), Windows không còn quản lý nó như một cổng Serial COM thông thường nữa.
   - Do đó, **Arduino IDE trên Windows sẽ không thể thấy cổng COM** để kết nối và nạp firmware.

---

## 3. Cách chuyển đổi nhanh (Chọn 1 trong 2 cách)

Bạn có thể thực hiện trên **Windows** hoặc ngay trong **Terminal WSL2**:

### 🎯 Cách 1: Thao tác ngay trong Terminal WSL2 (Khuyên dùng)
- **Khi muốn NẠP CODE Arduino (trả về Windows):**
  ```bash
  ./chuyen_sang_arduino.sh
  ```
- **Khi muốn CHẠY MÔ PHỎNG HITL (gắn vào WSL2):**
  ```bash
  ./chuyen_sang_hitl.sh
  ```

---

### 🎯 Cách 2: Thao tác bằng chuột ngoài Windows (File Explorer)
File đuôi `.bat` là kịch bản chạy trên **Windows** (chạy bằng chuột, không gõ `./file.bat` trong Linux):
- **Khi muốn NẠP CODE**: Click đúp chuột vào `CHUYEN_SANG_ARDUINO_NAP_CODE.bat` (Bấm *Yes* nếu hỏi quyền Admin).
- **Khi muốn CHẠY HITL**: Tắt Serial Monitor trong Arduino IDE rồi click đúp chuột vào `CHUYEN_SANG_HITL_WSL2.bat`.

---

## 4. Cách khắc phục thủ công bằng lệnh (Dành cho lập trình viên)

Nếu bạn muốn thao tác trực tiếp trên Terminal PowerShell:

### Bước 1: Mở PowerShell với quyền Administrator
- Nhấn phím `Windows`, gõ `PowerShell` -> Chuột phải chọn **Run as Administrator**.

### Bước 2: Kiểm tra danh sách thiết bị USB
```powershell
usbipd list
```
Kết quả sẽ hiển thị dạng:
```text
BUSID  VID:PID    DEVICE                                         STATE
2-2    10c4:ea60  Silicon Labs CP210x USB to UART Bridge (COM7)  Shared (forced)
```
*Lưu ý cột BUSID (ví dụ ở đây là `2-2`).*

### Bước 3: Thu hồi ESP32 về lại Windows
Chạy 2 lệnh sau (thay `2-2` bằng BUSID thực tế của máy bạn):
```powershell
usbipd detach --busid 2-2
usbipd unbind --busid 2-2
```

### Bước 4: Kiểm tra lại cổng COM trên Windows
```powershell
Get-CimInstance Win32_SerialPort | Select-Object DeviceID, Name
```
Nếu thấy hiện `COM6` hoặc `COM7` (Silicon Labs CP210x...) là thành công 100%!

---

## 5. Bảng tra cứu các lỗi liên quan thường gặp

| Hiện tượng | Nguyên nhân | Cách xử lý |
| :--- | :--- | :--- |
| **`Attach Request failed - Device busy (exported)`** | Cổng COM đang bị một phần mềm trên Windows chiếm giữ (QGroundControl, Arduino Serial Monitor, PuTTY, hoặc phần mềm 3D). | 1. Tắt hết QGroundControl và tắt tab Serial Monitor trong Arduino IDE.<br>2. Rút cáp USB ra cắm lại.<br>3. Chạy lại script chuyển đổi. |
| **Cắm cáp vào nhưng máy tính không có tiếng chuông, không hiện gì** | Dây cáp USB chỉ có chức năng sạc (chỉ có 2 dây nguồn, thiếu 2 dây data D+/D-), hoặc cổng USB bị lỏng. | Đổi sang dây cáp dữ liệu (loại dùng cắm điện thoại vào máy tính truyền file được), cắm vào cổng USB 3.0 trực tiếp trên bo mạch chủ/laptop. |
| **Khi nạp code Arduino bị lỗi: `Timed out waiting for packet header`** | ESP32 không tự động vào được chế độ Bootloader (do mạch auto-reset thiếu tụ). | Khi Arduino IDE hiện dòng chữ `Connecting........_____.....`: **Nhấn và giữ nút `BOOT` trên board ESP32** khoảng 2 giây rồi thả ra. |
| **Mất Driver CP210x trong Device Manager (hiện dấu chấm than vàng)** | Windows chưa cài driver VCP (Virtual COM Port) cho chip giao tiếp nạp. | Tải và cài đặt driver chính hãng: [Silicon Labs CP210x Universal Windows Driver](https://www.silabs.com/developers/usb-to-uart-bridge-vcp-drivers). |

