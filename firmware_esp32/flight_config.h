/**
 * @file flight_config.h
 * @brief TẬP TRUNG TOÀN BỘ THAM SỐ CẤU HÌNH & HIỆU CHỈNH BAY (CENTRAL FLIGHT CONFIGURATION)
 * 
 * =========================================================================================
 * NGUYÊN TẮC THIẾT KẾ DÀNH CHO BẢO VỆ ĐỒ ÁN / VẬN HÀNH:
 * 1. Mọi tham số vật lý, giới hạn an toàn, hệ số PID và thông số phần cứng ĐỀU NẰM TẠI ĐÂY.
 * 2. Khi Thầy yêu cầu: "Đổi góc nghiêng", "Tăng giảm chấn", "Sửa tốc độ phanh", "Đổi tần số",...
 *    chỉ cần mở DUY NHẤT file này, sửa giá trị và nạp lại trong 5 giây, KHÔNG CẦN BỚI CODE!
 * =========================================================================================
 */

#pragma once
#include <stdint.h>

namespace FlightConfig {

    // =====================================================================================
    // 1. CẤU HÌNH PHẦN CỨNG & GIAO TIẾP (HARDWARE & COMMUNICATION)
    // =====================================================================================
    namespace Hardware {
        // Tốc độ truyền Serial UART giữa ESP32 và Máy tính (Baud rate)
        // Lưu ý: 921600 baud truyền gói tin 48 bytes chỉ mất ~0.52ms, đảm bảo chu kỳ 200Hz mượt mà.
        constexpr uint32_t UART_BAUD_RATE        = 921600;

        // Kích thước bộ đệm phần cứng RX (bytes) để chống tràn buffer khi PC gửi liên tục
        constexpr size_t   UART_RX_BUFFER_SIZE   = 2048;

        // Chân GPIO điều khiển LED trạng thái (LED nhấp nháy báo hiệu đang nhận dữ liệu)
        constexpr uint8_t  STATUS_LED_PIN        = 2;

        // Thời gian mất kết nối tối đa cho phép (Fail-safe timeout tính bằng milliseconds).
        // Nếu quá thời gian này không nhận được gói tin từ PC -> Tự động Disarm tắt động cơ ngay lập tức!
        constexpr uint32_t FAILSAFE_TIMEOUT_MS   = 350; 
    }

    // =====================================================================================
    // 2. CHU KỲ VÒNG LẶP ĐIỀU KHIỂN THỜI GIAN THỰC (RTOS CONTROL TIMING)
    // =====================================================================================
    namespace Timing {
        // Tần số vòng lặp điều khiển bay (Hz). Mặc định 200Hz.
        constexpr float    CONTROL_LOOP_HZ       = 200.0f;

        // Chu kỳ lấy mẫu tương ứng dt = 1 / CONTROL_LOOP_HZ (giây). 200Hz -> dt = 0.005s (5ms)
        constexpr float    DT                    = 1.0f / CONTROL_LOOP_HZ;

        // Chu kỳ tính bằng Tick FreeRTOS (5ms)
        constexpr uint32_t LOOP_PERIOD_TICKS_MS  = static_cast<uint32_t>(DT * 1000.0f);
    }

    // =====================================================================================
    // 3. BỘ ĐIỀU KHIỂN GÓC VÀ VẬN TỐC GÓC (ATTITUDE & RATE CONTROLLER)
    //    Cấu trúc: P-D Damping (Lấy trực tiếp p, q, r từ Gyro để triệt tiêu hiện tượng dội vi phân)
    // =====================================================================================
    namespace Attitude {
        // Hệ số khuếch đại tỷ lệ Kp cho góc Roll và Pitch (Độ nhạy góc)
        // Ý nghĩa: Sai lệch góc càng lớn thì mô-men sinh ra càng mạnh để kéo drone về vị trí phẳng.
        // Cách chỉnh:
        //   - Nếu drone lờ đờ, phản ứng chậm khi đổi hướng: TĂNG Kp (thử 0.50 -> 0.60).
        //   - Nếu drone bị rung lắc tần số cao, giật liên hồi: GIẢM Kp (thử 0.35 -> 0.40).
        constexpr float KP_ATT                   = 0.45f;

        // Hệ số tích phân Ki cho góc Roll và Pitch (Khử sai số tĩnh)
        // Ý nghĩa: Bù trừ trọng tâm drone bị lệch (pin lệch hoặc gió tạt làm nghiêng nhẹ).
        // Cách chỉnh: Giữ nhỏ (0.02 - 0.08) để tránh gây tích lũy làm trôi góc.
        constexpr float KI_ATT                   = 0.05f;

        // Giới hạn chống bão hòa tích phân (Anti-windup clamp) cho Ki góc (rad)
        constexpr float INT_ATT_LIMIT            = 0.05f;

        // Hệ số giảm chấn vi phân Kd cho Roll và Pitch (Gyro Rate Damping)
        // Ý nghĩa: Dập tắt vận tốc góc (p, q), hoạt động như một "bộ giảm xóc ảo" cản trở sự quay.
        // Cách chỉnh:
        //   - Nếu drone vừa đổi góc xong bị lắc lư vài nhịp mới đứng yên (vọt lố): TĂNG Kd (0.10 -> 0.12).
        //   - Nếu động cơ kêu rít gắt, nóng động cơ hoặc rung giật: GIẢM Kd (0.05 -> 0.07).
        constexpr float KD_ATT                   = 0.08f;

        // Hệ số tỷ lệ Kp cho trục Yaw (Hướng mũi drone)
        // Ý nghĩa: Duy trì mũi drone theo góc đặt (Target Yaw).
        constexpr float KP_YAW                   = 0.30f;

        // Hệ số giảm chấn vi phân Kd cho trục Yaw (Damping con quay hồi chuyển r)
        // Ý nghĩa: Ngăn hiện tượng văng đuôi hoặc xoay vòng không kiểm soát khi phanh.
        constexpr float KD_YAW                   = 0.15f;

        // Giới hạn mô-men điều khiển tối đa ngõ ra (Torque limits - đơn vị chuẩn hóa)
        // Ngăn bộ điều khiển yêu cầu lực xoắn vượt ngưỡng quá lớn làm triệt tiêu lực nâng.
        constexpr float MAX_TAU_ROLL             = 0.25f; // Mô-men Roll cực đại
        constexpr float MAX_TAU_PITCH            = 0.25f; // Mô-men Pitch cực đại
        constexpr float MAX_TAU_YAW              = 0.20f; // Mô-men Yaw cực đại
    }

    // =====================================================================================
    // 4. BỘ ĐIỀU KHIỂN ĐỘ CAO (ALTITUDE CONTROLLER)
    // =====================================================================================
    namespace Altitude {
        // Tỉ lệ lực nâng cơ sở để treo lơ lửng (Hover Thrust: tỉ lệ 0.0 -> 1.0)
        // Ý nghĩa: Tại điểm cân bằng trọng lực (T = mg), mỗi motor cần chạy ở mức ga này.
        // Cách chỉnh:
        //   - Nếu drone cất cánh xong bị tụt độ cao dần: TĂNG giá trị này (0.60 -> 0.65).
        //   - Nếu drone cứ vọt lên trần nhà không chịu hạ: GIẢM giá trị này (0.50 -> 0.55).
        constexpr float HOVER_THRUST             = 0.59f;

        // Hệ số Kp sai số độ cao Z (Độ nhạy độ cao)
        // Ý nghĩa: Kéo drone về độ cao mục tiêu.
        constexpr float KP_ALT                   = 0.35f;

        // Hệ số Ki độ cao Z (Khử sai số tĩnh khi điện áp pin sụt giảm)
        constexpr float KI_ALT                   = 0.08f;

        // Giới hạn tích phân chống bão hòa (Anti-windup) cho độ cao
        constexpr float INT_ALT_LIMIT            = 0.15f;

        // Hệ số giảm chấn vận tốc đứng (Vertical Velocity Damping kv)
        // Ý nghĩa: Đóng vai trò cực kỳ quan trọng! Tạo lực cản ảo tỉ lệ với vận tốc vz.
        //          Ngăn drone lao lên quá nhanh hoặc rơi tự do khi đổi độ cao.
        // Cách chỉnh: Nếu drone nhấp nhô nhảy cóc theo chiều thẳng đứng -> TĂNG Kv (0.35 -> 0.45).
        constexpr float KV_DAMPING               = 0.32f;

        // Giới hạn lực đẩy tổng cực tiểu và cực đại (Thrust limits)
        constexpr float MIN_TOTAL_THRUST         = 0.10f; // Tránh tắt ngắt động cơ trên không
        constexpr float MAX_TOTAL_THRUST         = 0.85f; // Dự phòng 15% biên công suất để bẻ lái cân bằng

        // Giới hạn góc bù nghiêng (Cos tilt protection)
        // Khi drone nghiêng góc theta, lực nâng thẳng đứng giảm đi cos(theta).
        // Ta chia cho cos(theta) để bù lực, nhưng chặn tối thiểu 0.65 (~49 độ) để chống bão hòa động cơ.
        constexpr float MIN_COS_TILT             = 0.65f;
    }

    // =====================================================================================
    // 5. BỘ ĐIỀU KHIỂN VỊ TRÍ 2D & PHANH CHỦ ĐỘNG (POSITION & VELOCITY CONTROLLER)
    // =====================================================================================
    namespace Position {
        // Hệ số P điều khiển vị trí: Chuyển khoảng cách sai lệch (m) thành vận tốc mục tiêu (m/s)
        constexpr float KP_POS                   = 1.8f;

        // Vận tốc tiếp cận tối đa khi tự động bay về điểm neo Hover (m/s)
        constexpr float MAX_APPROACH_VEL         = 1.2f;

        // Hệ số P vận tốc: Chuyển sai lệch vận tốc (m/s) thành gia tốc góc nghiêng mong muốn
        // Cách chỉnh: Tăng sẽ giúp drone phanh gấp hơn, giảm sẽ giúp drone lướt mượt hơn.
        constexpr float KP_VEL                   = 2.5f;

        // Hệ số tích phân triệt tiêu gió ngang (Ki Velocity / Wind Rejection)
        constexpr float KI_VEL                   = 0.40f;

        // Gia tốc gió cực đại bù trừ (m/s^2)
        constexpr float MAX_WIND_ACCEL           = 1.5f;

        // Giới hạn gia tốc lệnh người lái (m/s^2) - Giúp drone tăng tốc mượt mà, không giật ga
        constexpr float MAX_CMD_ACCEL            = 3.5f;

        // Ngưỡng vận tốc coi như đã dừng hẳn để chuyển từ phanh (BRAKING) sang giữ điểm (HOLD) (m/s)
        constexpr float STOP_VEL_THRESHOLD       = 0.08f;

        // Góc nghiêng thân cực đại khi người lái điều khiển bay tiến/lùi/trái/phải (độ)
        // Cách chỉnh: Nếu thầy bảo "Bay chậm thôi cho an toàn" -> giảm về 8.0f - 10.0f.
        //             Nếu thầy bảo "Tăng tốc độ lướt gió tối đa" -> tăng lên 15.0f - 20.0f.
        constexpr float MAX_TILT_DRIVING_DEG     = 12.0f;

        // Góc ngả ngược tối đa khi kích hoạt PHANH CHỦ ĐỘNG (Braking tilt) (độ)
        // Thường lớn hơn góc lái bình thường một chút để ghìm drone dừng lại nhanh nhất.
        constexpr float MAX_TILT_BRAKING_DEG     = 16.0f;

        // Tốc độ thay đổi góc nghiêng tối đa (deg/s) - Bộ lọc Rate Limiter chống lật thân
        constexpr float MAX_TILT_RATE_DEG_S      = 180.0f;
    }

    // =====================================================================================
    // 6. QUẢN LÝ TRẠNG THÁI BAY (STATE MACHINE & LOGIC AN TOÀN)
    // =====================================================================================
    namespace FlightLogic {
        // Tốc độ leo thẳng đứng khi cất cánh tự động Takeoff (m/s). 
        // 0.004m mỗi chu kỳ 5ms = 0.8 m/s (chống sốc áp lực mặt đất Ground Effect)
        constexpr float TAKEOFF_STEP_PER_TICK    = 0.004f;

        // Độ cao mặc định khi nhấn lệnh Takeoff (m)
        constexpr float TAKEOFF_TARGET_ALT_M     = 2.0f;

        // Tốc độ hạ cánh tự động Land (m/s). 
        // 0.003m mỗi chu kỳ 5ms = 0.6 m/s (tiếp đất êm ái)
        constexpr float LAND_STEP_PER_TICK       = 0.003f;

        // Độ cao ngưỡng xác định drone đã chạm đất -> Ngắt động cơ an toàn (m)
        constexpr float TOUCHDOWN_ALT_THRESHOLD  = 0.10f;

        // Độ cao tối thiểu để kích hoạt nghiêng thân điều khiển vị trí X/Y (m)
        // Khi z < 0.25m (sát mặt đất), khóa phẳng Roll=0, Pitch=0 để 4 chân nhấc đều, không bị quệt đất!
        constexpr float MIN_TAKEOFF_ALT_FOR_TILT = 0.25f;

        // Giới hạn trần bay tối đa và tối thiểu cho phép (m)
        constexpr float MIN_FLIGHT_ALT_M         = 0.30f;
        constexpr float MAX_FLIGHT_ALT_M         = 10.0f;
    }

    // =====================================================================================
    // 7. BỘ TRỘN ĐỘNG CƠ & HÌNH HỌC KHUNG CHỮ X (MOTOR MIXER & GEOMETRY)
    // =====================================================================================
    namespace Mixer {
        // Vận tốc góc cực đại của động cơ BLDC (rad/s)
        // Giá trị này được nhân với căn bậc 2 của tỉ lệ điều khiển u_i: w_i = max_rpm * sqrt(u_i)
        constexpr float MAX_MOTOR_ROT_VELOCITY   = 1000.0f;

        // Ngưỡng lực nâng tối thiểu để bắt đầu quay cánh quạt (tránh quay khi đang disarm)
        constexpr float MIN_THRUST_THRESHOLD     = 0.02f;
    }
}
