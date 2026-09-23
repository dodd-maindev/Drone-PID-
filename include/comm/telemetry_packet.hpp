#pragma once
#include <stdint.h>
#include <stddef.h>

#pragma pack(push, 1)

/**
 * @brief Cờ điều khiển chuyến bay từ người lái (PC gửi sang ESP32)
 */
enum ControlFlagBits : uint8_t {
    CTRL_FLAG_ARMED       = 1 << 0, // Đã Arm động cơ
    CTRL_FLAG_DRIVING_X   = 1 << 1, // Đang ấn phím Tiến / Lùi (Pitch)
    CTRL_FLAG_DRIVING_Y   = 1 << 2, // Đang ấn phím Trái / Phải (Roll)
    CTRL_FLAG_ALT_ACTIVE  = 1 << 3, // Đang ấn phím Lên / Xuống (W / S)
    CTRL_FLAG_EMERGENCY   = 1 << 4, // Phanh khẩn cấp / Disarm
    CTRL_FLAG_TAKEOFF     = 1 << 5, // Lệnh cất cánh tự động
    CTRL_FLAG_LAND        = 1 << 6  // Lệnh hạ cánh an toàn
};

/**
 * @brief Gói tin cảm biến và lệnh lái từ PC sang ESP32 (SensorPacket)
 * Kích thước: 2 + 12 + 12 + 12 + 12 + 16 + 1 + 1 = 68 bytes
 */
struct SensorPacket {
    uint8_t header[2];      // Đồng bộ: 0xAA, 0x55

    // 1. Cảm biến Odometry từ Gazebo
    float x;                // Tọa độ X thế giới (m)
    float y;                // Tọa độ Y thế giới (m)
    float z;                // Độ cao Z thế giới (m)

    float vx;               // Vận tốc thân drone trục X (m/s)
    float vy;               // Vận tốc thân drone trục Y (m/s)
    float vz;               // Vận tốc thẳng đứng trục Z (m/s)

    float roll;             // Góc nghiêng Roll (rad)
    float pitch;            // Góc nghiêng Pitch (rad)
    float yaw;              // Góc hướng mũi Yaw (rad)

    float p;                // Vận tốc góc xoay Roll (rad/s)
    float q;                // Vận tốc góc xoay Pitch (rad/s)
    float r;                // Vận tốc góc xoay Yaw (rad/s)

    // 2. Lệnh điều khiển từ phi công / bàn phím PC
    float cmd_vx;           // Vận tốc tiến/lùi mong muốn (m/s)
    float cmd_vy;           // Vận tốc trái/phải mong muốn (m/s)
    float cmd_alt_vel;      // Vận tốc leo/hạ mong muốn (m/s)
    float cmd_yaw_rate;     // Tốc độ xoay đầu mong muốn (rad/s)

    uint8_t flags;          // Cờ điều khiển (ControlFlagBits)
    uint8_t checksum;       // Mã kiểm lỗi XOR toàn bộ gói
};

/**
 * @brief Gói tin phản hồi tốc độ động cơ từ ESP32 sang PC (ActuatorPacket)
 * Kích thước: 2 + 16 + 8 + 1 + 1 = 28 bytes
 */
struct ActuatorPacket {
    uint8_t header[2];      // Đồng bộ: 0x55, 0xAA

    // 1. Vận tốc góc của 4 cánh quạt (rad/s) để cấp cho Gazebo Sim
    float w0;               // Động cơ 0 (Front-Right, CCW)
    float w1;               // Động cơ 1 (Rear-Left, CCW)
    float w2;               // Động cơ 2 (Front-Left, CW)
    float w3;               // Động cơ 3 (Rear-Right, CW)

    // 2. Dữ liệu giám sát trạng thái thuật toán từ ESP32 (Telemetry HUD)
    float target_pitch_deg; // Góc Pitch mục tiêu do PositionController tính ra
    float target_roll_deg;  // Góc Roll mục tiêu do PositionController tính ra

    uint8_t status_flags;   // Bit 0: Is Armed, Bit 1: Is Braking, Bit 2: Is Hover
    uint8_t checksum;       // Mã kiểm lỗi XOR
};

#pragma pack(pop)

/**
 * @brief Hàm tính mã kiểm lỗi XOR Checksum cho gói tin
 */
inline uint8_t compute_packet_checksum(const uint8_t* data, size_t len) {
    uint8_t cs = 0;
    for (size_t i = 0; i < len; ++i) {
        cs ^= data[i];
    }
    return cs;
}
