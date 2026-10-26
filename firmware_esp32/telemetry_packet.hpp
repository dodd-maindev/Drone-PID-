#pragma once
#include <stdint.h>
#include <stddef.h>

#pragma pack(push, 1)

enum ControlFlagBits : uint8_t {
    CTRL_FLAG_ARMED       = 1 << 0,
    CTRL_FLAG_DRIVING_X   = 1 << 1,
    CTRL_FLAG_DRIVING_Y   = 1 << 2,
    CTRL_FLAG_ALT_ACTIVE  = 1 << 3,
    CTRL_FLAG_EMERGENCY   = 1 << 4,
    CTRL_FLAG_TAKEOFF     = 1 << 5,
    CTRL_FLAG_LAND        = 1 << 6
};

struct SensorPacket {
    uint8_t header[2];      // 0xAA, 0x55

    float x;
    float y;
    float z;

    float vx;
    float vy;
    float vz;

    float roll;
    float pitch;
    float yaw;

    float p;
    float q;
    float r;

    float cmd_vx;
    float cmd_vy;
    float cmd_alt_vel;
    float cmd_yaw_rate;

    uint8_t flags;
    uint8_t checksum;
};

struct ActuatorPacket {
    uint8_t header[2];      // 0x55, 0xAA

    float w0;
    float w1;
    float w2;
    float w3;

    float target_pitch_deg;
    float target_roll_deg;

    uint8_t status_flags;
    uint8_t checksum;
};

#pragma pack(pop)

inline uint8_t compute_packet_checksum(const uint8_t* data, size_t len) {
    uint8_t cs = 0;
    for (size_t i = 0; i < len; ++i) {
        cs ^= data[i];
    }
    return cs;
}
