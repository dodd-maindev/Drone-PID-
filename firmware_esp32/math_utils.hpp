#pragma once
#include <cmath>
#include <array>
#include <algorithm>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace DroneMath {

inline std::array<float, 4> euler_to_quaternion(float roll, float pitch, float yaw) {
    float cy = std::cos(yaw * 0.5f);
    float sy = std::sin(yaw * 0.5f);
    float cp = std::cos(pitch * 0.5f);
    float sp = std::sin(pitch * 0.5f);
    float cr = std::cos(roll * 0.5f);
    float sr = std::sin(roll * 0.5f);

    float w = cr * cp * cy + sr * sp * sy;
    float x = sr * cp * cy - cr * sp * sy;
    float y = cr * sp * cy + sr * cp * sy;
    float z = cr * cp * sy - sr * sp * cy;

    return {w, x, y, z};
}

inline void quaternion_to_euler(float w, float x, float y, float z, float& roll, float& pitch, float& yaw) {
    float sinr_cosp = 2.0f * (w * x + y * z);
    float cosr_cosp = 1.0f - 2.0f * (x * x + y * y);
    roll = std::atan2(sinr_cosp, cosr_cosp);

    float sinp = 2.0f * (w * y - z * x);
    if (std::abs(sinp) >= 1.0f) {
        pitch = std::copysign(static_cast<float>(M_PI / 2.0), sinp);
    } else {
        pitch = std::asin(sinp);
    }

    float siny_cosp = 2.0f * (w * z + x * y);
    float cosy_cosp = 1.0f - 2.0f * (y * y + z * z);
    yaw = std::atan2(siny_cosp, cosy_cosp);
}

inline float normalize_angle(float angle) {
    while (angle > M_PI) angle -= 2.0f * M_PI;
    while (angle < -M_PI) angle += 2.0f * M_PI;
    return angle;
}

inline float deg2rad(float deg) {
    return deg * static_cast<float>(M_PI / 180.0);
}

inline float rad2deg(float rad) {
    return rad * static_cast<float>(180.0 / M_PI);
}

template<typename T>
inline T clamp(T val, T min_val, T max_val) {
    return std::max(min_val, std::min(val, max_val));
}

} // namespace DroneMath
