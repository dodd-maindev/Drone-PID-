#pragma once
#include <cmath>
#include <array>

namespace MathUtils {

/**
 * @brief Chuyển đổi góc Euler (roll, pitch, yaw theo radian) sang Quaternion [w, x, y, z]
 */
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

} // namespace MathUtils
