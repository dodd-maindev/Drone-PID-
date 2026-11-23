#pragma once

#include <mutex>
#include <cmath>
#include <cstdint>

namespace DroneEstimation {

struct EstimatedState {
    float x{0.0f};        // World ENU X (East), m
    float y{0.0f};        // World ENU Y (North), m
    float z{0.0f};        // World ENU Z (Up), m

    float vx_body{0.0f};  // Body-frame linear velocity X (Forward), m/s
    float vy_body{0.0f};  // Body-frame linear velocity Y (Left), m/s
    float vz_world{0.0f}; // World-frame vertical velocity Z (Up), m/s

    float vx_world{0.0f}; // World ENU velocity X, m/s
    float vy_world{0.0f}; // World ENU velocity Y, m/s

    float roll{0.0f};     // rad
    float pitch{0.0f};    // rad
    float yaw{0.0f};      // rad

    float p{0.0f};        // Body roll rate, rad/s
    float q{0.0f};        // Body pitch rate, rad/s
    float r{0.0f};        // Body yaw rate, rad/s

    bool imu_ready{false};
    bool gps_ready{false};
    bool baro_ready{false};
    bool mag_ready{false};

    bool is_healthy() const {
        return imu_ready && gps_ready && baro_ready && mag_ready;
    }
};

/**
 * @brief Bộ ước lượng trạng thái EKF Sensor Fusion chuẩn PX4 (IMU + GPS + Barometer + Magnetometer)
 * 
 * Thay thế hoàn toàn việc đọc trực tiếp tọa độ thế giới (Ground Truth Odometry) từ Gazebo:
 * - Prediction (250Hz): Tích phân IMU Gyro (góc) & IMU Accelerometer (vị trí, vận tốc).
 * - Correction:
 *   + Accelerometer + Magnetometer: Khử trôi góc Roll/Pitch/Yaw.
 *   + GPS / NavSat (30Hz): Hiệu chỉnh vị trí ngang (x, y) và vận tốc ngang (vx, vy) qua Kalman 2D.
 *   + Barometer (50Hz) + GPS Vel Up: Hiệu chỉnh độ cao (z) và vận tốc đứng (vz) qua Kalman 1D.
 */
class StateEstimator {
public:
    StateEstimator() = default;

    // Đặt tọa độ tham chiếu GPS gốc của thế giới (khớp với <spherical_coordinates> trong drone_world.sdf)
    void set_gps_origin(double lat_deg, double lon_deg, double alt_m);

    // Đặt vector từ trường tham chiếu thế giới (khớp với <magnetic_field> trong drone_world.sdf, ENU)
    void set_mag_reference(float bx, float by, float bz);

    // Cập nhật từ cảm biến IMU (250Hz) - Body frame
    void on_imu(double timestamp_s,
                float gx, float gy, float gz,
                float ax, float ay, float az);

    // Cập nhật từ cảm biến GPS / NavSat (30Hz)
    void on_gps(double lat_deg, double lon_deg, double alt_m,
                float vel_east, float vel_north, float vel_up);

    // Cập nhật từ cảm biến Áp suất khí quyển / Barometer (50Hz)
    void on_baro(float pressure_pa);

    // Cập nhật từ cảm biến Từ kế / Magnetometer (100Hz) - Body frame
    void on_mag(float mx, float my, float mz);

    EstimatedState get_state() const;

private:
    static float wrap_pi(float angle);

    // Chuyển đổi kinh/vĩ độ sang tọa độ phẳng ENU (m) so với gốc
    void gps_to_enu(double lat_deg, double lon_deg, float& out_x, float& out_y) const;

    // Cập nhật bộ lọc Kalman 2-state [pos, vel] cho từng trục độc lập
    static void kalman_predict_axis(float& pos, float& vel, float P[2][2],
                                    float accel, float dt,
                                    float q_pos, float q_vel);
    static void kalman_update_pos(float& pos, float& vel, float P[2][2],
                                  float meas_pos, float r_pos);
    static void kalman_update_vel(float& pos, float& vel, float P[2][2],
                                  float meas_vel, float r_vel);

    mutable std::mutex mutex_;

    // Trạng thái góc (Attitude)
    float roll_{0.0f};
    float pitch_{0.0f};
    float yaw_{0.0f};
    float p_{0.0f}, q_{0.0f}, r_{0.0f};

    // Gia tốc trọng trường & tham số thế giới ENU
    static constexpr float GRAVITY = 9.80f;
    float mag_ref_x_{6e-6f};
    float mag_ref_y_{2.3e-5f};
    float mag_ref_z_{-4.2e-5f};
    float mag_ref_yaw_{1.3155f}; // atan2(2.3e-5, 6e-6)

    // Gốc tọa độ GPS
    bool gps_origin_set_{false};
    double origin_lat_{47.397971057728974};
    double origin_lon_{8.546163739800146};
    double origin_alt_{0.0};
    double meters_per_deg_lat_{111320.0};
    double meters_per_deg_lon_{111320.0 * std::cos(47.397971057728974 * M_PI / 180.0)};

    // Trạng thái vị trí & vận tốc trong World ENU
    float x_{0.0f}, vx_{0.0f};
    float y_{0.0f}, vy_{0.0f};
    float z_{0.12f}, vz_{0.0f};

    // Ma trận hiệp phương sai P cho 3 trục X, Y, Z
    float Px_[2][2] = {{1.0f, 0.0f}, {0.0f, 1.0f}};
    float Py_[2][2] = {{1.0f, 0.0f}, {0.0f, 1.0f}};
    float Pz_[2][2] = {{1.0f, 0.0f}, {0.0f, 1.0f}};

    // Hiệu chuẩn từ kế lúc khởi động (mặt đất)
    bool mag_calibrated_{false};
    double mag_hx_sum_{0.0};
    double mag_hy_sum_{0.0};
    int mag_calib_count_{0};

    // Hiệu chuẩn áp suất mặt đất cho Barometer
    bool baro_calibrated_{false};
    float baro_p0_{101325.0f};
    double baro_p0_sum_{0.0};
    int baro_calib_count_{0};

    // Thời gian IMU trước đó
    bool imu_time_init_{false};
    double last_imu_time_{0.0};

    // Cờ trạng thái cảm biến
    bool imu_ready_{false};
    bool gps_ready_{false};
    bool baro_ready_{false};
    bool mag_ready_{false};
    bool yaw_initialized_from_mag_{false};
};

} // namespace DroneEstimation
