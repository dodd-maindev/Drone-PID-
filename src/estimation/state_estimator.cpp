#include "estimation/state_estimator.hpp"
#include <algorithm>

namespace DroneEstimation {

float StateEstimator::wrap_pi(float angle) {
    while (angle > static_cast<float>(M_PI))  angle -= 2.0f * static_cast<float>(M_PI);
    while (angle < -static_cast<float>(M_PI)) angle += 2.0f * static_cast<float>(M_PI);
    return angle;
}

void StateEstimator::set_gps_origin(double lat_deg, double lon_deg, double alt_m) {
    std::lock_guard<std::mutex> lock(mutex_);
    origin_lat_ = lat_deg;
    origin_lon_ = lon_deg;
    origin_alt_ = alt_m;
    meters_per_deg_lat_ = 111320.0;
    meters_per_deg_lon_ = 111320.0 * std::cos(origin_lat_ * M_PI / 180.0);
    gps_origin_set_ = true;
}

void StateEstimator::set_mag_reference(float bx, float by, float bz) {
    std::lock_guard<std::mutex> lock(mutex_);
    mag_ref_x_ = bx;
    mag_ref_y_ = by;
    mag_ref_z_ = bz;
    mag_ref_yaw_ = std::atan2(by, bx);
}

void StateEstimator::gps_to_enu(double lat_deg, double lon_deg, float& out_x, float& out_y) const {
    // Trong ENU: X = East (Longitude), Y = North (Latitude)
    out_x = static_cast<float>((lon_deg - origin_lon_) * meters_per_deg_lon_);
    out_y = static_cast<float>((lat_deg - origin_lat_) * meters_per_deg_lat_);
}

void StateEstimator::kalman_predict_axis(float& pos, float& vel, float P[2][2],
                                         float accel, float dt,
                                         float q_pos, float q_vel) {
    pos += vel * dt + 0.5f * accel * dt * dt;
    vel += accel * dt;

    // F = [1 dt; 0 1]
    float p00 = P[0][0] + dt * (P[1][0] + P[0][1]) + dt * dt * P[1][1] + q_pos * dt;
    float p01 = P[0][1] + dt * P[1][1];
    float p10 = P[1][0] + dt * P[1][1];
    float p11 = P[1][1] + q_vel * dt;

    P[0][0] = p00; P[0][1] = p01;
    P[1][0] = p10; P[1][1] = p11;
}

void StateEstimator::kalman_update_pos(float& pos, float& vel, float P[2][2],
                                       float meas_pos, float r_pos) {
    // H = [1 0]
    float y = meas_pos - pos;
    float S = P[0][0] + r_pos;
    if (S < 1e-6f) return;

    float k0 = P[0][0] / S;
    float k1 = P[1][0] / S;

    pos += k0 * y;
    vel += k1 * y;

    float p00 = P[0][0];
    float p01 = P[0][1];
    P[0][0] -= k0 * p00;
    P[0][1] -= k0 * p01;
    P[1][0] -= k1 * p00;
    P[1][1] -= k1 * p01;
}

void StateEstimator::kalman_update_vel(float& pos, float& vel, float P[2][2],
                                       float meas_vel, float r_vel) {
    // H = [0 1]
    float y = meas_vel - vel;
    float S = P[1][1] + r_vel;
    if (S < 1e-6f) return;

    float k0 = P[0][1] / S;
    float k1 = P[1][1] / S;

    pos += k0 * y;
    vel += k1 * y;

    float p10 = P[1][0];
    float p11 = P[1][1];
    P[0][0] -= k0 * p10;
    P[0][1] -= k0 * p11;
    P[1][0] -= k1 * p10;
    P[1][1] -= k1 * p11;
}

void StateEstimator::on_imu(double timestamp_s,
                            float gx, float gy, float gz,
                            float ax, float ay, float az) {
    std::lock_guard<std::mutex> lock(mutex_);

    float dt = 0.004f;
    if (imu_time_init_) {
        double d = timestamp_s - last_imu_time_;
        if (d > 0.0005 && d < 0.1) {
            dt = static_cast<float>(d);
        }
    }
    last_imu_time_ = timestamp_s;
    imu_time_init_ = true;

    p_ = gx;
    q_ = gy;
    r_ = gz;

    // 1. Attitude Prediction từ Gyro rates (ZYX Euler kinematics)
    float sin_phi = std::sin(roll_);
    float cos_phi = std::cos(roll_);
    float cos_theta = std::cos(pitch_);
    if (std::abs(cos_theta) < 0.05f) cos_theta = (cos_theta >= 0.0f) ? 0.05f : -0.05f;
    float tan_theta = std::sin(pitch_) / cos_theta;

    float d_roll  = gx + gy * sin_phi * tan_theta + gz * cos_phi * tan_theta;
    float d_pitch = gy * cos_phi - gz * sin_phi;
    float d_yaw   = (gy * sin_phi + gz * cos_phi) / cos_theta;

    roll_  = wrap_pi(roll_  + d_roll  * dt);
    pitch_ = std::clamp(pitch_ + d_pitch * dt, -1.35f, 1.35f);
    yaw_   = wrap_pi(yaw_   + d_yaw   * dt);

    // 2. Attitude Correction từ Accelerometer (Complementary Filter với trọng số thích nghi theo gia tốc)
    float acc_norm = std::sqrt(ax * ax + ay * ay + az * az);
    if (acc_norm > 0.5f) {
        float acc_roll  = std::atan2(ay, az);
        float acc_pitch = std::atan2(-ax, std::sqrt(ay * ay + az * az));

        // Chỉ tin tưởng gia tốc kế nhẹ nhàng để khử trôi Gyro (tau ~ 2.5s @ 250Hz -> alpha = 0.0016)
        // Tránh alpha quá lớn làm gia tốc kế kéo sai góc nghiêng thật khi drone đang bay nghiêng!
        float accel_err = std::abs(acc_norm - GRAVITY);
        float alpha = (accel_err < 0.4f) ? 0.0016f : 0.0002f;

        roll_  = wrap_pi(roll_  + alpha * wrap_pi(acc_roll  - roll_));
        pitch_ = std::clamp(pitch_ + alpha * (acc_pitch - pitch_), -1.35f, 1.35f);
    }

    // 3. Chuyển gia tốc đo Body -> World ENU và trừ trọng lực để dự đoán vị trí & vận tốc
    float sr = std::sin(roll_),  cr = std::cos(roll_);
    float sp = std::sin(pitch_), cp = std::cos(pitch_);
    float sy = std::sin(yaw_),   cy = std::cos(yaw_);

    // R = Rz(yaw) * Ry(pitch) * Rx(roll)
    float r00 = cy * cp;
    float r01 = cy * sp * sr - sy * cr;
    float r02 = cy * sp * cr + sy * sr;

    float r10 = sy * cp;
    float r11 = sy * sp * sr + cy * cr;
    float r12 = sy * sp * cr - cy * sr;

    float r20 = -sp;
    float r21 = cp * sr;
    float r22 = cp * cr;

    float ax_w = r00 * ax + r01 * ay + r02 * az;
    float ay_w = r10 * ax + r11 * ay + r12 * az;
    float az_w = (r20 * ax + r21 * ay + r22 * az) - GRAVITY;

    // EKF Predict cho 3 trục
    kalman_predict_axis(x_, vx_, Px_, ax_w, dt, 0.001f, 0.15f);
    kalman_predict_axis(y_, vy_, Py_, ay_w, dt, 0.001f, 0.15f);
    kalman_predict_axis(z_, vz_, Pz_, az_w, dt, 0.001f, 0.20f);

    imu_ready_ = true;
}

void StateEstimator::on_mag(float mx, float my, float mz) {
    std::lock_guard<std::mutex> lock(mutex_);

    // Bù nghiêng (Tilt compensation): đưa vector từ trường Body về mặt phẳng ngang (World-level)
    float sr = std::sin(roll_),  cr = std::cos(roll_);
    float sp = std::sin(pitch_), cp = std::cos(pitch_);

    // B_horiz = Ry(pitch) * Rx(roll) * B_body
    float hx = cp * mx + sp * sr * my + sp * cr * mz;
    float hy = cr * my - sr * mz;

    if (hx * hx + hy * hy < 1e-12f) return;

    // Tự động hiệu chuẩn hướng tham chiếu từ trường ban đầu từ 20 mẫu đầu trên mặt đất (yaw_init = 0)
    // Loại bỏ 100% sai lệch cố định giữa <magnetic_field> và hướng thực tế của Gazebo Magnetometer!
    if (!mag_calibrated_) {
        mag_hx_sum_ += static_cast<double>(hx);
        mag_hy_sum_ += static_cast<double>(hy);
        mag_calib_count_++;
        if (mag_calib_count_ >= 20) {
            mag_ref_yaw_ = static_cast<float>(std::atan2(mag_hy_sum_, mag_hx_sum_));
            mag_calibrated_ = true;
            yaw_ = 0.0f;
            yaw_initialized_from_mag_ = true;
            mag_ready_ = true;
        }
        return;
    }

    float yaw_meas = wrap_pi(mag_ref_yaw_ - std::atan2(hy, hx));
    if (!yaw_initialized_from_mag_) {
        yaw_ = yaw_meas;
        yaw_initialized_from_mag_ = true;
    } else {
        // Bù trôi Gyro Yaw nhẹ nhàng ở 100Hz (chống giật góc từ nhiễu từ kế)
        float yaw_err = std::clamp(wrap_pi(yaw_meas - yaw_), -0.35f, 0.35f);
        yaw_ = wrap_pi(yaw_ + 0.015f * yaw_err);
    }

    mag_ready_ = true;
}

void StateEstimator::on_gps(double lat_deg, double lon_deg, double alt_m,
                            float vel_east, float vel_north, float vel_up) {
    std::lock_guard<std::mutex> lock(mutex_);

    if (!gps_origin_set_) {
        origin_lat_ = lat_deg;
        origin_lon_ = lon_deg;
        origin_alt_ = alt_m;
        meters_per_deg_lat_ = 111320.0;
        meters_per_deg_lon_ = 111320.0 * std::cos(origin_lat_ * M_PI / 180.0);
        gps_origin_set_ = true;
    }

    float gx_enu = 0.0f, gy_enu = 0.0f;
    gps_to_enu(lat_deg, lon_deg, gx_enu, gy_enu);
    float gz_enu = static_cast<float>(alt_m - origin_alt_);

    if (!gps_ready_) {
        x_ = gx_enu;
        y_ = gy_enu;
        vx_ = vel_east;
        vy_ = vel_north;
        gps_ready_ = true;
        return;
    }

    // EKF Measurement Update từ GPS (r_pos, r_vel đặc trưng cho GPS/NavSat)
    kalman_update_pos(x_, vx_, Px_, gx_enu, 0.04f);
    kalman_update_pos(y_, vy_, Py_, gy_enu, 0.04f);
    kalman_update_vel(x_, vx_, Px_, vel_east,  0.02f);
    kalman_update_vel(y_, vy_, Py_, vel_north, 0.02f);

    // GPS altitude & vel_up hỗ trợ bổ sung nhẹ cho trục Z
    kalman_update_pos(z_, vz_, Pz_, gz_enu, 0.35f);
    kalman_update_vel(z_, vz_, Pz_, vel_up, 0.05f);
}

void StateEstimator::on_baro(float pressure_pa) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (pressure_pa <= 10000.0f) return;

    // Hiệu chuẩn áp suất tham chiếu mặt đất (P0) bằng 15 mẫu đầu tiên
    if (!baro_calibrated_) {
        baro_p0_sum_ += static_cast<double>(pressure_pa);
        baro_calib_count_++;
        if (baro_calib_count_ >= 15) {
            baro_p0_ = static_cast<float>(baro_p0_sum_ / static_cast<double>(baro_calib_count_));
            baro_calibrated_ = true;
            baro_ready_ = true;
        }
        return;
    }

    // Công thức khí áp chuẩn quốc tế (ICAO) + offset độ cao chân đáp x500 ban đầu (0.12m)
    float alt_baro = 44330.0f * (1.0f - std::pow(pressure_pa / baro_p0_, 1.0f / 5.255f)) + 0.12f;

    // EKF Measurement Update từ Barometer (50Hz)
    kalman_update_pos(z_, vz_, Pz_, alt_baro, 0.06f);
    baro_ready_ = true;
}

EstimatedState StateEstimator::get_state() const {
    std::lock_guard<std::mutex> lock(mutex_);
    EstimatedState st;
    st.x = x_;
    st.y = y_;
    st.z = z_;

    st.vx_world = vx_;
    st.vy_world = vy_;
    st.vz_world = vz_;

    // Chuyển đổi vận tốc World ENU -> Body-frame (Forward, Left) để tương thích 100% với PositionController ESP32
    float cy = std::cos(yaw_);
    float sy = std::sin(yaw_);
    st.vx_body =  cy * vx_ + sy * vy_;
    st.vy_body = -sy * vx_ + cy * vy_;

    st.roll  = roll_;
    st.pitch = pitch_;
    st.yaw   = yaw_;
    st.p = p_;
    st.q = q_;
    st.r = r_;

    st.imu_ready  = imu_ready_;
    st.gps_ready  = gps_ready_;
    st.baro_ready = baro_ready_;
    st.mag_ready  = mag_ready_;
    return st;
}

} // namespace DroneEstimation
