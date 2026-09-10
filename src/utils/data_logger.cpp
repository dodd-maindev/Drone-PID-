#include "utils/data_logger.hpp"
#include <iostream>
#include <ctime>
#include <sstream>

namespace DroneUtils {

DataLogger::DataLogger() = default;

DataLogger::~DataLogger() {
    stop();
}

bool DataLogger::start(const std::string& log_dir, const std::string& prefix) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (is_logging_) {
        return true;
    }

    try {
        std::filesystem::create_directories(log_dir);

        // Sinh tên file theo thời gian thực YYYYMMDD_HHMMSS
        auto now = std::chrono::system_clock::now();
        std::time_t now_c = std::chrono::system_clock::to_time_t(now);
        std::tm tm_now{};
        localtime_r(&now_c, &tm_now);

        std::ostringstream filename_oss;
        filename_oss << prefix << "_"
                     << std::put_time(&tm_now, "%Y%m%d_%H%M%S")
                     << ".csv";

        current_log_path_ = log_dir + "/" + filename_oss.str();
        latest_log_path_ = log_dir + "/latest_flight.csv";

        file_.open(current_log_path_, std::ios::out | std::ios::trunc);
        if (!file_.is_open()) {
            std::cerr << "[DataLogger] [!] Không thể tạo file log tại: " << current_log_path_ << std::endl;
            return false;
        }

        // Ghi tiêu đề các cột dữ liệu
        file_ << "time_s,"
              << "target_thrust,"
              << "target_roll_deg,target_pitch_deg,target_yaw_deg,"
              << "actual_alt_m,actual_vz_mps,"
              << "actual_roll_deg,actual_pitch_deg,actual_yaw_deg,"
              << "rate_p_rad_s,rate_q_rad_s,rate_r_rad_s,"
              << "tau_roll,tau_pitch,tau_yaw,"
              << "w0_rad_s,w1_rad_s,w2_rad_s,w3_rad_s\n";

        is_logging_ = true;
        records_count_ = 0;

        std::cout << "[DataLogger] [✓] Bắt đầu ghi log bay tại: " << current_log_path_ << std::endl;
        return true;

    } catch (const std::exception& e) {
        std::cerr << "[DataLogger] [!] Ngoại lệ khi tạo file log: " << e.what() << std::endl;
        return false;
    }
}

void DataLogger::log(const FlightLogEntry& entry) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!is_logging_ || !file_.is_open()) {
        return;
    }

    file_ << std::fixed << std::setprecision(4)
          << entry.time_s << ","
          << entry.target_thrust << ","
          << entry.target_roll_deg << ","
          << entry.target_pitch_deg << ","
          << entry.target_yaw_deg << ","
          << entry.actual_alt << ","
          << entry.actual_vz << ","
          << entry.actual_roll_deg << ","
          << entry.actual_pitch_deg << ","
          << entry.actual_yaw_deg << ","
          << entry.rate_p << ","
          << entry.rate_q << ","
          << entry.rate_r << ","
          << entry.tau_roll << ","
          << entry.tau_pitch << ","
          << entry.tau_yaw << ","
          << entry.w0 << ","
          << entry.w1 << ","
          << entry.w2 << ","
          << entry.w3 << "\n";

    records_count_++;
}

void DataLogger::stop() {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!is_logging_) {
        return;
    }

    if (file_.is_open()) {
        file_.flush();
        file_.close();
    }
    is_logging_ = false;

    // Sao chép sang latest_flight.csv để người dùng luôn có file mới nhất
    try {
        std::filesystem::copy_file(
            current_log_path_, latest_log_path_,
            std::filesystem::copy_options::overwrite_existing
        );
    } catch (...) {
        // Bỏ qua nếu không thể copy
    }

    std::cout << "\n[DataLogger] [✓] Đã lưu tổng cộng " << records_count_ << " mẫu log vào:" << std::endl;
    std::cout << "  -> File chi tiết: " << current_log_path_ << std::endl;
    std::cout << "  -> File mới nhất: " << latest_log_path_ << std::endl;
}

} // namespace DroneUtils
