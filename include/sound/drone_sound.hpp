#pragma once

#include <string>

class DroneSoundManager {
public:
    enum SoundState {
        STATE_IDLE = 0,       // Dưới mặt đất / Ngắt động cơ -> Yên lặng
        STATE_STARTING = 1,   // Cất cánh từ mặt đất -> Phát start.mp3
        STATE_CONTINUE = 2,   // Bay tiếp diễn -> Lặp vô tận continue.mp3
        STATE_LANDING = 3     // Đang hạ cánh -> Âm lượng nhỏ dần (fade-out) cho tới khi tiếp đất tắt hẳn
    };

    DroneSoundManager();
    ~DroneSoundManager();

    // Khởi tạo audio engine và nạp các file mp3
    bool init(const std::string& audio_dir = "");

    // Cập nhật trạng thái âm thanh ở mỗi chu kỳ lặp (100Hz)
    // is_landing: cờ người dùng ra lệnh hạ cánh (nút A)
    void update(bool is_armed, bool is_landing, float altitude, float motor_speed_w0);

    // Dừng tất cả âm thanh
    void stop();

    // Dọn dẹp giải phóng tài nguyên
    void cleanup();

    SoundState get_state() const { return m_state; }
    bool is_initialized() const { return m_initialized; }

private:
    struct Impl;
    Impl* m_impl;
    SoundState m_state;
    bool m_initialized;
    float m_volume;
};

