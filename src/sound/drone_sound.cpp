#define MINIAUDIO_IMPLEMENTATION
#include "miniaudio.h"
#include "sound/drone_sound.hpp"

#include <iostream>
#include <iomanip>
#include <vector>
#include <unistd.h>
#include <algorithm>

struct DroneSoundManager::Impl {
    ma_engine engine;
    ma_sound engine_sound;
    bool sound_loaded = false;
    bool engine_ready = false;
};

static std::string resolve_path(const std::string& filename, const std::string& preferred_dir) {
    std::vector<std::string> search_dirs;
    if (!preferred_dir.empty()) {
        search_dirs.push_back(preferred_dir);
    }
    search_dirs.push_back("simulation/audio");
    search_dirs.push_back("../simulation/audio");
    search_dirs.push_back("/home/do/drone_control_cpp/simulation/audio");

    for (const auto& dir : search_dirs) {
        std::string full_path = dir + "/" + filename;
        if (access(full_path.c_str(), R_OK) == 0) {
            return full_path;
        }
    }
    return filename;
}

DroneSoundManager::DroneSoundManager()
    : m_impl(new Impl()), m_state(STATE_IDLE), m_initialized(false), m_playing(false), m_volume(0.0f), m_land_start_alt(2.0f) {
}

DroneSoundManager::~DroneSoundManager() {
    cleanup();
    delete m_impl;
}

bool DroneSoundManager::init(const std::string& audio_dir) {
    if (m_initialized) return true;

    ma_result res = ma_engine_init(NULL, &m_impl->engine);
    if (res != MA_SUCCESS) {
        std::cerr << "[!] Cảnh báo: Không thể khởi tạo Audio Engine (Code: " << res << ")" << std::endl;
        return false;
    }
    m_impl->engine_ready = true;

    // Ưu tiên nạp continue.mp3 (âm thanh động cơ bay liên tục), fallback sang start.mp3
    std::string sound_path = resolve_path("continue.mp3", audio_dir);
    res = ma_sound_init_from_file(&m_impl->engine, sound_path.c_str(), 0, NULL, NULL, &m_impl->engine_sound);
    if (res != MA_SUCCESS) {
        sound_path = resolve_path("start.mp3", audio_dir);
        res = ma_sound_init_from_file(&m_impl->engine, sound_path.c_str(), 0, NULL, NULL, &m_impl->engine_sound);
    }

    if (res == MA_SUCCESS) {
        ma_sound_set_looping(&m_impl->engine_sound, MA_TRUE);
        ma_sound_set_volume(&m_impl->engine_sound, 0.0f);
        // Bắt đầu phát âm thanh lặp vô tận ở volume 0.0 ngay khi khởi tạo
        // Nhờ vậy luồng audio luôn sẵn sàng (Hot/Pre-primed), loại bỏ hoàn toàn 100% độ trễ khởi động khi bấm Y!
        ma_sound_start(&m_impl->engine_sound);
        m_impl->sound_loaded = true;
        std::cout << "[✓] Nạp âm thanh động cơ drone (Pre-primed Looping): " << sound_path << std::endl;
    } else {
        std::cerr << "[!] Không tìm thấy file âm thanh động cơ drone (Code: " << res << ")" << std::endl;
    }

    m_state = STATE_IDLE;
    m_volume = 0.0f;
    m_playing = false;
    m_land_start_alt = 2.0f;
    m_initialized = m_impl->sound_loaded;
    return m_initialized;
}

void DroneSoundManager::update(bool is_armed, bool is_landing, float altitude, float motor_speed_w0) {
    if (!m_initialized || !m_impl->engine_ready || !m_impl->sound_loaded) return;

    // 1. Máy bay nằm yên trên mặt đất / Disarmed:
    // Khi máy bay không ARM và ở sát mặt đất (altitude <= 0.20m) -> Âm thanh tắt hoàn toàn (Volume = 0)
    if (!is_armed && altitude <= 0.20f) {
        if (m_playing || m_state != STATE_IDLE || m_volume > 0.001f) {
            stop();
            std::cout << "\n[🔇 Âm thanh] Máy bay đã tiếp đất an toàn -> Âm thanh tắt hoàn toàn." << std::endl;
        }
        return;
    }

    // Đánh dấu âm thanh đang hoạt động khi máy bay ARM
    // Khởi tạo ngay âm lượng xuất phát 0.20 (không bị delay do bộ lọc từ 0.0)
    if (is_armed && !m_playing) {
        m_playing = true;
        m_volume = 0.20f;
        ma_sound_set_volume(&m_impl->engine_sound, m_volume);
        std::cout << "\n[🔊 Âm thanh] Kích hoạt cất cánh tức thì -> Âm lượng tăng dần theo độ cao..." << std::endl;
    }

    // 2. Tính toán mục tiêu âm lượng (target_vol) và cao độ (target_pitch)
    // Hoàn toàn mượt mà theo độ cao thực tế (Altitude) và vòng tua motor
    float target_vol = 0.85f;
    float target_pitch = 1.0f;

    if (is_landing) {
        // --- CHẾ ĐỘ HẠ CÁNH (Âm lượng GIẢM DẦN theo độ cao cho tới khi tiếp đất thì tắt hẳn) ---
        if (m_state != STATE_LANDING) {
            m_state = STATE_LANDING;
            m_land_start_alt = std::max(altitude, 1.0f);
            std::cout << "\n[🔉 Âm thanh] Bắt đầu hạ cánh -> Âm lượng giảm dần đều từ trên cao xuống đất..." << std::endl;
        }

        // Tỷ lệ độ cao từ lúc bắt đầu hạ cánh xuống mặt đất (~0.14m)
        float land_range = std::max(0.20f, m_land_start_alt - 0.14f);
        float alt_ratio = std::clamp((altitude - 0.14f) / land_range, 0.0f, 1.0f);

        // Áp dụng đường cong suy giảm âm lượng thực tế (Perceptual Loudness Curve):
        // Ở 2.0m: 0.85 (vang to) -> 1.0m: 0.30 -> 0.5m: 0.10 -> chạm đất (0.14m): 0.00 (tắt hẳn)
        float curved_fade = std::pow(alt_ratio, 1.35f);
        target_vol = 0.85f * curved_fade;

        // Cao độ âm thanh cũng hạ trầm dần theo độ cao tạo cảm giác xả gió hạ cánh
        target_pitch = 0.78f + 0.30f * curved_fade;

    } else if (is_armed) {
        // --- CHẾ ĐỘ CẤT CÁNH & BAY LÊN (Âm lượng TĂNG DẦN theo độ cao thực tế) ---
        m_state = (altitude < 1.6f) ? STATE_STARTING : STATE_CONTINUE;

        // Tỷ lệ độ cao leo dốc từ mặt đất (0.14m) lên độ cao bay chuẩn (1.90m)
        float climb_ratio = std::clamp((altitude - 0.14f) / 1.76f, 0.0f, 1.0f);
        float curved_climb = std::pow(climb_ratio, 0.85f);

        // Tăng dần âm lượng theo độ cao:
        // - Vừa kích hoạt cất cánh trên mặt đất: 0.20 (tiếng đề pa êm ái)
        // - Càng bay lên cao âm lượng càng TĂNG DẦN ĐỀU lên 0.85
        target_vol = 0.20f + 0.65f * curved_climb;
        target_pitch = 0.82f + 0.26f * curved_climb;

    } else {
        m_state = STATE_IDLE;
        target_vol = 0.0f;
    }

    // 3. Bộ lọc làm mượt âm lượng
    // Khi hạ cánh dùng hệ số nhanh hơn (0.22) để triệt tiêu hoàn toàn độ trễ khi tiếp đất
    float alpha = is_landing ? 0.22f : 0.15f;
    m_volume += (target_vol - m_volume) * alpha;

    // Khi hạ cánh xuống sát đất, cắt dứt khoát về 0 để không bị ngâm tiếng
    if (is_landing && target_vol < 0.015f) {
        m_volume = 0.0f;
    }
    m_volume = std::clamp(m_volume, 0.0f, 0.90f);

    ma_sound_set_volume(&m_impl->engine_sound, m_volume);
    ma_sound_set_pitch(&m_impl->engine_sound, target_pitch);
}

void DroneSoundManager::stop() {
    if (m_impl && m_impl->sound_loaded) {
        ma_sound_set_volume(&m_impl->engine_sound, 0.0f);
    }
    m_playing = false;
    m_volume = 0.0f;
    m_state = STATE_IDLE;
}

void DroneSoundManager::cleanup() {
    if (m_impl) {
        if (m_impl->sound_loaded) {
            ma_sound_stop(&m_impl->engine_sound);
            ma_sound_uninit(&m_impl->engine_sound);
            m_impl->sound_loaded = false;
        }
        if (m_impl->engine_ready) {
            ma_engine_uninit(&m_impl->engine);
            m_impl->engine_ready = false;
        }
    }
    m_initialized = false;
    m_playing = false;
    m_volume = 0.0f;
    m_state = STATE_IDLE;
}
