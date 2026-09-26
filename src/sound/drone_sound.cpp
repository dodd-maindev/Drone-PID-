#define MINIAUDIO_IMPLEMENTATION
#include "miniaudio.h"
#include "sound/drone_sound.hpp"

#include <iostream>
#include <vector>
#include <unistd.h>
#include <algorithm>

struct DroneSoundManager::Impl {
    ma_engine engine;
    ma_sound start_sound;
    ma_sound continue_sound;
    bool start_loaded = false;
    bool continue_loaded = false;
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
    : m_impl(new Impl()), m_state(STATE_IDLE), m_initialized(false), m_volume(0.85f) {
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

    std::string start_path = resolve_path("start.mp3", audio_dir);
    std::string continue_path = resolve_path("continue.mp3", audio_dir);

    res = ma_sound_init_from_file(&m_impl->engine, start_path.c_str(), 0, NULL, NULL, &m_impl->start_sound);
    if (res == MA_SUCCESS) {
        ma_sound_set_looping(&m_impl->start_sound, MA_FALSE);
        m_impl->start_loaded = true;
        std::cout << "[✓] Nạp âm thanh cất cánh: " << start_path << std::endl;
    } else {
        std::cerr << "[!] Không tìm thấy file: " << start_path << " (Code: " << res << ")" << std::endl;
    }

    res = ma_sound_init_from_file(&m_impl->engine, continue_path.c_str(), 0, NULL, NULL, &m_impl->continue_sound);
    if (res == MA_SUCCESS) {
        ma_sound_set_looping(&m_impl->continue_sound, MA_TRUE);
        m_impl->continue_loaded = true;
        std::cout << "[✓] Nạp âm thanh bay liên tục: " << continue_path << std::endl;
    } else {
        std::cerr << "[!] Không tìm thấy file: " << continue_path << " (Code: " << res << ")" << std::endl;
    }

    m_state = STATE_IDLE;
    m_volume = 0.85f;
    m_initialized = (m_impl->start_loaded || m_impl->continue_loaded);
    return m_initialized;
}

void DroneSoundManager::update(bool is_armed, bool is_landing, float altitude, float motor_speed_w0) {
    if (!m_initialized || !m_impl->engine_ready) return;

    // Khi người dùng phanh khẩn cấp / Disarm và drone đã ở sát mặt đất -> Ngắt âm thanh lập tức
    if (!is_armed && altitude <= 0.15f) {
        if (m_state != STATE_IDLE) {
            stop();
            m_state = STATE_IDLE;
            m_volume = 0.85f;
            std::cout << "\n[🔇 Âm thanh] Drone đã tiếp đất / ngắt động cơ -> Đã tắt âm thanh." << std::endl;
        }
        return;
    }

    // 1. Xử lý trạng thái ĐANG HẠ CÁNH (Giảm âm lượng từ từ - Fade-out)
    if (is_landing) {
        if (m_state != STATE_LANDING && m_state != STATE_IDLE) {
            m_state = STATE_LANDING;
            std::cout << "\n[🔉 Âm thanh] Nhận lệnh hạ cánh -> Âm lượng đang giảm dần (Fade-out)..." << std::endl;
        }

        if (m_state == STATE_LANDING) {
            // Giảm âm lượng 0.005f mỗi chu kỳ 10ms (100Hz) -> Mất ~1.7s để tắt hẳn
            m_volume = std::max(0.0f, m_volume - 0.005f);

            if (m_impl->continue_loaded) {
                ma_sound_set_volume(&m_impl->continue_sound, m_volume);
            }
            if (m_impl->start_loaded) {
                ma_sound_set_volume(&m_impl->start_sound, m_volume);
            }

            // Drone đã tiếp đất an toàn (độ cao thấp và tốc độ quay nhỏ) hoặc âm lượng đã tắt hết
            if ((altitude <= 0.12f && motor_speed_w0 < 100.0f) || (m_volume <= 0.001f && altitude <= 0.20f)) {
                stop();
                m_state = STATE_IDLE;
                m_volume = 0.85f;
                std::cout << "\n[🔇 Âm thanh] Máy bay đã hạ cánh thành công -> Âm thanh đã tắt hoàn toàn." << std::endl;
            }
            return;
        }
    }

    // Khôi phục nếu người dùng hủy hạ cánh giữa chừng (ví dụ kéo ga bay lên lại)
    if (m_state == STATE_LANDING && !is_landing && is_armed && altitude > 0.20f) {
        m_state = STATE_CONTINUE;
        m_volume = 0.85f;
        if (m_impl->continue_loaded) {
            ma_sound_set_volume(&m_impl->continue_sound, m_volume);
        }
        std::cout << "\n[🔊 Âm thanh] Hủy hạ cánh -> Khôi phục âm lượng bay bình thường." << std::endl;
    }

    // 2. Drone đang ở trạng thái chuẩn bị bay / cất cánh từ mặt đất
    if (m_state == STATE_IDLE) {
        if (is_armed || (altitude >= 0.20f && motor_speed_w0 >= 150.0f)) {
            m_state = STATE_STARTING;
            m_volume = 0.85f;
            if (m_impl->start_loaded) {
                ma_sound_seek_to_pcm_frame(&m_impl->start_sound, 0);
                ma_sound_set_volume(&m_impl->start_sound, m_volume);
                ma_sound_start(&m_impl->start_sound);
                std::cout << "\n[🔊 Âm thanh] Cất cánh từ mặt đất -> Đang phát start.mp3..." << std::endl;
            } else {
                m_state = STATE_CONTINUE;
                if (m_impl->continue_loaded) {
                    ma_sound_seek_to_pcm_frame(&m_impl->continue_sound, 0);
                    ma_sound_set_looping(&m_impl->continue_sound, MA_TRUE);
                    ma_sound_set_volume(&m_impl->continue_sound, m_volume);
                    ma_sound_start(&m_impl->continue_sound);
                }
            }
        }
    } else if (m_state == STATE_STARTING) {
        // Kiểm tra xem audio start.mp3 đã phát hết chưa
        bool finished = false;
        if (m_impl->start_loaded) {
            finished = ma_sound_at_end(&m_impl->start_sound);
        } else {
            finished = true;
        }

        if (finished) {
            // Hết audio start.mp3 -> Chuyển sang continue.mp3 và lặp lại mãi mãi
            if (m_impl->start_loaded) {
                ma_sound_stop(&m_impl->start_sound);
            }
            if (m_impl->continue_loaded) {
                ma_sound_seek_to_pcm_frame(&m_impl->continue_sound, 0);
                ma_sound_set_looping(&m_impl->continue_sound, MA_TRUE);
                ma_sound_set_volume(&m_impl->continue_sound, 0.85f);
                ma_sound_start(&m_impl->continue_sound);
            }
            m_state = STATE_CONTINUE;
            std::cout << "\n[🔊 Âm thanh] start.mp3 kết thúc -> Chuyển sang continue.mp3 (lặp vô tận)..." << std::endl;
        }
    } else if (m_state == STATE_CONTINUE) {
        // Đang bay liên tục: điều chỉnh nhẹ cao độ (pitch) theo tốc độ motor để tạo cảm giác chân thực
        if (m_impl->continue_loaded) {
            float norm = (motor_speed_w0 - 550.0f) / 400.0f;
            if (norm < 0.0f) norm = 0.0f;
            if (norm > 1.0f) norm = 1.0f;
            float pitch = 0.95f + 0.25f * norm;
            ma_sound_set_pitch(&m_impl->continue_sound, pitch);
            ma_sound_set_volume(&m_impl->continue_sound, m_volume);
        }
    }
}

void DroneSoundManager::stop() {
    if (m_impl) {
        if (m_impl->start_loaded) {
            ma_sound_stop(&m_impl->start_sound);
            ma_sound_seek_to_pcm_frame(&m_impl->start_sound, 0);
        }
        if (m_impl->continue_loaded) {
            ma_sound_stop(&m_impl->continue_sound);
            ma_sound_seek_to_pcm_frame(&m_impl->continue_sound, 0);
        }
    }
}

void DroneSoundManager::cleanup() {
    stop();
    if (m_impl) {
        if (m_impl->start_loaded) {
            ma_sound_uninit(&m_impl->start_sound);
            m_impl->start_loaded = false;
        }
        if (m_impl->continue_loaded) {
            ma_sound_uninit(&m_impl->continue_sound);
            m_impl->continue_loaded = false;
        }
        if (m_impl->engine_ready) {
            ma_engine_uninit(&m_impl->engine);
            m_impl->engine_ready = false;
        }
    }
    m_initialized = false;
    m_state = STATE_IDLE;
}
