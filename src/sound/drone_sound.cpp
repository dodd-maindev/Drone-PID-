#define MINIAUDIO_IMPLEMENTATION
#include "miniaudio.h"
#include "sound/drone_sound.hpp"

#include <iostream>
#include <iomanip>
#include <vector>
#include <cmath>
#include <unistd.h>
#include <algorithm>
#include <chrono>

struct DroneSoundManager::Impl {
    ma_engine engine;
    ma_sound engine_sound;
    ma_audio_buffer audio_buf;
    std::vector<float> pcm_buffer;
    bool buffer_ready = false;
    bool sound_loaded = false;
    bool engine_ready = false;
    float last_applied_vol = -1.0f;
    float last_applied_pitch = -1.0f;
};

static std::string resolve_path(const std::string& filename, const std::string& preferred_dir) {
    std::vector<std::string> search_dirs;
    if (!preferred_dir.empty()) {
        search_dirs.push_back(preferred_dir);
    }
    search_dirs.push_back("simulation/audio");
    search_dirs.push_back("../simulation/audio");
    search_dirs.push_back("/mnt/c/Users/ASUS/Drone-PID-/simulation/audio");
    search_dirs.push_back("/mnt/c/Users/ASUS/Drone-PID-/DroneHITL/simulation/audio");
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

    // Giải mã đoạn âm thanh động cơ ổn định nhất (giây thứ 6 -> 22 của continue.mp3)
    // và trộn nối mượt (Cosine Crossfade 2 giây) trực tiếp trong bộ nhớ RAM (ma_audio_buffer).
    // Cách này loại bỏ hoàn toàn:
    //  1) Đoạn im lặng 4.2 giây ở cuối file continue.mp3 gốc (giây 230..234)
    //  2) Tiếng "bụp bụp" khi lặp qua ranh giới trang bộ nhớ (paged buffer) hoặc đầu/cuối file
    //  3) Lỗi mất tiếng ở lần cất cánh thứ 2 trở đi sau khi hạ cánh
    std::string sound_path = resolve_path("continue.mp3", audio_dir);
    ma_decoder decoder;
    ma_decoder_config dec_cfg = ma_decoder_config_init(ma_format_f32, 1, 48000);
    res = ma_decoder_init_file(sound_path.c_str(), &dec_cfg, &decoder);
    if (res != MA_SUCCESS) {
        sound_path = resolve_path("start.mp3", audio_dir);
        res = ma_decoder_init_file(sound_path.c_str(), &dec_cfg, &decoder);
    }

    if (res == MA_SUCCESS) {
        const ma_uint64 sample_rate = 48000;
        const ma_uint64 skip_frames = 6 * sample_rate;      // Bỏ qua 6s khởi động đầu file
        const ma_uint64 read_frames = 16 * sample_rate;     // Đọc 16s âm thanh động cơ đều nhất
        const ma_uint64 crossfade_frames = 2 * sample_rate; // Crossfade 2s tạo vòng lặp 14s không điểm nối

        ma_decoder_seek_to_pcm_frame(&decoder, skip_frames);
        std::vector<float> raw_pcm(read_frames, 0.0f);
        ma_uint64 frames_read = 0;
        ma_decoder_read_pcm_frames(&decoder, raw_pcm.data(), read_frames, &frames_read);
        ma_decoder_uninit(&decoder);

        if (frames_read > crossfade_frames * 2) {
            ma_uint64 loop_frames = frames_read - crossfade_frames;
            m_impl->pcm_buffer.resize(loop_frames);

            // Trộn mượt (Cosine Crossfade) phần đầu và phần đuôi để vòng lặp liền mạch 100%
            for (ma_uint64 i = 0; i < crossfade_frames; ++i) {
                float t = static_cast<float>(i) / static_cast<float>(crossfade_frames);
                float fade_in  = 0.5f * (1.0f - std::cos(t * 3.14159265f));
                float fade_out = 1.0f - fade_in;
                m_impl->pcm_buffer[i] = fade_in * raw_pcm[i] + fade_out * raw_pcm[loop_frames + i];
            }
            for (ma_uint64 i = crossfade_frames; i < loop_frames; ++i) {
                m_impl->pcm_buffer[i] = raw_pcm[i];
            }

            ma_audio_buffer_config buf_cfg = ma_audio_buffer_config_init(
                ma_format_f32, 1, loop_frames, m_impl->pcm_buffer.data(), NULL);
            res = ma_audio_buffer_init(&buf_cfg, &m_impl->audio_buf);
            if (res == MA_SUCCESS) {
                m_impl->buffer_ready = true;
                ma_data_source_set_looping(&m_impl->audio_buf, MA_TRUE);
                res = ma_sound_init_from_data_source(
                    &m_impl->engine, &m_impl->audio_buf, 0, NULL, &m_impl->engine_sound);
            }
        } else {
            res = MA_ERROR;
        }
    }

    if (res == MA_SUCCESS) {
        ma_sound_set_looping(&m_impl->engine_sound, MA_TRUE);
        ma_sound_set_volume(&m_impl->engine_sound, 0.0f);
        m_impl->last_applied_vol = 0.0f;
        m_impl->last_applied_pitch = 1.0f;
        // Phát liên tục vô tận ở tầng mixer (chỉ điều khiển qua volume & pitch)
        ma_sound_start(&m_impl->engine_sound);
        m_impl->sound_loaded = true;
        std::cout << "[✓] Nạp âm thanh động cơ drone (Seamless Crossfade Loop): " << sound_path << std::endl;
    } else {
        std::cerr << "[!] Không tìm thấy hoặc không thể giải mã file âm thanh động cơ drone (Code: " << res << ")" << std::endl;
    }

    m_state = STATE_IDLE;
    m_volume = 0.0f;
    m_playing = false;
    m_land_start_alt = 2.0f;
    m_initialized = m_impl->sound_loaded;
    return m_initialized;
}

void DroneSoundManager::update(bool is_armed, bool is_landing, float altitude, float vz, float motor_speed_w0, float dt) {
    if (!m_initialized || !m_impl->engine_ready || !m_impl->sound_loaded) return;
    if (dt <= 0.0005f || dt > 0.1f) dt = 0.004f;

    // Âm thanh đã được khởi tạo loop vô tận ở volume=0 từ lúc init.
    // KHÔNG BAO GIỜ gọi seek/start trong update() — đây là nguyên nhân gốc gây tiếng "bụp bụp"!
    // Chỉ cần điều chỉnh volume và pitch, âm thanh tự chạy mãi mãi.

    float target_vol = 0.0f;
    float target_pitch = 1.0f;
    float tau = 0.35f; // Hằng số thời gian làm mượt (giây)

    if (!is_armed) {
        // ====================================================================
        // 1. TRẠNG THÁI DISARMED / TẮT MÁY HOÀN TOÀN
        // ====================================================================
        m_state = STATE_IDLE;
        target_vol = 0.0f;
        target_pitch = 0.75f;
        tau = 0.15f; // Tắt nhanh trong 150ms khi khóa máy
        m_ascent_time = 0.0f;
        m_descent_time = 0.0f;

    } else if (altitude <= 0.20f && vz < 0.12f && motor_speed_w0 < 350.0f && !is_landing) {
        // ====================================================================
        // 2. TRẠNG THÁI QUAY KHÔNG TẢI TẠI CHỖ TRÊN MẶT ĐẤT (GROUND IDLE 200 rad/s)
        // ====================================================================
        m_state = STATE_STARTING;
        target_vol = 0.20f;  // Âm thanh nổ máy thì thầm êm dịu trên bãi đáp
        target_pitch = 0.78f;
        tau = 0.25f;
        m_ascent_time = 0.0f;
        m_descent_time = 0.0f;

    } else if (is_landing || vz < -0.15f) {
        // ====================================================================
        // 3. KHOẢNH KHẮC BAY THẤP XUỐNG & CHUẨN BỊ HẠ CÁNH (DESCENT / LANDING APPROACH)
        //    Âm lượng & cao độ GIẢM DẦN ĐỀU THEO THỜI GIAN VÀ KHOẢNG CÁCH TỚI MẶT ĐẤT
        // ====================================================================
        m_state = STATE_LANDING;
        m_descent_time += dt;
        m_ascent_time = std::max(0.0f, m_ascent_time - dt * 2.5f);

        // Ghi nhận độ cao bắt đầu hạ cánh
        if (m_land_start_alt < 0.5f || m_land_start_alt < altitude) {
            m_land_start_alt = std::max(altitude, 1.5f);
        }

        // Tỷ lệ khoảng cách tiếp cận mặt đất (từ trần hạ cánh xuống mặt sàn 0.14m)
        float land_range = std::max(0.30f, m_land_start_alt - 0.14f);
        float alt_ratio = std::clamp((altitude - 0.14f) / land_range, 0.0f, 1.0f);

        // Đường cong giảm âm tự nhiên: Giảm đều theo thời gian hạ cánh và độ cao thấp dần
        // Hàm mũ bậc 1.25 mô phỏng chính xác sự tan dần của luồng gió quạt (downwash)
        float curved_fade = std::pow(alt_ratio, 1.25f);

        // Tích lũy thời gian hạ: Càng hạ cánh lâu, âm lượng càng thu nhỏ êm đềm
        float time_fade = std::clamp(1.0f - (m_descent_time / 6.0f) * 0.25f, 0.75f, 1.0f);

        // Âm lượng giảm dần đều từ 0.75 -> 0.40 -> 0.15 khi tiếp cận sát đất
        target_vol = (0.12f + 0.65f * curved_fade) * time_fade;

        // Cao độ âm thanh hạ trầm dần khi giảm ga (xả bớt lực cản rít gió)
        target_pitch = 0.75f + 0.25f * curved_fade;

        // Khi đã chạm sàn (altitude <= 0.16m): Cắt dứt khoát về 0 để không bị ngâm tiếng
        if (altitude <= 0.16f) {
            target_vol = 0.0f;
            tau = 0.12f;
        } else {
            tau = 0.35f;
        }

    } else {
        // ====================================================================
        // 4. KHOẢNH KHẮC BAY LÊN CAO (CLIMB / ASCENT & CRUISE)
        //    Âm lượng & cao độ TĂNG DẦN ĐỀU THEO THỜI GIAN LEO DỐC VÀ LỰC ĐẨY ĐỘNG CƠ
        // ====================================================================
        m_state = STATE_CONTINUE;
        m_land_start_alt = 0.0f; // Reset cờ mốc hạ cánh

        // Nhận diện khoảnh khắc bốc đầu leo cao
        if (vz > 0.06f) {
            m_ascent_time += dt;
        } else {
            m_ascent_time = std::max(0.0f, m_ascent_time - dt * 0.8f);
        }
        m_descent_time = std::max(0.0f, m_descent_time - dt * 2.0f);

        // 4.1. Tỷ lệ tăng dần theo thời gian leo dốc (0.0 -> 1.0 trong khoảng 1.5 giây leo đầu tiên)
        float time_boost = std::clamp(m_ascent_time / 1.5f, 0.0f, 1.0f);

        // 4.2. Tỷ lệ độ cao (0.14m mặt đất lên đến 3.0m độ cao bay chuẩn)
        float alt_climb = std::clamp((altitude - 0.14f) / 2.86f, 0.0f, 1.0f);
        float curved_alt = std::pow(alt_climb, 0.80f);

        // 4.3. Tỷ lệ tải trọng lực đẩy động cơ (Tải motor 500 rad/s -> 950 rad/s)
        float motor_load = std::clamp((motor_speed_w0 - 500.0f) / 450.0f, 0.0f, 1.0f);

        // Âm lượng: Khởi hành từ 0.32, dâng trào tăng dần theo thời gian và độ cao lên 0.90 - 0.95
        target_vol = 0.32f + 0.40f * curved_alt + 0.18f * time_boost + 0.08f * motor_load;

        // Cao độ (Pitch): Động cơ rít mạnh, âm thanh gầm vang khi tăng tốc leo cao
        target_pitch = 0.84f + 0.18f * curved_alt + 0.12f * motor_load + 0.10f * std::clamp(vz / 1.5f, 0.0f, 1.0f);

        // Thời gian làm mượt tăng dần êm ái
        tau = 0.30f;
    }

    // 5. BỘ LỌC TÍCH PHÂN THỜI GIAN BẬC 1 (First-order Exponential Filter)
    float alpha = 1.0f - std::exp(-dt / tau);
    m_volume += (target_vol - m_volume) * alpha;
    m_filtered_pitch += (target_pitch - m_filtered_pitch) * alpha;

    if (!is_armed && m_volume < 0.015f) {
        m_volume = 0.0f;
        m_playing = false;
    } else if (is_armed && m_volume > 0.02f) {
        m_playing = true;
    }

    m_volume = std::clamp(m_volume, 0.0f, 0.96f);
    m_filtered_pitch = std::clamp(m_filtered_pitch, 0.70f, 1.30f);

    // 6. CẬP NHẬT AUDIO ENGINE: Bộ lọc Deadband chống nghẽn luồng âm thanh
    if (std::abs(m_volume - m_impl->last_applied_vol) >= 0.004f ||
        (m_volume == 0.0f && m_impl->last_applied_vol != 0.0f)) {
        ma_sound_set_volume(&m_impl->engine_sound, m_volume);
        m_impl->last_applied_vol = m_volume;
    }
    if (std::abs(m_filtered_pitch - m_impl->last_applied_pitch) >= 0.005f) {
        ma_sound_set_pitch(&m_impl->engine_sound, m_filtered_pitch);
        m_impl->last_applied_pitch = m_filtered_pitch;
    }
}

void DroneSoundManager::update(bool is_armed, bool is_landing, float altitude, float motor_speed_w0) {
    float vz = (altitude - m_prev_alt) / 0.004f;
    m_prev_alt = altitude;
    update(is_armed, is_landing, altitude, vz, motor_speed_w0, 0.004f);
}

void DroneSoundManager::stop() {
    // Tuyệt đối KHÔNG gọi ma_sound_stop()! Giữ node âm thanh luôn chạy ngầm ở volume 0.0
    // để lượt bay thứ 2, thứ 3... phát lại ngay lập tức mà không bao giờ bị mất tiếng hay khựng.
    if (m_impl && m_impl->sound_loaded) {
        ma_sound_set_volume(&m_impl->engine_sound, 0.0f);
        m_impl->last_applied_vol = 0.0f;
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
        if (m_impl->buffer_ready) {
            ma_audio_buffer_uninit(&m_impl->audio_buf);
            m_impl->buffer_ready = false;
        }
        m_impl->pcm_buffer.clear();
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
