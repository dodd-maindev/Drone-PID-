#pragma once

#include <Arduino.h>
#include <Preferences.h>

/**
 * @brief Quản lý lưu trữ thông tin Wi-Fi (SSID & Password) vào bộ nhớ Flash NVS của ESP32
 */
class WifiCredentialStorage {
private:
    static constexpr const char *STORAGE_NAMESPACE = "drone_wifi";
    static constexpr const char *STORAGE_KEY_SSID = "sta_ssid";
    static constexpr const char *STORAGE_KEY_PASSWORD = "sta_pass";

public:
    bool hasStoredCredentials() const {
        Preferences preferences;
        preferences.begin(STORAGE_NAMESPACE, true);
        String storedSsid = preferences.getString(STORAGE_KEY_SSID, "");
        preferences.end();
        return storedSsid.length() > 0;
    }

    String getStoredSsid() const {
        Preferences preferences;
        preferences.begin(STORAGE_NAMESPACE, true);
        String storedSsid = preferences.getString(STORAGE_KEY_SSID, "");
        preferences.end();
        return storedSsid;
    }

    String getStoredPassword() const {
        Preferences preferences;
        preferences.begin(STORAGE_NAMESPACE, true);
        String storedPassword = preferences.getString(STORAGE_KEY_PASSWORD, "");
        preferences.end();
        return storedPassword;
    }

    bool saveCredentials(const String &networkSsid, const String &networkPassword) {
        Preferences preferences;
        if (!preferences.begin(STORAGE_NAMESPACE, false)) {
            return false;
        }
        preferences.putString(STORAGE_KEY_SSID, networkSsid);
        preferences.putString(STORAGE_KEY_PASSWORD, networkPassword);
        preferences.end();
        return true;
    }

    void clearCredentials() {
        Preferences preferences;
        preferences.begin(STORAGE_NAMESPACE, false);
        preferences.clear();
        preferences.end();
    }
};
