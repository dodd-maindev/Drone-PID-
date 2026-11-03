#pragma once

#include <Arduino.h>
#include <WiFi.h>

/**
 * @brief Quét danh sách các mạng Wi-Fi 2.4GHz xung quanh
 */
class WifiScannerService {
private:
    int cachedNetworkCount;

public:
    WifiScannerService() : cachedNetworkCount(0) {}

    int scanAvailableNetworks() {
        Serial.println(F("\n[WIFI SCAN] Dang quet cac mang Wi-Fi 2.4GHz xung quanh..."));
        WiFi.disconnect(false, false);
        delay(100);
        WiFi.scanDelete();

        cachedNetworkCount = WiFi.scanNetworks(false, false);
        return cachedNetworkCount;
    }

    int getNetworkCount() const { return cachedNetworkCount; }

    String getNetworkSsid(int index) const {
        if (index < 0 || index >= cachedNetworkCount) return "";
        return WiFi.SSID(index);
    }

    int32_t getNetworkRssi(int index) const {
        if (index < 0 || index >= cachedNetworkCount) return 0;
        return WiFi.RSSI(index);
    }

    bool isNetworkEncrypted(int index) const {
        if (index < 0 || index >= cachedNetworkCount) return false;
        return WiFi.encryptionType(index) != WIFI_AUTH_OPEN;
    }

    void printScanResults(Stream &outputStream) const {
        if (cachedNetworkCount < 0) {
            outputStream.println(F("[WIFI SCAN] Loi quet song. Go 'r' de thu lai."));
            return;
        }
        if (cachedNetworkCount == 0) {
            outputStream.println(F("[WIFI SCAN] Khong tim thay mang 2.4GHz nao. Go 'r' de quet lai."));
            return;
        }
        outputStream.println(F("\n================ DANH SACH MANG WI-FI 2.4GHz ================"));
        for (int i = 0; i < cachedNetworkCount; ++i) {
            const char *sec = isNetworkEncrypted(i) ? "[Co mat khau]" : "[Mo]";
            outputStream.printf("  %2d. %-24s (RSSI: %3d dBm) %s\n",
                                i + 1, WiFi.SSID(i).c_str(), WiFi.RSSI(i), sec);
        }
        outputStream.println(F("============================================================="));
    }
};
