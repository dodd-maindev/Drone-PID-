#pragma once

#include <Arduino.h>
#include "wifi_scanner_service.h"

/**
 * @brief Giao diện chọn mạng Wi-Fi và nhập mật khẩu qua Serial Monitor
 */
class WifiSerialConfigurator {
private:
    static void flushStream(Stream &stream) {
        delay(30);
        while (stream.available() > 0) {
            stream.read();
        }
    }

    static String readLine(Stream &stream, uint32_t timeoutMs) {
        flushStream(stream);
        String line = "";
        uint32_t startMs = millis();
        while (millis() - startMs < timeoutMs) {
            if (stream.available() > 0) {
                char c = (char)stream.read();
                if (c == '\r' || c == '\n') {
                    delay(20);
                    while (stream.available() > 0) {
                        char next = (char)stream.peek();
                        if (next == '\r' || next == '\n') stream.read();
                        else break;
                    }
                    return line;
                } else if (c >= 32 && c <= 126) {
                    line += c;
                }
            } else {
                delay(10);
            }
        }
        return line;
    }

public:
    static bool promptUserForNetwork(Stream &serialStream,
                                     WifiScannerService &scanner,
                                     String &chosenSsid,
                                     String &enteredPassword,
                                     uint32_t timeoutMs = 15000) {
        while (true) {
            scanner.scanAvailableNetworks();
            scanner.printScanResults(serialStream);
            int count = scanner.getNetworkCount();
            if (count <= 0) {
                serialStream.println(F("[WIFI] Khong tim thay mang. Go 'r' de quet lai:"));
            } else {
                serialStream.printf("[WIFI] Chon so thu tu mang (1-%d), 'r' de quet lai, hoac cho %ds de bo qua:\n",
                                    count, (int)(timeoutMs / 1000));
                serialStream.print(F(">> Nhap so thu tu roi an Enter: "));
            }

            String userChoice = readLine(serialStream, timeoutMs);
            userChoice.trim();
            if (userChoice.length() == 0 || userChoice.equalsIgnoreCase("s")) {
                serialStream.println(F("\n[WIFI] Bo qua nhap mang thu cong."));
                return false;
            }
            if (userChoice.equalsIgnoreCase("r")) {
                serialStream.println(F("\n[WIFI] Dang tien hanh quet lai mang..."));
                continue;
            }

            int chosenIndex = userChoice.toInt();
            if (chosenIndex >= 1 && chosenIndex <= count) {
                chosenSsid = scanner.getNetworkSsid(chosenIndex - 1);
                bool isEncrypted = scanner.isNetworkEncrypted(chosenIndex - 1);
                serialStream.printf("\n[WIFI] Ban da chon mang [%d]: '%s'\n", chosenIndex, chosenSsid.c_str());

                if (!isEncrypted) {
                    serialStream.println(F("[WIFI] Day la mang khong co mat khau. Dang ket noi..."));
                    enteredPassword = "";
                    return true;
                }

                serialStream.println(F(">> Vui long nhap mat khau Wi-Fi roi an Enter (Send): "));
                enteredPassword = readLine(serialStream, 45000);
                serialStream.printf("[WIFI] Da nhan mat khau (do dai: %d ky tu). Dang ket noi...\n",
                                    enteredPassword.length());
                return true;
            }
            serialStream.println(F("\n[WIFI] So thu tu khong hop le. Vui long chon lai."));
        }
    }
};
