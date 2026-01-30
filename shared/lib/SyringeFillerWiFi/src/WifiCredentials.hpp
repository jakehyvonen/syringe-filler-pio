/**
 * @file WifiCredentials.hpp
 * @brief Persist WiFi credentials using ESP32 NVS.
 */
#pragma once

#include <Arduino.h>

namespace Shared {
namespace WiFiCredentials {

bool load(String& ssid, String& password);
bool save(const String& ssid, const String& password);
bool clear();

}  // namespace WiFiCredentials
}  // namespace Shared
