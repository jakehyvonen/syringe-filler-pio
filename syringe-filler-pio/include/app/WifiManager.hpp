/**
 * @file WifiManager.hpp
 * @brief WiFi connection helpers and diagnostics.
 */
#pragma once

#include <Arduino.h>
#include <WiFi.h>

namespace App {

class WifiManager {
 public:
  WifiManager();

  bool connect(const String& ssid, const String& password);
  void startAccessPoint();
  String buildStatusJson() const;
  String buildScanJson();
  const char* hostname() const;

 private:
  void ensureEventHandler();
  bool startMdns();
  static const char* statusToString(wl_status_t status);
  static const char* authModeToString(wifi_auth_mode_t mode);
};

}  // namespace App
