/**
 * @file WifiManager.cpp
 * @brief WiFi connection helpers and diagnostics.
 */
#include "WifiManager.hpp"

#include <ArduinoJson.h>
#include <ESPmDNS.h>

namespace Shared {
namespace {
constexpr uint32_t kWifiConnectTimeoutMs = 20000;
constexpr uint32_t kWifiPollIntervalMs = 500;
constexpr uint32_t kWifiModeDelayMs = 200;
constexpr char kMdnsHostname[] = "syringe-filler";

#ifndef SYRINGE_FILLER_WIFI_SSID
#define SYRINGE_FILLER_WIFI_SSID "SyringeFiller"
#endif
#ifndef SYRINGE_FILLER_WIFI_PASS
#define SYRINGE_FILLER_WIFI_PASS "syringe1234"
#endif

bool s_eventRegistered = false;
}  // namespace

WifiManager::WifiManager() { ensureEventHandler(); }

void WifiManager::ensureEventHandler() {
  if (s_eventRegistered) return;
  s_eventRegistered = true;
  WiFi.onEvent([](WiFiEvent_t event, WiFiEventInfo_t info) {
    Serial.printf("[WiFi] Event: %d\n", static_cast<int>(event));
    if (event == ARDUINO_EVENT_WIFI_STA_DISCONNECTED) {
      Serial.printf("[WiFi] Disconnected, reason: %d\n",
                    static_cast<int>(info.wifi_sta_disconnected.reason));
    }
  });
}

const char* WifiManager::statusToString(wl_status_t status) {
  switch (status) {
    case WL_NO_SSID_AVAIL: return "NO_SSID_AVAIL";
    case WL_SCAN_COMPLETED: return "SCAN_COMPLETED";
    case WL_CONNECTED: return "CONNECTED";
    case WL_CONNECT_FAILED: return "CONNECT_FAILED";
    case WL_CONNECTION_LOST: return "CONNECTION_LOST";
    case WL_DISCONNECTED: return "DISCONNECTED";
    case WL_IDLE_STATUS: return "IDLE";
    default: return "UNKNOWN";
  }
}

const char* WifiManager::authModeToString(wifi_auth_mode_t mode) {
  switch (mode) {
    case WIFI_AUTH_OPEN: return "OPEN";
    case WIFI_AUTH_WEP: return "WEP";
    case WIFI_AUTH_WPA_PSK: return "WPA_PSK";
    case WIFI_AUTH_WPA2_PSK: return "WPA2_PSK";
    case WIFI_AUTH_WPA_WPA2_PSK: return "WPA_WPA2_PSK";
    case WIFI_AUTH_WPA2_ENTERPRISE: return "WPA2_ENTERPRISE";
    case WIFI_AUTH_WPA3_PSK: return "WPA3_PSK";
    case WIFI_AUTH_WPA2_WPA3_PSK: return "WPA2_WPA3_PSK";
    case WIFI_AUTH_WAPI_PSK: return "WAPI_PSK";
    default: return "UNKNOWN";
  }
}

bool WifiManager::startMdns() {
  if (!MDNS.begin(kMdnsHostname)) {
    Serial.println("[WiFi] mDNS failed to start.");
    return false;
  }
  MDNS.addService("http", "tcp", 80);
  Serial.printf("[WiFi] mDNS started: http://%s.local/\n", kMdnsHostname);
  return true;
}

bool WifiManager::connect(const String& ssid, const String& password) {
  ensureEventHandler();
  if (ssid.length() == 0) {
    Serial.println("[WiFi] Missing SSID.");
    return false;
  }
  Serial.printf("[WiFi] Connecting to SSID '%s'...\n", ssid.c_str());

  WiFi.softAPdisconnect(true);
  WiFi.disconnect(false, false);
  delay(kWifiModeDelayMs);
  WiFi.mode(WIFI_STA);
  delay(kWifiModeDelayMs);
  WiFi.setSleep(false);
  WiFi.begin(ssid.c_str(), password.c_str());

  unsigned long start = millis();
  wl_status_t lastStatus = WL_IDLE_STATUS;
  while (millis() - start < kWifiConnectTimeoutMs) {
    wl_status_t status = WiFi.status();
    if (status != lastStatus) {
      Serial.printf("[WiFi] Status: %s\n", statusToString(status));
      lastStatus = status;
    }
    if (status == WL_CONNECTED) {
      IPAddress ip = WiFi.localIP();
      Serial.printf("[WiFi] Connected. IP: %s RSSI: %ld dBm\n",
                    ip.toString().c_str(), WiFi.RSSI());
      startMdns();
      return true;
    }
    delay(kWifiPollIntervalMs);
  }

  Serial.println("[WiFi] Connection timed out.");
  return false;
}

void WifiManager::startAccessPoint() {
  Serial.println("[WiFi] Starting access point...");
  WiFi.mode(WIFI_AP);
  bool ok = WiFi.softAP(SYRINGE_FILLER_WIFI_SSID, SYRINGE_FILLER_WIFI_PASS);
  if (!ok) {
    Serial.println("[WiFi] Failed to start AP.");
    return;
  }
  IPAddress ip = WiFi.softAPIP();
  Serial.printf("[WiFi] AP '%s' started. IP: %s\n", SYRINGE_FILLER_WIFI_SSID, ip.toString().c_str());
}

String WifiManager::buildStatusJson() const {
  JsonDocument doc;
  doc["connected"] = WiFi.isConnected();
  doc["status"] = statusToString(WiFi.status());
  doc["ssid"] = WiFi.SSID();
  doc["ip"] = WiFi.localIP().toString();
  doc["hostname"] = kMdnsHostname;
  doc["ap_ssid"] = SYRINGE_FILLER_WIFI_SSID;
  doc["ap_ip"] = WiFi.softAPIP().toString();
  String data;
  serializeJson(doc, data);
  return data;
}

String WifiManager::buildScanJson() {
  wifi_mode_t originalMode = WiFi.getMode();
  bool switched = false;
  if (originalMode == WIFI_AP) {
    WiFi.mode(WIFI_AP_STA);
    switched = true;
    delay(kWifiModeDelayMs);
  } else if (originalMode == WIFI_MODE_NULL) {
    WiFi.mode(WIFI_STA);
    switched = true;
    delay(kWifiModeDelayMs);
  }

  WiFi.setSleep(false);
  int16_t count = WiFi.scanNetworks(false, true);

  JsonDocument doc;
  doc["count"] = count < 0 ? 0 : count;
  JsonArray networks = doc["networks"].to<JsonArray>();
  if (count > 0) {
    for (int16_t i = 0; i < count; ++i) {
      JsonObject entry = networks.add<JsonObject>();
      entry["ssid"] = WiFi.SSID(i);
      entry["rssi"] = WiFi.RSSI(i);
      entry["channel"] = WiFi.channel(i);
      entry["auth"] = authModeToString(WiFi.encryptionType(i));
    }
  }
  String data;
  serializeJson(doc, data);

  WiFi.scanDelete();
  if (switched) {
    WiFi.mode(originalMode);
    delay(kWifiModeDelayMs);
  }
  return data;
}

const char* WifiManager::hostname() const { return kMdnsHostname; }

}  // namespace Shared
