/**
 * @file WifiCredentials.cpp
 * @brief Persist WiFi credentials using ESP32 NVS.
 */
#include "WifiCredentials.hpp"

#include <nvs.h>
#include <nvs_flash.h>

namespace {
struct WifiBlob {
  char ssid[33];
  char password[65];
  uint32_t crc;
};

uint32_t crc32_acc(const uint8_t* p, size_t n) {
  uint32_t c = 0xFFFFFFFFu;
  for (size_t i = 0; i < n; ++i) {
    c ^= p[i];
    for (int k = 0; k < 8; ++k) {
      c = (c & 1) ? (0xEDB88320u ^ (c >> 1)) : (c >> 1);
    }
  }
  return ~c;
}

bool ensureNvsInit() {
  static bool s_inited = false;
  if (s_inited) return true;
  esp_err_t err = nvs_flash_init();
  if (err != ESP_OK) return false;
  s_inited = true;
  return true;
}

bool nvsSaveBlob(const char* ns, const char* key, const void* data, size_t len) {
  nvs_handle_t h;
  if (nvs_open(ns, NVS_READWRITE, &h) != ESP_OK) return false;
  esp_err_t err = nvs_set_blob(h, key, data, len);
  if (err == ESP_OK) err = nvs_commit(h);
  nvs_close(h);
  return err == ESP_OK;
}

bool nvsLoadBlob(const char* ns, const char* key, void* data, size_t len) {
  nvs_handle_t h;
  if (nvs_open(ns, NVS_READONLY, &h) != ESP_OK) return false;
  size_t required = 0;
  esp_err_t err = nvs_get_blob(h, key, nullptr, &required);
  if (err != ESP_OK || required != len) { nvs_close(h); return false; }
  err = nvs_get_blob(h, key, data, &required);
  nvs_close(h);
  return err == ESP_OK;
}

bool nvsGetBlobSize(const char* ns, const char* key, size_t& outSize) {
  nvs_handle_t h;
  if (nvs_open(ns, NVS_READONLY, &h) != ESP_OK) return false;
  size_t required = 0;
  esp_err_t err = nvs_get_blob(h, key, nullptr, &required);
  nvs_close(h);
  if (err != ESP_OK) return false;
  outSize = required;
  return true;
}
}  // namespace

namespace Shared {
namespace WiFiCredentials {

bool load(String& ssid, String& password) {
  if (!ensureNvsInit()) return false;
  size_t required = 0;
  if (!nvsGetBlobSize("wifi", "creds", required)) return false;
  if (required != sizeof(WifiBlob)) return false;

  WifiBlob blob{};
  if (!nvsLoadBlob("wifi", "creds", &blob, sizeof(blob))) return false;

  uint32_t saved = blob.crc;
  blob.crc = 0;
  uint32_t calc = crc32_acc(reinterpret_cast<const uint8_t*>(&blob), sizeof(blob));
  blob.crc = saved;
  if (saved != calc) return false;

  blob.ssid[sizeof(blob.ssid) - 1] = '\0';
  blob.password[sizeof(blob.password) - 1] = '\0';
  ssid = String(blob.ssid);
  password = String(blob.password);
  return ssid.length() > 0;
}

bool save(const String& ssid, const String& password) {
  if (!ensureNvsInit()) return false;
  if (ssid.length() == 0) return false;

  WifiBlob blob{};
  strncpy(blob.ssid, ssid.c_str(), sizeof(blob.ssid) - 1);
  strncpy(blob.password, password.c_str(), sizeof(blob.password) - 1);
  blob.crc = 0;
  blob.crc = crc32_acc(reinterpret_cast<const uint8_t*>(&blob), sizeof(blob));
  return nvsSaveBlob("wifi", "creds", &blob, sizeof(blob));
}

bool clear() {
  if (!ensureNvsInit()) return false;
  nvs_handle_t h;
  if (nvs_open("wifi", NVS_READWRITE, &h) != ESP_OK) return false;
  esp_err_t err = nvs_erase_key(h, "creds");
  if (err == ESP_OK) err = nvs_commit(h);
  nvs_close(h);
  return err == ESP_OK || err == ESP_ERR_NVS_NOT_FOUND;
}

}  // namespace WiFiCredentials
}  // namespace Shared
