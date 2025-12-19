#include "util/Storage.hpp"
#include "util/Recipe.hpp"
#include "hw/Pots.hpp"
#include <nvs_flash.h>
#include <nvs.h>
#include <LittleFS.h>
#include <ArduinoJson.h>

namespace {

uint32_t crc32_acc(const uint8_t* p, size_t n) {
  uint32_t c = 0xFFFFFFFFu;
  for (size_t i = 0; i < n; ++i) {
    c ^= p[i];
    for (int k = 0; k < 8; ++k)
      c = (c & 1) ? (0xEDB88320u ^ (c >> 1)) : (c >> 1);
  }
  return ~c;
}

String rfidKey(uint32_t rfid, const char* suffix) {
  char buf[16];
  snprintf(buf, sizeof(buf), "%08X", rfid);
  String s = buf;
  s += suffix;
  return s;
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

} // namespace

namespace Util {

namespace {
constexpr uint16_t kBlobVersionV1 = 1;
constexpr uint16_t kBlobVersionV2 = 2;
constexpr uint8_t kCalFlagLegacy = 0x01;
constexpr uint8_t kMinPointCount = 2;
}

struct PotCalibrationV1 {
  uint16_t adcEmpty;
  uint16_t adcFull;
  float mlFull;
  float steps_mL;
};

struct CalBlobV1 {
  uint16_t version;
  PotCalibrationV1 cal;
  uint32_t crc;
};

struct BaseBlobV1 {
  uint16_t version;
  Util::BaseMeta meta;
  PotCalibrationV1 cal;
  uint32_t crc;
};

struct CalBlobV2 {
  uint16_t version;
  uint8_t flags;
  uint8_t pointCount;
  Util::CalPoint points[Util::kCalPointCount];
  float steps_mL;
  uint32_t crc;
};

struct BaseBlobV2 {
  uint16_t version;
  Util::BaseMeta meta;
  uint8_t flags;
  uint8_t pointCount;
  Util::CalPoint points[Util::kCalPointCount];
  float steps_mL;
  uint32_t crc;
};

uint8_t clampPointCount(uint8_t count) {
  return (count > Util::kCalPointCount) ? Util::kCalPointCount : count;
}

void fillPointsFromLegacy(const App::PotCalibration& cal,
                          Util::CalPoint* points,
                          uint8_t& count) {
  count = kMinPointCount;
  points[0].volume_ml = 0.0f;
  points[0].ratio = Pots::ratioFromCounts(cal.adcEmpty);
  points[1].volume_ml = cal.mlFull;
  points[1].ratio = Pots::ratioFromCounts(cal.adcFull);
}

void applyPointsToCalibration(const Util::CalPoint* points,
                              uint8_t count,
                              App::PotCalibration& cal) {
  uint8_t capped = clampPointCount(count);
  cal.pointCount = capped;
  for (uint8_t i = 0; i < Util::kCalPointCount; ++i) {
    cal.points[i].volume_ml = 0.0f;
    cal.points[i].ratio = 0.0f;
  }
  for (uint8_t i = 0; i < capped; ++i) {
    cal.points[i] = points[i];
  }
  if (capped >= kMinPointCount) {
    cal.adcEmpty = Pots::countsFromRatio(points[0].ratio);
    cal.adcFull  = Pots::countsFromRatio(points[capped - 1].ratio);
    cal.mlFull   = points[capped - 1].volume_ml;
  }
}
bool initStorage() {
  // NVS
  if (nvs_flash_init() != ESP_OK) return false;
  // FS
  if (!LittleFS.begin(true)) return false;  // true: format if needed
  LittleFS.mkdir("/recipes");
  return true;
}

// ---------------------- calibration ----------------------
bool loadCalibration(uint32_t rfid, App::PotCalibration& out) {
  String key = rfidKey(rfid, ":cal");
  size_t required = 0;
  if (!nvsGetBlobSize("cal", key.c_str(), required)) return false;
  if (required == sizeof(CalBlobV2)) {
    CalBlobV2 blob;
    if (!nvsLoadBlob("cal", key.c_str(), &blob, sizeof(blob))) return false;
    uint32_t saved = blob.crc;
    blob.crc = 0;
    uint32_t calc = crc32_acc(reinterpret_cast<uint8_t*>(&blob), sizeof(blob));
    if (saved != calc) return false;
    if (blob.version != kBlobVersionV2) return false;
    if (blob.pointCount < kMinPointCount || blob.pointCount > Util::kCalPointCount) return false;
    applyPointsToCalibration(blob.points, blob.pointCount, out);
    out.steps_mL = blob.steps_mL;
    out.legacy   = (blob.flags & kCalFlagLegacy) != 0;
    return true;
  }
  if (required == sizeof(CalBlobV1)) {
    CalBlobV1 blob;
    if (!nvsLoadBlob("cal", key.c_str(), &blob, sizeof(blob))) return false;
    uint32_t saved = blob.crc;
    blob.crc = 0;
    uint32_t calc = crc32_acc(reinterpret_cast<uint8_t*>(&blob), sizeof(blob));
    if (saved != calc) return false;
    if (blob.version != kBlobVersionV1) return false;
    float emptyRatio = Pots::ratioFromCounts(blob.cal.adcEmpty);
    float fullRatio  = Pots::ratioFromCounts(blob.cal.adcFull);
    out.adcEmpty = Pots::countsFromRatio(emptyRatio);
    out.adcFull  = Pots::countsFromRatio(fullRatio);
    out.mlFull   = blob.cal.mlFull;
    out.steps_mL = blob.cal.steps_mL;
    uint8_t count = 0;
    fillPointsFromLegacy(out, out.points, count);
    out.pointCount = count;
    out.legacy   = true;
    return true;
  }
  return false;
}

bool saveCalibration(uint32_t rfid, const App::PotCalibration& cal) {
  CalBlobV2 blob;
  blob.version = kBlobVersionV2;
  blob.flags = cal.legacy ? kCalFlagLegacy : 0;
  uint8_t count = clampPointCount(cal.pointCount);
  for (uint8_t i = 0; i < Util::kCalPointCount; ++i) {
    blob.points[i] = {};
  }
  if (count < kMinPointCount) {
    fillPointsFromLegacy(cal, blob.points, count);
  } else {
    for (uint8_t i = 0; i < count; ++i) {
      blob.points[i] = cal.points[i];
    }
  }
  blob.pointCount = count;
  blob.steps_mL = cal.steps_mL;
  blob.crc = 0;
  blob.crc = crc32_acc(reinterpret_cast<uint8_t*>(&blob), sizeof(blob));
  String key = rfidKey(rfid, ":cal");
  return nvsSaveBlob("cal", key.c_str(), &blob, sizeof(blob));
}

// ---------------------- base meta ------------------------
bool loadBase(uint32_t rfid, BaseMeta& meta, App::PotCalibration& cal) {
  String key = rfidKey(rfid, ":meta");
  size_t required = 0;
  if (!nvsGetBlobSize("base", key.c_str(), required)) return false;
  if (required == sizeof(BaseBlobV2)) {
    BaseBlobV2 blob;
    if (!nvsLoadBlob("base", key.c_str(), &blob, sizeof(blob))) return false;
    uint32_t saved = blob.crc;
    blob.crc = 0;
    uint32_t calc = crc32_acc(reinterpret_cast<uint8_t*>(&blob), sizeof(blob));
    if (saved != calc) return false;
    if (blob.version != kBlobVersionV2) return false;
    if (blob.pointCount < kMinPointCount || blob.pointCount > Util::kCalPointCount) return false;
    meta = blob.meta;
    applyPointsToCalibration(blob.points, blob.pointCount, cal);
    cal.steps_mL = blob.steps_mL;
    cal.legacy   = (blob.flags & kCalFlagLegacy) != 0;
    return true;
  }
  if (required == sizeof(BaseBlobV1)) {
    BaseBlobV1 blob;
    if (!nvsLoadBlob("base", key.c_str(), &blob, sizeof(blob))) return false;
    uint32_t saved = blob.crc;
    blob.crc = 0;
    uint32_t calc = crc32_acc(reinterpret_cast<uint8_t*>(&blob), sizeof(blob));
    if (saved != calc) return false;
    if (blob.version != kBlobVersionV1) return false;
    meta = blob.meta;
    float emptyRatio = Pots::ratioFromCounts(blob.cal.adcEmpty);
    float fullRatio  = Pots::ratioFromCounts(blob.cal.adcFull);
    cal.adcEmpty = Pots::countsFromRatio(emptyRatio);
    cal.adcFull  = Pots::countsFromRatio(fullRatio);
    cal.mlFull   = blob.cal.mlFull;
    cal.steps_mL = blob.cal.steps_mL;
    uint8_t count = 0;
    fillPointsFromLegacy(cal, cal.points, count);
    cal.pointCount = count;
    cal.legacy   = true;
    return true;
  }
  return false;
}

bool saveBase(uint32_t rfid, const BaseMeta& meta, const App::PotCalibration& cal) {
  BaseBlobV2 blob;
  blob.version = kBlobVersionV2;
  blob.meta = meta;
  blob.flags = cal.legacy ? kCalFlagLegacy : 0;
  uint8_t count = clampPointCount(cal.pointCount);
  for (uint8_t i = 0; i < Util::kCalPointCount; ++i) {
    blob.points[i] = {};
  }
  if (count < kMinPointCount) {
    fillPointsFromLegacy(cal, blob.points, count);
  } else {
    for (uint8_t i = 0; i < count; ++i) {
      blob.points[i] = cal.points[i];
    }
  }
  blob.pointCount = count;
  blob.steps_mL = cal.steps_mL;
  blob.crc = 0;
  blob.crc = crc32_acc(reinterpret_cast<uint8_t*>(&blob), sizeof(blob));
  String key = rfidKey(rfid, ":meta");
  return nvsSaveBlob("base", key.c_str(), &blob, sizeof(blob));
}

// ---------------------- recipes --------------------------
// stored as /recipes/<RFID-HEX>.json
static String recipePath(uint32_t toolheadRfid) {
  char buf[16];
  snprintf(buf, sizeof(buf), "%08X", toolheadRfid);
  String p = "/recipes/";
  p += buf;
  p += ".json";
  return p;
}

bool saveRecipe(uint32_t toolheadRfid, const RecipeDTO& in) {
  DynamicJsonDocument doc(2048);
  char buf[16];
  snprintf(buf, sizeof(buf), "%08X", toolheadRfid);
  doc["toolhead_rfid"] = buf;

  JsonArray arr = doc.createNestedArray("steps");
  for (uint8_t i = 0; i < in.count; ++i) {
    JsonObject o = arr.createNestedObject();
    o["base_slot"] = in.steps[i].baseSlot;
    o["ml"]        = in.steps[i].ml;
  }

  File f = LittleFS.open(recipePath(toolheadRfid), "w");
  if (!f) return false;
  if (serializeJson(doc, f) == 0) { f.close(); return false; }
  f.close();
  return true;
}

bool loadRecipe(uint32_t toolheadRfid, RecipeDTO& out) {
  File f = LittleFS.open(recipePath(toolheadRfid), "r");
  if (!f) return false;
  DynamicJsonDocument doc(2048);
  DeserializationError err = deserializeJson(doc, f);
  f.close();
  if (err) return false;

  JsonArray arr = doc["steps"].as<JsonArray>();
  out.count = 0;
  for (JsonVariant v : arr) {
    if (out.count >= RecipeDTO::kMaxSteps) break;
    out.steps[out.count].baseSlot = v["base_slot"] | 0;
    out.steps[out.count].ml       = v["ml"] | 0.0f;
    out.count++;
  }
  return true;
}


// ------------------------------------------------------
// base axis positions
// namespace: "bases"
// key: "p<idx0>"
// value: int32 (long)
// ------------------------------------------------------
bool loadBasePos(uint8_t idx0, long& steps) {
  char key[8];
  snprintf(key, sizeof(key), "p%u", idx0);
  long tmp = 0;
  if (!nvsLoadBlob("bases", key, &tmp, sizeof(tmp))) return false;
  steps = tmp;
  return true;
}

bool saveBasePos(uint8_t idx0, long steps) {
  char key[8];
  snprintf(key, sizeof(key), "p%u", idx0);
  return nvsSaveBlob("bases", key, &steps, sizeof(steps));
}

bool saveRecipe(uint32_t toolheadRfid, const Util::Recipe& recipe) {
    if (toolheadRfid == 0) return false;
    File f = LittleFS.open(recipePath(toolheadRfid), "w");
    if (!f) return false;

    DynamicJsonDocument doc(2048);
    JsonArray arr = doc.createNestedArray("steps");
    recipe.toJson(arr);
    serializeJson(doc, f);
    f.close();
    return true;
}

bool loadRecipe(uint32_t toolheadRfid, Util::Recipe& recipe) {
    if (toolheadRfid == 0) return false;
    File f = LittleFS.open(recipePath(toolheadRfid), "r");
    if (!f) return false;

    DynamicJsonDocument doc(2048);
    DeserializationError err = deserializeJson(doc, f);
    f.close();
    if (err) return false;

    JsonArrayConst arr = doc["steps"].as<JsonArrayConst>();
    recipe.fromJson(arr);
    return !recipe.isEmpty();
}


} // namespace Util
