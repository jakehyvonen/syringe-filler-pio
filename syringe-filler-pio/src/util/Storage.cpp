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

// ---------------------------
// Calibration blobs
// ---------------------------

struct CalBlob {
  uint8_t  pointCount;  // number of valid points in points[]
  Util::CalPoint points[Util::kCalPointCount]; // expects .ratio and .ml
  float    steps_mL;
  uint32_t crc;
};

// ---------------------------
// Base blobs
// ---------------------------

struct BaseBlob {
  Util::BaseMeta         meta;
  App::PotCalibration    cal;
  App::CalibrationPoints points;
  uint32_t               crc;
};

static uint32_t crcForBlob(void* blob, size_t len, uint32_t* crcField) {
  uint32_t saved = *crcField;
  *crcField = 0;
  uint32_t calc = crc32_acc(reinterpret_cast<const uint8_t*>(blob), len);
  *crcField = saved;
  return calc;
}

bool initStorage() {
  if (nvs_flash_init() != ESP_OK) return false;

  if (!LittleFS.begin(true)) return false; // format if needed
  LittleFS.mkdir("/recipes");
  return true;
}

// ---------------------- calibration ----------------------
bool loadCalibration(uint32_t rfid, App::PotCalibration& out) {
  String key = rfidKey(rfid, ":cal");

  size_t required = 0;
  if (!nvsGetBlobSize("cal", key.c_str(), required)) return false;

  if (required != sizeof(CalBlob)) return false;

  CalBlob blob;
  if (!nvsLoadBlob("cal", key.c_str(), &blob, sizeof(blob))) return false;

  uint32_t calc = crcForBlob(&blob, sizeof(blob), &blob.crc);
  if (blob.crc != calc) return false;
  if (blob.pointCount > Util::kCalPointCount) return false;

  out.steps_mL = blob.steps_mL;
  out.pointCount = 0;
  for (uint8_t i = 0; i < blob.pointCount; ++i) {
    out.addPoint(blob.points[i].ml, blob.points[i].ratio);
  }

  if (out.pointCount > 0) {
    out.adcEmpty = Pots::countsFromRatio(out.points[0].ratio);
    if (out.pointCount > 1) {
      out.adcFull = Pots::countsFromRatio(out.points[out.pointCount - 1].ratio);
      out.mlFull = out.points[out.pointCount - 1].volume_ml;
    } else {
      out.adcFull = out.adcEmpty;
      out.mlFull = out.points[0].volume_ml;
    }
  } else {
    out.adcEmpty = 0;
    out.adcFull  = 0;
    out.mlFull   = 0.0f;
  }

  return true;
}

bool saveCalibration(uint32_t rfid, const App::PotCalibration& cal) {
  CalBlob blob{};
  blob.pointCount = cal.pointCount;
  if (blob.pointCount > Util::kCalPointCount) {
    blob.pointCount = Util::kCalPointCount;
  }
  for (uint8_t i = 0; i < blob.pointCount; ++i) {
    blob.points[i].ratio = cal.points[i].ratio;
    blob.points[i].ml    = cal.points[i].volume_ml;
  }

  blob.steps_mL = cal.steps_mL;

  blob.crc = 0;
  blob.crc = crc32_acc(reinterpret_cast<const uint8_t*>(&blob), sizeof(blob));

  String key = rfidKey(rfid, ":cal");
  return nvsSaveBlob("cal", key.c_str(), &blob, sizeof(blob));
}

// ---------------------- base meta ------------------------
// Primary API (with points)
bool loadBase(uint32_t rfid,
              BaseMeta& meta,
              App::PotCalibration& cal,
              App::CalibrationPoints& points) {
  String key = rfidKey(rfid, ":meta");

  size_t required = 0;
  if (!nvsGetBlobSize("base", key.c_str(), required)) return false;

  if (required != sizeof(BaseBlob)) return false;

  BaseBlob blob;
  if (!nvsLoadBlob("base", key.c_str(), &blob, sizeof(blob))) return false;

  uint32_t calc = crcForBlob(&blob, sizeof(blob), &blob.crc);
  if (blob.crc != calc) return false;

  meta   = blob.meta;
  cal    = blob.cal;
  points = blob.points;
  return true;
}

bool saveBase(uint32_t rfid,
              const BaseMeta& meta,
              const App::PotCalibration& cal,
              const App::CalibrationPoints& points) {
  BaseBlob blob{};
  blob.meta    = meta;
  blob.cal     = cal;
  blob.points  = points;

  blob.crc = 0;
  blob.crc = crc32_acc(reinterpret_cast<const uint8_t*>(&blob), sizeof(blob));

  String key = rfidKey(rfid, ":meta");
  return nvsSaveBlob("base", key.c_str(), &blob, sizeof(blob));
}

// Convenience overloads (keep existing call sites working)
bool loadBase(uint32_t rfid, BaseMeta& meta, App::PotCalibration& cal) {
  App::CalibrationPoints dummy{};
  bool ok = loadBase(rfid, meta, cal, dummy);

  // If callers don’t use points, we still try to fold them into cal if present.
  // Only do this if your PotCalibration has compatible fields (as in your Syringe.hpp).
  if (ok && dummy.count > 0) {
    cal.pointCount = 0;
    for (uint8_t i = 0; i < dummy.count && i < App::PotCalibration::kMaxPoints; ++i) {
      cal.addPoint(dummy.points[i].volumeMl, dummy.points[i].ratio);
    }
  }

  return ok;
}

bool saveBase(uint32_t rfid, const BaseMeta& meta, const App::PotCalibration& cal) {
  // If no explicit points provided, serialize an empty set.
  // (Safer than guessing how you want to pack cal.points into CalibrationPoints.)
  App::CalibrationPoints empty{};
  empty.count = 0;
  return saveBase(rfid, meta, cal, empty);
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

// RecipeDTO version
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

// Util::Recipe version
bool saveRecipe(uint32_t toolheadRfid, const Util::Recipe& recipe) {
  if (toolheadRfid == 0) return false;
  File f = LittleFS.open(recipePath(toolheadRfid), "w");
  if (!f) return false;

  DynamicJsonDocument doc(2048);
  JsonArray arr = doc.createNestedArray("steps");
  recipe.toJson(arr);

  bool ok = (serializeJson(doc, f) != 0);
  f.close();
  return ok;
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

} // namespace Util
