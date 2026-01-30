/**
 * @file Storage.cpp
 * @brief LittleFS persistence for base syringe metadata.
 */
#include "Storage.hpp"

#include <ArduinoJson.h>
#include <LittleFS.h>
#include <cstring>

namespace {
String basePath(uint32_t rfid) {
  char buf[16];
  snprintf(buf, sizeof(buf), "%08X", rfid);
  String path = "/bases/";
  path += buf;
  path += ".json";
  return path;
}
}  // namespace

namespace Storage {

bool init() {
  if (!LittleFS.begin(true)) return false;
  LittleFS.mkdir("/bases");
  return true;
}

bool loadBase(uint32_t rfid, BaseInfo& out) {
  if (rfid == 0) return false;
  File f = LittleFS.open(basePath(rfid), "r");
  if (!f) return false;

  JsonDocument doc;
  DeserializationError err = deserializeJson(doc, f);
  f.close();
  if (err) return false;

  strlcpy(out.paintName, doc["paint_name"] | "", sizeof(out.paintName));
  strlcpy(out.recipeName, doc["recipe_name"] | "", sizeof(out.recipeName));
  strlcpy(out.recipeId, doc["recipe_id"] | "", sizeof(out.recipeId));
  strlcpy(out.notes, doc["notes"] | "", sizeof(out.notes));
  return true;
}

bool saveBase(uint32_t rfid, const BaseInfo& info) {
  if (rfid == 0) return false;
  File f = LittleFS.open(basePath(rfid), "w");
  if (!f) return false;

  JsonDocument doc;
  doc["paint_name"] = info.paintName;
  doc["recipe_name"] = info.recipeName;
  doc["recipe_id"] = info.recipeId;
  doc["notes"] = info.notes;

  bool ok = serializeJson(doc, f) != 0;
  f.close();
  return ok;
}

bool deleteBase(uint32_t rfid) {
  if (rfid == 0) return false;
  return LittleFS.remove(basePath(rfid));
}

bool listBaseIds(uint32_t* out, size_t max, size_t& count) {
  count = 0;
  File root = LittleFS.open("/bases");
  if (!root || !root.isDirectory()) return false;

  File file = root.openNextFile();
  while (file) {
    if (!file.isDirectory()) {
      String name = file.name();
      int slash = name.lastIndexOf('/');
      if (slash >= 0) name = name.substring(slash + 1);
      if (name.endsWith(".json")) {
        String hex = name.substring(0, name.length() - 5);
        uint32_t val = strtoul(hex.c_str(), nullptr, 16);
        if (val != 0 && count < max) {
          out[count++] = val;
        }
      }
    }
    file.close();
    file = root.openNextFile();
  }
  return true;
}

}  // namespace Storage
