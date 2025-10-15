#include "util/EEStore.hpp"
#include <EEPROM.h>
#include <Arduino.h>

namespace EEStore {

static const int BASE_ADDR = 0; // start of EEPROM region

void begin(size_t bytes) {
  EEPROM.begin(bytes);
}

void saveBasePositions(const long* arr, uint8_t n) {
  // Store count + values (fixed layout)
  int addr = BASE_ADDR;
  EEPROM.write(addr++, n);
  for (uint8_t i = 0; i < n; ++i) {
    long v = arr[i];
    EEPROM.put(addr, v);
    addr += sizeof(long);
  }
  EEPROM.commit();
  Serial.println("EEStore: base positions saved.");
}

void loadBasePositions(long* arr, uint8_t n) {
  int addr = BASE_ADDR;
  uint8_t storedN = EEPROM.read(addr++);
  if (storedN != n) {
    Serial.println("EEStore: mismatch count; skipping load.");
    return;
  }
  for (uint8_t i = 0; i < n; ++i) {
    long v;
    EEPROM.get(addr, v);
    addr += sizeof(long);
    arr[i] = v;
  }
  Serial.println("EEStore: base positions loaded.");
}

} // namespace EEStore
