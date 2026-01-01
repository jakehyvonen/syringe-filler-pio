/**
 * @file RFID.cpp
 * @brief Toolhead RFID reader implementation on the primary I2C bus.
 */
#include "hw/RFID.hpp"
#include "hw/Pins.hpp"

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_PN532.h>
#include <string.h>   // for memcpy, memcmp

// Toolhead PN532 on primary I2C bus (Wire).
// IRQ = 27, RST = 14, same style as BaseRFID but with &Wire instead of &Wire1.
static Adafruit_PN532 nfc(27, 14, &Wire);

// ---- State ----
static bool    s_enabled   = false;
static bool    s_available = false;
static uint8_t s_uid[7]    = {0};
static uint8_t s_uidLen    = 0;

// Listener (mirrors BaseRFID)
static RFID::TagListener s_listener      = nullptr;
static void*             s_listenerUser  = nullptr;

namespace RFID {

// Register a listener for newly detected tags.
void setListener(TagListener cb, void* user) {
  s_listener     = cb;
  s_listenerUser = user;
}

// Initialize the PN532 on the primary I2C bus.
void init() {
  // Primary I2C bus
  Wire.begin(Pins::I2C_SDA, Pins::I2C_SCL);

  nfc.begin();

  uint32_t verdata = nfc.getFirmwareVersion();
  if (!verdata) {
    Serial.println(F("[RFID] Didn't find PN532 on Wire. Check DIP (I2C=ON/OFF) & wiring."));
    return;
  }
  Serial.print(F("[RFID] PN532 found. IC: 0x"));
  Serial.println((verdata >> 24) & 0xFF, HEX);
  Serial.print(F("[RFID] Version: "));
  Serial.print((verdata >> 16) & 0xFF);
  Serial.print('.');
  Serial.println((verdata >> 8) & 0xFF);

  nfc.SAMConfig();

  s_enabled   = false;
  s_available = false;
  s_uidLen    = 0;
}

// Enable or disable RFID polling.
void enable(bool e) { s_enabled = e; }
// Return true when polling is enabled.
bool enabled()      { return s_enabled; }

// Return true when a new UID is available.
bool available()    { return s_available; }
// Return the length of the last UID.
uint8_t  uidLen()   { return s_uidLen; }
// Return a pointer to the last UID bytes.
const uint8_t* uidBytes() { return s_uid; }

// Print the last UID to a stream.
void printUID(Stream& s) {
  if (!s_uidLen) { s.println(F("<none>")); return; }
  for (uint8_t i = 0; i < s_uidLen; ++i) {
    if (s_uid[i] < 0x10) s.print('0');
    s.print(s_uid[i], HEX);
    if (i + 1 < s_uidLen) s.print(':');
  }
}

// Poll the PN532 once and update UID state.
void tick() {
  static uint32_t tickCount = 0;

  if (!s_enabled) return;

  Serial.print("[RFID] tick #");
  Serial.print(tickCount++);
  Serial.println(" (enabled)");

  uint8_t uid[7];
  uint8_t uidLength = 0;

  Serial.println("[RFID] calling nfc.readPassiveTargetID(...)");
  bool success = nfc.readPassiveTargetID(PN532_MIFARE_ISO14443A, uid, &uidLength);
  Serial.print("[RFID] readPassiveTargetID done, success=");
  Serial.println(success ? "YES" : "NO");

  if (success) {
    bool changed = (uidLength != s_uidLen) || memcmp(uid, s_uid, uidLength) != 0;
    Serial.print("[RFID] success; changed=");
    Serial.println(changed ? "YES" : "NO");

    if (changed) {
      memcpy(s_uid, uid, uidLength);
      s_uidLen    = uidLength;
      s_available = true;

      Serial.print(F("[RFID] Tag detected, UID ("));
      Serial.print(uidLength);
      Serial.print(F("): "));
      printUID(Serial);
      Serial.println();

      // publish event
      if (s_listener) {
        Serial.println("[RFID] firing listener");
        s_listener(s_uid, s_uidLen, s_listenerUser);
      }
    }
    // debounce
    delay(300);
  }

  delay(5);
}

// Poll for a tag with retries and return true on success.
bool detectOnce(uint16_t tries, uint16_t delay_ms) {
  // same pattern as BaseRFID
  for (uint16_t i = 0; i < tries; ++i) {
    uint8_t uid[7]; 
    uint8_t len = 0;
    if (nfc.readPassiveTargetID(PN532_MIFARE_ISO14443A, uid, &len)) {
      memcpy(s_uid, uid, len);
      s_uidLen    = len;
      s_available = true;

      Serial.print(F("[RFID] Tag detected, UID ("));
      Serial.print(len);
      Serial.print(F("): "));
      printUID(Serial);
      Serial.println();

      if (s_listener) {
        s_listener(s_uid, s_uidLen, s_listenerUser);
      }

      return true;
    }
    delay(delay_ms);
  }
  Serial.println(F("[RFID] No tag detected."));
  return false;
}

} // namespace RFID
