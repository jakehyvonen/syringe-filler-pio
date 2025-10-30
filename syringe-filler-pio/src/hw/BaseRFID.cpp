#include "hw/BaseRFID.hpp"
#include "hw/Pins.hpp"

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_PN532.h>

// Use the Adafruit I2C ctor with an explicit TwoWire*
// IRQ/RESET are unused for us; pass 0xFF so the lib won’t touch pins.
static Adafruit_PN532 nfc(27, 14, &Wire1);

// ---- State ----
static bool    s_enabled   = false;
static bool    s_available = false;
static uint8_t s_uid[7]    = {0};
static uint8_t s_uidLen    = 0;

namespace BaseRFID {

void init() {
  // Bring up the secondary I2C controller exactly like your primary
  Wire1.begin(Pins::I2C2_SDA, Pins::I2C2_SCL);
  

  nfc.begin();

  uint32_t verdata = nfc.getFirmwareVersion();
  if (!verdata) {
    Serial.println(F("[BaseRFID] Didn't find PN532 on Wire1. Check DIP (I2C=ON/OFF) & wiring."));
    return;
  }
  Serial.print(F("[BaseRFID] PN532 found. IC: 0x"));
  Serial.println((verdata >> 24) & 0xFF, HEX);
  Serial.print(F("[BaseRFID] Version: "));
  Serial.print((verdata >> 16) & 0xFF);
  Serial.print('.');
  Serial.println((verdata >> 8) & 0xFF);

  // Configure once, same as your other reader
  nfc.SAMConfig();

  s_enabled   = false;
  s_available = false;
  s_uidLen    = 0;
}

void enable(bool e) { s_enabled = e; }
bool enabled()      { return s_enabled; }

bool available()    { return s_available; }
uint8_t  uidLen()   { return s_uidLen; }
const uint8_t* uidBytes() { return s_uid; }

void printUID(Stream& s) {
  if (!s_uidLen) { s.println(F("<none>")); return; }
  for (uint8_t i = 0; i < s_uidLen; ++i) {
    if (s_uid[i] < 0x10) s.print('0');
    s.print(s_uid[i], HEX);
    if (i + 1 < s_uidLen) s.print(':');
  }
}

// Run only when enabled(); this is your loop() verbatim.
void tick() {
  if (!s_enabled) return;

  uint8_t uid[7];
  uint8_t uidLength = 0;

  bool success = nfc.readPassiveTargetID(PN532_MIFARE_ISO14443A, uid, &uidLength);

  if (success) {
    bool changed = (uidLength != s_uidLen) || memcmp(uid, s_uid, uidLength) != 0;
    if (changed) {
      memcpy(s_uid, uid, uidLength);
      s_uidLen    = uidLength;
      s_available = true;

      Serial.print(F("[BaseRFID] Tag detected, UID ("));
      Serial.print(uidLength);
      Serial.print(F("): "));
      printUID(Serial);
      Serial.println();
    }
    // EXACT debounce & idle like your other reader
    delay(300);
  }

  // small idle delay for symmetry
  delay(5);
}

// One-shot using the same call pattern; bounded tries
bool detectOnce(uint16_t tries, uint16_t delay_ms) {
  for (uint16_t i = 0; i < tries; ++i) {
    uint8_t uid[7]; uint8_t len = 0;
    if (nfc.readPassiveTargetID(PN532_MIFARE_ISO14443A, uid, &len)) {
      memcpy(s_uid, uid, len);
      s_uidLen    = len;
      s_available = true;

      Serial.print(F("[BaseRFID] Tag detected, UID ("));
      Serial.print(len);
      Serial.print(F("): "));
      printUID(Serial);
      Serial.println();
      return true;
    }
    delay(delay_ms);
  }
  Serial.println(F("[BaseRFID] No tag detected."));
  return false;
}

} // namespace BaseRFID
