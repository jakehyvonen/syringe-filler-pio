#include "hw/BaseRFID.hpp"
#include "hw/Pins.hpp"

#include <Arduino.h>
#include <HardwareSerial.h>

// Seeed PN532 (HSU)
#include <PN532_HSU.h>
#include <PN532.h>

// ---- UART instance for PN532 #2 (Serial1) ----
static HardwareSerial& PN2_SERIAL = Serial1;

// PN532 HSU transport + driver
static PN532_HSU pn2_hsu(PN2_SERIAL);
static PN532     nfc(pn2_hsu);

// State
static bool    s_enabled   = false;
static bool    s_available = false;
static uint8_t s_uid[7]    = {0};
static uint8_t s_uidLen    = 0;

namespace BaseRFID {

void init() {
  // Bring up UART1 on the chosen pins
  // PN532 HSU default baud is 115200 8N1
  PN2_SERIAL.begin(115200, SERIAL_8N1, Pins::PN2_RX, Pins::PN2_TX);

  // PN532 init
  nfc.begin();

  uint32_t verdata = nfc.getFirmwareVersion();
  if (!verdata) {
    Serial.println(F("[BaseRFID] Didn't find PN532 (HSU/UART1). Check wiring & DIP (HSU=OFF/OFF)."));
    return;
  }
  Serial.print(F("[BaseRFID] PN532 found. IC: 0x"));
  Serial.println((verdata >> 24) & 0xFF, HEX);
  Serial.print(F("[BaseRFID] Version: "));
  Serial.print((verdata >> 16) & 0xFF);
  Serial.print('.');
  Serial.println((verdata >> 8) & 0xFF);

  // Same behavior as your working loop: configure once
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

static void printUIDInternal(Stream& s, const uint8_t* uid, uint8_t len) {
  for (uint8_t i = 0; i < len; ++i) {
    if (uid[i] < 0x10) s.print('0');
    s.print(uid[i], HEX);
    if (i + 1 < len) s.print(':');
  }
}

void printUID(Stream& s) {
  if (!s_uidLen) { s.println(F("<none>")); return; }
  printUIDInternal(s, s_uid, s_uidLen);
}

// IDENTICAL behavior to your working pattern, but gated by enable()
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
      printUIDInternal(Serial, uid, uidLength);
      Serial.println();
    }

    // Debounce like your working code
    delay(300);
  }

  // Small idle delay like your working code
  delay(5);
}

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
      printUIDInternal(Serial, uid, len);
      Serial.println();
      return true;
    }
    delay(delay_ms);
  }
  Serial.println(F("[BaseRFID] No tag detected."));
  return false;
}

} // namespace BaseRFID
