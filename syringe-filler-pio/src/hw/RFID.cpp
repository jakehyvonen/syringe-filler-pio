#include "hw/RFID.hpp"
#include "hw/Pins.hpp"

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_PN532.h>
#include <string.h>   // for memcpy, memcmp

// I2C pins from your Pins.hpp
static constexpr int SDA_PIN        = Pins::I2C_SDA;   // 21
static constexpr int SCL_PIN        = Pins::I2C_SCL;   // 22

// We *only* really use RESET in practice, IRQ is not physically wired
static constexpr int PN532_IRQPIN   = 27;              // unused in HW
static constexpr int PN532_RESETPIN = 14;              // connected

// Use the same ctor you had before (library expects this)
static Adafruit_PN532 nfc(PN532_IRQPIN, PN532_RESETPIN);

// ---- State ----
static bool    s_enabled   = false;
static bool    s_available = false;
static uint8_t s_uid[7]    = {0};
static uint8_t s_uidLen    = 0;

namespace RFID {

void init() {
  Serial.println(F("[RFID] init() start"));

  // I2C should already be up elsewhere:
  //   Wire.begin(SDA_PIN, SCL_PIN, 100000);
  //   Wire.setTimeOut(20);
  // DO NOT re-begin Wire here.

  // Hard reset the PN532 so it always starts clean
  pinMode(PN532_RESETPIN, OUTPUT);
  digitalWrite(PN532_RESETPIN, LOW);
  delay(10);
  digitalWrite(PN532_RESETPIN, HIGH);
  delay(50);

  // IRQ pin is NOT wired; don't rely on its state.
  // Just configure it as input so the library doesn't see nonsense.
  pinMode(PN532_IRQPIN, INPUT);

  nfc.begin();
  delay(50);

  uint32_t verdata = nfc.getFirmwareVersion();
  if (!verdata) {
    Serial.println(F("[RFID] Didn't find PN532. Check wiring / DIP / power."));
    s_available = false;
    return;
  }

  Serial.print(F("[RFID] PN532 found. IC: 0x"));
  Serial.println((verdata >> 24) & 0xFF, HEX);
  Serial.print(F("[RFID] Version: "));
  Serial.print((verdata >> 16) & 0xFF);
  Serial.print('.');
  Serial.println((verdata >> 8) & 0xFF);

  // Put PN532 in "read passive" mode and make sure it doesn't retry forever
  nfc.SAMConfig();
  nfc.setPassiveActivationRetries(0x10);   // finite retries, not infinite

  s_enabled   = false;
  s_available = false;
  s_uidLen    = 0;

  Serial.println(F("[RFID] init() done"));
}

void enable(bool e) {
  s_enabled = e;
  Serial.print(F("[RFID] polling "));
  Serial.println(e ? F("ENABLED") : F("DISABLED"));
}

bool enabled()      { return s_enabled; }
bool available()    { return s_available; }
uint8_t  uidLen()   { return s_uidLen; }
const uint8_t* uidBytes() { return s_uid; }

void printUID(Stream& s) {
  if (!s_uidLen) {
    s.println(F("<none>"));
    return;
  }
  for (uint8_t i = 0; i < s_uidLen; ++i) {
    if (s_uid[i] < 0x10) s.print('0');
    s.print(s_uid[i], HEX);
    if (i + 1 < s_uidLen) s.print(':');
  }
}

// Call this from loop(), NOT from an ISR.
// Keep it light and rate-limited so it can't dominate.
void tick() {
  if (!s_enabled) return;

  static uint32_t lastCallMs = 0;
  const uint32_t now = millis();
  if (now - lastCallMs < 100) {
    // max 10 Hz
    return;
  }
  lastCallMs = now;

  Serial.println(F("[RFID] tick()"));

  uint8_t uid[7] = {0};
  uint8_t uidLength = 0;

  // This may block internally while the PN532 waits for a card,
  // but setPassiveActivationRetries() keeps it bounded.
  bool success = nfc.readPassiveTargetID(PN532_MIFARE_ISO14443A, uid, &uidLength);

  Serial.print(F("[RFID] readPassiveTargetID -> success="));
  Serial.print(success);
  Serial.print(F(" len="));
  Serial.println(uidLength);

  if (success && uidLength > 0) {
    bool changed = (uidLength != s_uidLen) || (memcmp(uid, s_uid, uidLength) != 0);
    if (changed) {
      memcpy(s_uid, uid, uidLength);
      s_uidLen    = uidLength;
      s_available = true;

      Serial.print(F("[RFID] Tag detected, UID ("));
      Serial.print(uidLength);
      Serial.print(F("): "));
      printUID(Serial);
      Serial.println();
    }
    // small debounce so the same card doesn't spam endlessly
    delay(300);
  }

  // brief idle delay; most of the time this will be dominated by I2C anyway
  delay(5);
}

// One-shot helper used by "rfid once"
bool detectOnce(uint16_t tries, uint16_t delay_ms) {
  Serial.print(F("[RFID] detectOnce(tries="));
  Serial.print(tries);
  Serial.print(F(", delay="));
  Serial.print(delay_ms);
  Serial.println(F(")"));

  // Re-arm config before a burst of reads
  nfc.SAMConfig();
  nfc.setPassiveActivationRetries(0x10);

  for (uint16_t i = 0; i < tries; ++i) {
    uint8_t uid[7] = {0};
    uint8_t len    = 0;

    bool success = nfc.readPassiveTargetID(PN532_MIFARE_ISO14443A, uid, &len);

    Serial.print(F("[RFID] try #"));
    Serial.print(i);
    Serial.print(F(" success="));
    Serial.print(success);
    Serial.print(F(" len="));
    Serial.println(len);

    if (success && len > 0) {
      memcpy(s_uid, uid, len);
      s_uidLen    = len;
      s_available = true;

      Serial.print(F("[RFID] Tag detected, UID ("));
      Serial.print(len);
      Serial.print(F("): "));
      printUID(Serial);
      Serial.println();
      return true;
    }

    delay(delay_ms);
  }

  Serial.println(F("[RFID] No tag detected."));
  return false;
}

} // namespace RFID
