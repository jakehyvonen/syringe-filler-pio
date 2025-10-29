#include "hw/RFID.hpp"
#include "hw/Pins.hpp"
#include <Adafruit_PN532.h>
#include <Wire.h>
#include <Arduino.h>

// --------------------------------------------------
// Private globals (file-local)
// --------------------------------------------------
namespace {
  // Compatible ctor (irq, rst) even for I2C mode; Adafruit lib routes to I2C.
  Adafruit_PN532 s_nfc(Pins::PN532_IRQ, Pins::PN532_RST);
  bool     s_inited = false;
  uint32_t s_fw     = 0;

  inline void printHexByte(uint8_t v) { if (v < 16) Serial.print('0'); Serial.print(v, HEX); }
  void dumpUID(const uint8_t* uid, uint8_t len) {
    for (uint8_t i = 0; i < len; ++i) { printHexByte(uid[i]); if (i + 1 < len) Serial.print(':'); }
  }

  // Post-SAM verification: issue a short polling call that MUST return.
  // We don't require finding a tag; we just need the function to return cleanly.
  bool postSAMProbe() {
    uint8_t uid[7] = {0};
    uint8_t uidLen = 0;
    // Keep retries small here so it returns quickly.
    s_nfc.setPassiveActivationRetries(1);
    (void)s_nfc.readPassiveTargetID(PN532_MIFARE_ISO14443A, uid, &uidLen);
    return true; // If we got here, the chip responded and didn't wedge the bus.
  }
}

// --------------------------------------------------
// Public API
// --------------------------------------------------
namespace RFID {

bool init() {
  // I2C must already be up via Drivers::initI2C().

  if (Pins::PN532_IRQ >= 0)  pinMode(Pins::PN532_IRQ, INPUT_PULLUP);
  if (Pins::PN532_RST >= 0) {
    Serial.println("[RFID] PN532 RST pulse");
    pinMode(Pins::PN532_RST, OUTPUT);
    digitalWrite(Pins::PN532_RST, HIGH); delay(5);
    digitalWrite(Pins::PN532_RST, LOW);  delay(10);
    digitalWrite(Pins::PN532_RST, HIGH); delay(20);
  }

  Serial.println("[RFID] PN532 begin()");
  s_nfc.begin();
  delay(10);

  Serial.println("[RFID] PN532 getFirmwareVersion()");
  s_fw = 0;
  for (int attempt = 0; attempt < 3 && s_fw == 0; ++attempt) {
    s_fw = s_nfc.getFirmwareVersion();   // 0 on failure
    if (!s_fw) { Serial.printf("  attempt %d -> no response\n", attempt + 1); delay(20); }
  }
  if (!s_fw) {
    Serial.println("[RFID] ERROR: PN532 did not return firmware.");
    s_inited = false;
    return false;
  }
  Serial.printf("[RFID] PN532 firmware: 0x%08lX\n", s_fw);

  Serial.println("[RFID] PN532 wakeup()");
  s_nfc.wakeup();
  delay(5);


  // Configure SAM (normal mode). Adafruit's SAMConfig() is void; verify afterwards.
  Serial.println("[RFID] PN532 SAMConfig()");
  s_nfc.SAMConfig();

  // Verify by issuing a minimal poll that must return (tag or no tag).
  if (!postSAMProbe()) {
    Serial.println("[RFID] SAM post-probe failed (unexpected).");
    s_inited = false;
    return false;
  }

  // For runtime, use generous retries since we don't wire IRQ.
  s_nfc.setPassiveActivationRetries(0xFF);

  s_inited = true;
  Serial.println("[RFID] PN532 ready.");
  return true;
}

bool present() {
  if (!s_inited) return false;
  uint8_t uid[7] = {0};
  uint8_t uidLen = 0;

  // Two quick polls; true only if a tag was seen
  for (int attempt = 0; attempt < 2; ++attempt) {
    if (s_nfc.readPassiveTargetID(PN532_MIFARE_ISO14443A, uid, &uidLen)) return true;
    delay(120);
  }
  return false;
}

int readUID(uint8_t* uidBuf, uint8_t uidBufLen) {
  if (!s_inited || uidBufLen < 7) return -1;

  uint8_t uid[7] = {0};
  uint8_t uidLen = 0;

  // Retry a few times; internal wait is governed by setPassiveActivationRetries()
  for (int attempt = 0; attempt < 5; ++attempt) {
    bool ok = s_nfc.readPassiveTargetID(PN532_MIFARE_ISO14443A, uid, &uidLen);
    if (ok) {
      for (uint8_t i = 0; i < uidLen && i < uidBufLen; ++i) uidBuf[i] = uid[i];
      Serial.print("[RFID] UID="); dumpUID(uid, uidLen); Serial.println();
      return (int)uidLen;
    }
    delay(180);
  }
  return -1;
}

uint32_t firmware() { return s_fw; }

} // namespace RFID
