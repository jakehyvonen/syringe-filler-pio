/**
 * @file RfidReader.cpp
 * @brief PN532 RFID reader helper.
 */
#include "RfidReader.hpp"

#include <Adafruit_PN532.h>
#include <Wire.h>

#include "Pins.hpp"

namespace {
Adafruit_PN532 nfc(Pins::PN532_IRQ, Pins::PN532_RST, &Wire);
constexpr uint16_t kReadTimeoutMs = 120;

uint32_t uidToRfid(const uint8_t* uid, uint8_t len) {
  if (len == 0) return 0;
  uint32_t value = 0;
  uint8_t start = len > 4 ? len - 4 : 0;
  for (uint8_t i = start; i < len; ++i) {
    value = (value << 8) | uid[i];
  }
  return value;
}
}  // namespace

bool RfidReader::begin() {
  Wire.begin(Pins::I2C_SDA, Pins::I2C_SCL);
  Wire.setClock(Pins::I2C_FREQ);
  Wire.setTimeOut(3000);

  nfc.begin();
  uint32_t verdata = nfc.getFirmwareVersion();
  if (!verdata) {
    Serial.println(F("[RFID] PN532 not found on I2C. Check wiring and DIP switches."));
    return false;
  }

  Serial.print(F("[RFID] PN532 found. IC: 0x"));
  Serial.println((verdata >> 24) & 0xFF, HEX);
  nfc.SAMConfig();
  return true;
}

void RfidReader::poll() {
  uint8_t uid[7] = {0};
  uint8_t uidLength = 0;
  bool success = nfc.readPassiveTargetID(PN532_MIFARE_ISO14443A, uid, &uidLength, kReadTimeoutMs);
  if (!success) return;

  uint32_t tag = uidToRfid(uid, uidLength);
  if (tag != 0 && tag != m_currentTag) {
    m_currentTag = tag;
    Serial.printf("[RFID] Tag detected: 0x%08X\n", m_currentTag);
  }
}
