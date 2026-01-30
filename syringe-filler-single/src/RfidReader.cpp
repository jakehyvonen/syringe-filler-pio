/**
 * @file RfidReader.cpp
 * @brief PN532 RFID reader helper.
 */
#include "RfidReader.hpp"

#include <Adafruit_PN532.h>
#include <SPI.h>

#include "Pins.hpp"

namespace {
Adafruit_PN532 nfc(Pins::PN532_SS, Pins::PN532_MOSI, Pins::PN532_MISO, Pins::PN532_SCK);
constexpr uint16_t kReadTimeoutMs = 120;
constexpr uint32_t kStatusLogIntervalMs = 5000;

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
  Serial.println(F("[RFID] Initializing PN532 over SPI."));
  Serial.printf("[RFID] SPI pins: SS=%d SCK=%d MOSI=%d MISO=%d RST=%d\n",
                Pins::PN532_SS,
                Pins::PN532_SCK,
                Pins::PN532_MOSI,
                Pins::PN532_MISO,
                Pins::PN532_RST);
  pinMode(Pins::PN532_RST, OUTPUT);
  digitalWrite(Pins::PN532_RST, HIGH);
  delay(10);
  digitalWrite(Pins::PN532_RST, LOW);
  delay(10);
  digitalWrite(Pins::PN532_RST, HIGH);
  delay(50);

  nfc.begin();
  uint32_t verdata = nfc.getFirmwareVersion();
  if (!verdata) {
    Serial.println(F("[RFID] PN532 not detected over SPI. Check wiring, power, and DIP switches."));
    m_available = false;
    return false;
  }

  Serial.print(F("[RFID] PN532 found. IC: 0x"));
  Serial.println((verdata >> 24) & 0xFF, HEX);
  Serial.printf("[RFID] Firmware: 0x%08lX\n", static_cast<unsigned long>(verdata));
  nfc.SAMConfig();
  m_available = true;
  return true;
}

void RfidReader::poll() {
  if (!m_available) return;
  uint8_t uid[7] = {0};
  uint8_t uidLength = 0;
  bool success = nfc.readPassiveTargetID(PN532_MIFARE_ISO14443A, uid, &uidLength, kReadTimeoutMs);
  if (!success) {
    uint32_t now = millis();
    if (now - m_lastStatusLogMs >= kStatusLogIntervalMs) {
      Serial.println(F("[RFID] No tag detected."));
      m_lastStatusLogMs = now;
    }
    return;
  }

  if (uidLength == 0) {
    Serial.println(F("[RFID] Read returned empty UID."));
    return;
  }

  uint32_t tag = uidToRfid(uid, uidLength);
  if (tag != 0 && tag != m_currentTag) {
    m_currentTag = tag;
    Serial.printf("[RFID] Tag detected: 0x%08X (len=%u)\n", m_currentTag, uidLength);
  }
}
