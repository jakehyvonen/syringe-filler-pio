#include "hw/RFID.hpp"
#include "hw/Pins.hpp"
#include "hw/Drivers.hpp"
#include <Adafruit_PN532.h>
#include <Wire.h>

// --------------------------------------------------
// Private globals (file-local)
// --------------------------------------------------
namespace {
  Adafruit_PN532 s_nfc(Pins::PN532_IRQ, Pins::PN532_RST);
  bool     s_inited = false;
  uint32_t s_fw     = 0;
}

// --------------------------------------------------
// Public API
// --------------------------------------------------
namespace RFID {

bool init() {
  // Optional sanity check that I2C bus is up
  Drivers::initI2C(Pins::I2C_SDA, Pins::I2C_SCL, 100000);

  // Optional manual reset if RST pin defined
  if (Pins::PN532_RST >= 0) {
    pinMode(Pins::PN532_RST, OUTPUT);
    digitalWrite(Pins::PN532_RST, HIGH);
    delay(5);
    digitalWrite(Pins::PN532_RST, LOW);
    delay(10);
    digitalWrite(Pins::PN532_RST, HIGH);
    delay(20);
  }

  s_nfc.begin();  // uses I2C internally by default

  s_fw = s_nfc.getFirmwareVersion();  // 0 on failure
  if (!s_fw) {
    s_inited = false;
    return false;
  }

  s_nfc.setPassiveActivationRetries(5);  // faster polling
  s_nfc.SAMConfig();                     // enable normal mode (must call before reads)

  s_inited = true;
  return true;
}

bool present() {
  if (!s_inited) return false;
  uint8_t uid[7];
  uint8_t uidLen = 0;
  bool success = s_nfc.readPassiveTargetID(PN532_MIFARE_ISO14443A, uid, &uidLen, 50);
  return success;
}

int readUID(uint8_t* uidBuf, uint8_t uidBufLen) {
  if (!s_inited || uidBufLen < 7) return -1;

  uint8_t uid[7];
  uint8_t uidLen = 0;
  bool success = s_nfc.readPassiveTargetID(PN532_MIFARE_ISO14443A, uid, &uidLen, 100);
  if (!success) return -1;

  for (uint8_t i = 0; i < uidLen && i < uidBufLen; ++i) {
    uidBuf[i] = uid[i];
  }

  return (int)uidLen;
}

uint32_t firmware() {
  return s_fw;
}

}  // namespace RFID
