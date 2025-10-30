#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_PN532.h>

// I2C pins (ESP32 defaults; match your project)
constexpr int SDA_PIN = 21;
constexpr int SCL_PIN = 22;

// Choose free GPIOs for PN532 IRQ and RESET
constexpr int PN532_IRQ   = 27;  // from PN532 "IRQ" pin
constexpr int PN532_RESET = 14;  // from PN532 "RST" / "RSTO" pin

Adafruit_PN532 nfc(PN532_IRQ, PN532_RESET); // I2C ctor uses IRQ/RESET

void setup() {
  Serial.begin(115200);
  delay(100);

  // If you already call Wire.begin elsewhere, you can omit this:
  Wire.begin(SDA_PIN, SCL_PIN, 100000); // 100 kHz is fine

  // Init PN532
  nfc.begin();

  uint32_t verdata = nfc.getFirmwareVersion();
  if (!verdata) {
    Serial.println(F("Didn't find PN532. Check wiring, DIP switches, and power."));
    while (1) delay(10);
  }
  Serial.print(F("PN532 found. IC: ")); Serial.println((verdata >> 24) & 0xFF, HEX);
  Serial.print(F("Version: ")); Serial.print((verdata >> 16) & 0xFF);
  Serial.print('.');           Serial.println((verdata >> 8) & 0xFF);

  // Configure board to read MiFare cards
  nfc.SAMConfig(); // enables IRQ line for I2C mode

  Serial.println(F("Waiting for an ISO14443A tag..."));
}

void loop() {
  // Look for new cards (non-blocking when using IRQ)
  boolean success;
  uint8_t uid[7];
  uint8_t uidLength;

  success = nfc.readPassiveTargetID(PN532_MIFARE_ISO14443A, uid, &uidLength);

  if (success) {
    Serial.print(F("Tag detected, UID ("));
    Serial.print(uidLength);
    Serial.print(F("): "));
    for (uint8_t i = 0; i < uidLength; i++) {
      if (uid[i] < 0x10) Serial.print('0');
      Serial.print(uid[i], HEX);
      if (i + 1 < uidLength) Serial.print(':');
    }
    Serial.println();

    // Simple debounce so you don’t spam while the tag lingers
    delay(300);
  }

  // Small idle delay; IRQ will wake reads quickly when a tag appears
  delay(5);
}
