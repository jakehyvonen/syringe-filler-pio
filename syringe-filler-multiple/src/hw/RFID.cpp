/**
 * @file RFID.cpp
 * @brief Toolhead RFID reader implementation on SPI.
 */
#include "hw/RFID.hpp"
#include "hw/Pins.hpp"

#include <Arduino.h>
#include <SPI.h>
#include <Adafruit_PN532.h>
#include <string.h>
#include <type_traits>
#include <utility>

namespace {
constexpr uint8_t kFailureThreshold = 3;
constexpr uint8_t kPn532SsPin = Pins::PN532_SPI_SS;
constexpr uint8_t kPn532RstPin = Pins::PN532_SPI_RST;
}

static Adafruit_PN532 nfc(kPn532SsPin, &SPI);

static bool s_enabled = false;
static bool s_available = false;
static uint8_t s_uid[7] = {0};
static uint8_t s_uidLen = 0;
static uint8_t s_failures = 0;

static RFID::TagListener s_listener = nullptr;
static void* s_listenerUser = nullptr;

namespace RFID {
namespace {
void pulseReset() {
  pinMode(kPn532RstPin, OUTPUT);
  digitalWrite(kPn532RstPin, LOW);
  delay(10);
  digitalWrite(kPn532RstPin, HIGH);
  delay(50);
}

void handleFailure(const char* reason) {
  Serial.print(F("[RFID] read failed: "));
  Serial.println(reason);
  if (++s_failures < kFailureThreshold) return;
  s_failures = 0;
  pulseReset();
  RFID::reinit();
}
} // namespace

namespace {
template <typename T>
class HasReadPassiveTargetIDTimeout {
 private:
  template <typename U>
  static auto test(int) -> decltype(
      std::declval<U>().readPassiveTargetID(uint8_t{}, static_cast<uint8_t*>(nullptr),
                                            static_cast<uint8_t*>(nullptr), uint16_t{}),
      std::true_type{});
  template <typename>
  static std::false_type test(...);

 public:
  static constexpr bool value = decltype(test<T>(0))::value;
};
} // namespace

void setListener(TagListener cb, void* user) {
  s_listener = cb;
  s_listenerUser = user;
}

void init() {
  pinMode(kPn532RstPin, OUTPUT);
  digitalWrite(kPn532RstPin, HIGH);

  SPI.begin(Pins::PN532_SPI_SCK, Pins::PN532_SPI_MISO, Pins::PN532_SPI_MOSI, Pins::PN532_SPI_SS);
  Serial.printf("[RFID] SPI configured SCK=%d MISO=%d MOSI=%d SS=%d RST=%d\n",
                Pins::PN532_SPI_SCK,
                Pins::PN532_SPI_MISO,
                Pins::PN532_SPI_MOSI,
                Pins::PN532_SPI_SS,
                Pins::PN532_SPI_RST);

  nfc.begin();

  uint32_t verdata = nfc.getFirmwareVersion();
  if (!verdata) {
    Serial.println(F("[RFID] Didn't find PN532 on SPI. Check interface switch and wiring."));
    return;
  }

  Serial.print(F("[RFID] PN532 found. IC: 0x"));
  Serial.println((verdata >> 24) & 0xFF, HEX);
  Serial.print(F("[RFID] Version: "));
  Serial.print((verdata >> 16) & 0xFF);
  Serial.print('.');
  Serial.println((verdata >> 8) & 0xFF);

  nfc.SAMConfig();

  s_enabled = false;
  s_available = false;
  s_uidLen = 0;
  s_failures = 0;
}

void reinit() {
  Serial.println(F("[RFID] reinitializing PN532 (SPI)"));
  nfc.begin();
  nfc.SAMConfig();
}

void enable(bool e) { s_enabled = e; }
bool enabled() { return s_enabled; }

bool available() { return s_available; }
uint8_t uidLen() { return s_uidLen; }
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

void tick() {
  static uint32_t tickCount = 0;
  constexpr uint16_t kReadTimeoutMs = 150;

  if (!s_enabled) return;

  Serial.print("[RFID] tick #");
  Serial.print(tickCount++);
  Serial.println(" (enabled)");

  uint8_t uid[7];
  uint8_t uidLength = 0;

  bool success = false;
  bool timedOut = false;
  uint32_t startMs = millis();
  if constexpr (HasReadPassiveTargetIDTimeout<Adafruit_PN532>::value) {
    success = nfc.readPassiveTargetID(PN532_MIFARE_ISO14443A, uid, &uidLength, kReadTimeoutMs);
    timedOut = (!success && (millis() - startMs) >= kReadTimeoutMs);
  } else {
    success = nfc.readPassiveTargetID(PN532_MIFARE_ISO14443A, uid, &uidLength);
    timedOut = (millis() - startMs) >= kReadTimeoutMs;
  }

  if (timedOut) {
    Serial.println("[RFID] readPassiveTargetID TIMEOUT, resetting PN532");
    pulseReset();
    reinit();
    return;
  }

  if (success) {
    s_failures = 0;
    bool changed = (uidLength != s_uidLen) || memcmp(uid, s_uid, uidLength) != 0;
    if (changed) {
      memcpy(s_uid, uid, uidLength);
      s_uidLen = uidLength;
      s_available = true;

      Serial.print(F("[RFID] Tag detected, UID ("));
      Serial.print(uidLength);
      Serial.print(F("): "));
      printUID(Serial);
      Serial.println();

      if (s_listener) s_listener(s_uid, s_uidLen, s_listenerUser);
    }
    delay(300);
  } else {
    handleFailure("tick");
  }

  delay(5);
}

bool detectOnce(uint16_t tries, uint16_t delay_ms) {
  for (uint16_t i = 0; i < tries; ++i) {
    uint8_t uid[7];
    uint8_t len = 0;
    if (nfc.readPassiveTargetID(PN532_MIFARE_ISO14443A, uid, &len)) {
      s_failures = 0;
      memcpy(s_uid, uid, len);
      s_uidLen = len;
      s_available = true;

      Serial.print(F("[RFID] Tag detected, UID ("));
      Serial.print(len);
      Serial.print(F("): "));
      printUID(Serial);
      Serial.println();

      if (s_listener) s_listener(s_uid, s_uidLen, s_listenerUser);
      return true;
    }
    handleFailure("detectOnce");
    delay(delay_ms);
  }

  Serial.println(F("[RFID] No tag detected."));
  return false;
}

} // namespace RFID
