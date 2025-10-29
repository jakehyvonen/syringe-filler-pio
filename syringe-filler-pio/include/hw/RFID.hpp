#pragma once
#include <Arduino.h>

namespace RFID {
  // Initialize PN532 over I2C; returns true if firmware responds
  bool init();

  // Returns true if any ISO14443A target is present (quick poll)
  bool present();

  // Reads a tag UID if present. Returns length (>=0) on success, -1 otherwise.
  // uidBuf must be at least 10 bytes (covers 7-byte UIDs).
  int readUID(uint8_t* uidBuf, uint8_t uidBufLen);

  // Returns last known firmware version (chip + revision), or 0 if unknown.
  uint32_t firmware();
}
