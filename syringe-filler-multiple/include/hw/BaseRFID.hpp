/**
 * @file BaseRFID.hpp
 * @brief RFID interface for base tags (primary I2C bus).
 */
#pragma once
#include <Arduino.h>

namespace BaseRFID {

// Base RFID polling and UID access.
void init();
void enable(bool e);
bool enabled();
bool available();
uint8_t uidLen();
const uint8_t* uidBytes();
void tick();
bool detectOnce(uint16_t tries, uint16_t delay_ms);
void printUID(Stream& s);

// Event listener API (callback receives raw UID bytes, length, and user pointer).
typedef void (*TagListener)(const uint8_t* uid, uint8_t len, void* user);

void setListener(TagListener cb, void* user);

} // namespace BaseRFID
