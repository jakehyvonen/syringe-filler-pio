#pragma once
#include <Arduino.h>

namespace BaseRFID {

// existing API
void init();
void enable(bool e);
bool enabled();
bool available();
uint8_t uidLen();
const uint8_t* uidBytes();
void tick();
bool detectOnce(uint16_t tries, uint16_t delay_ms);
void printUID(Stream& s);

// NEW: event listener API
// callback gets raw uid bytes and length, plus the user pointer you registered
typedef void (*TagListener)(const uint8_t* uid, uint8_t len, void* user);

void setListener(TagListener cb, void* user);

} // namespace BaseRFID
