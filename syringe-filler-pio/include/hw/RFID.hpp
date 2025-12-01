#pragma once
#include <Arduino.h>

namespace RFID {

// Call once from setup()
void init();

// Call every loop(); does nothing unless enabled()
void tick();

// Start/stop the internal polling loop
void enable(bool e);
bool enabled();

// Last-seen UID helpers (updated when a new tag is read)
bool     available();            // true if a new UID was captured since last tick()
uint8_t  uidLen();
const uint8_t* uidBytes();
void     printUID(Stream& s);

// One-shot helper used by the "rfid once" command (tries N times, returns true on hit)
bool     detectOnce(uint16_t tries = 30, uint16_t delay_ms = 100);
// callback gets raw uid bytes and length, plus the user pointer you registered
typedef void (*TagListener)(const uint8_t* uid, uint8_t len, void* user);

void setListener(TagListener cb, void* user);
} // namespace RFID
