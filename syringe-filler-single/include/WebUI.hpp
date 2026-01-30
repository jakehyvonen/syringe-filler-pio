/**
 * @file WebUI.hpp
 * @brief Embedded HTTP server for base syringe metadata.
 */
#pragma once

#include <Arduino.h>

namespace WebUI {

void begin();
void handle();
void setCurrentRfid(uint32_t rfid);

}  // namespace WebUI
