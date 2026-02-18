/**
 * @file WebUI.hpp
 * @brief Embedded HTTP server for base syringe metadata.
 */
#pragma once

#include <Arduino.h>

namespace WebUI {

using FillSpeedGetter = uint32_t (*)();
using FillSpeedSetter = uint32_t (*)(uint32_t);

void begin();
void handle();
void setCurrentRfid(uint32_t rfid);
void setFillSpeedHooks(FillSpeedGetter getter, FillSpeedSetter setter);

}  // namespace WebUI
