/**
 * @file WebUI.hpp
 * @brief Minimal HTTP server + UI for recipe management.
 */
#pragma once
#include <Arduino.h>

namespace App {
namespace WebUI {
  void begin();
  void handle();
  void pushSerialLine(const String &line);
} // namespace WebUI
} // namespace App
