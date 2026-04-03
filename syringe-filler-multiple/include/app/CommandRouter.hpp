/**
 * @file CommandRouter.hpp
 * @brief Serial command parsing and dispatch.
 */
#pragma once
#include <Arduino.h>
namespace CommandRouter {
  void handleSerial();
  bool executeCommandLine(const String &line, String &responseJson);
}
