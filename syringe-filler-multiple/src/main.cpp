/**
 * @file main.cpp
 * @brief Arduino entry points that delegate to App::setup/loop.
 */
#include <Arduino.h>
#include "app/App.hpp"

// Arduino setup entry point.
void setup() { App::setup(); }
// Arduino loop entry point.
void loop()  { App::loop();  }
