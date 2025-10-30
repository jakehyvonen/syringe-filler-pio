#include <Arduino.h>
#include "app/App.hpp"          // declares namespace App { void setup(); void loop(); }
#include "hw/Pins.hpp"
#include "hw/Drivers.hpp"
#include "hw/Bases.hpp"
#include "hw/Pots.hpp"
#include "motion/Axis.hpp"
#include "motion/AxisPair.hpp"
#include "servo/Toolhead.hpp"
#include "util/EEStore.hpp"
#include "app/CommandRouter.hpp"
#include "hw/RFID.hpp"

namespace App {

void setup() {
  Serial.begin(115200);
  delay(200);

  // Pin modes
  pinMode(Pins::STEP12, OUTPUT); pinMode(Pins::DIR12, OUTPUT); pinMode(Pins::EN1, OUTPUT);
  pinMode(Pins::STEP12, OUTPUT); pinMode(Pins::DIR12, OUTPUT); pinMode(Pins::EN2, OUTPUT);
//  pinMode(Pins::STEP3, OUTPUT); pinMode(Pins::DIR3, OUTPUT); pinMode(Pins::EN3, OUTPUT);
  pinMode(Pins::LIMIT, INPUT);
  pinMode(Pins::RAISED, INPUT);

  // Defaults
  digitalWrite(Pins::EN2, Pins::DISABLE_LEVEL);
  //digitalWrite(Pins::EN3, Pins::DISABLE_LEVEL);
  digitalWrite(Pins::EN1, Pins::DISABLE_LEVEL);
  digitalWrite(Pins::DIR12, HIGH);

  // I2C + EEPROM + subsystems
  Drivers::initI2C();
  EEStore::begin();

  Bases::init();
  Axis::init();
  AxisPair::init();

  if (Drivers::hasADS()) {
    Pots::init();
  } else {
    Serial.println("WARN: ADS1115 not found; pot readings disabled.");
  }

  if (Drivers::hasPCA()) {
    if (!Toolhead::ensureRaised()) {
      Serial.println("ERROR: toolhead not raised.");
      //while (true) delay(1000);
    }
  } else {
    Serial.println("WARN: no PCA9685; movement will be blocked by safety where needed.");
  }

  Serial.println("Setup complete. Type commands: on/off, home, move, servo, ...");
}

void loop() {
  CommandRouter::handleSerial();
  RFID::tick(); // active only if enabled

}

} // namespace App
