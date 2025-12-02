#include <Arduino.h>
#include "app/App.hpp"          // declares namespace App { void setup(); void loop(); }
#include "hw/Pins.hpp"
#include "hw/Drivers.hpp"
#include "hw/Bases.hpp"
#include "hw/Pots.hpp"
#include "motion/Axis.hpp"
#include "motion/AxisPair.hpp"
#include "servo/Toolhead.hpp"
#include "util/Storage.hpp"
#include "app/CommandRouter.hpp"
#include "hw/RFID.hpp"
#include "hw/BaseRFID.hpp"
#include "hw/Encoder.hpp"       // <-- NEW

namespace App {

void setup() {
  Serial.begin(115200);
  delay(200);

  // Pin modes
  pinMode(Pins::STEP1, OUTPUT); pinMode(Pins::DIR1, OUTPUT); pinMode(Pins::EN1, OUTPUT);
  pinMode(Pins::STEP2, OUTPUT); pinMode(Pins::DIR2, OUTPUT); pinMode(Pins::EN2, OUTPUT);
//  pinMode(Pins::STEP3, OUTPUT); pinMode(Pins::DIR3, OUTPUT); pinMode(Pins::EN3, OUTPUT);
  pinMode(Pins::LIMIT, INPUT);
  pinMode(Pins::RAISED, INPUT);

  // Defaults
  digitalWrite(Pins::EN2, Pins::DISABLE_LEVEL);
  //digitalWrite(Pins::EN3, Pins::DISABLE_LEVEL);
  digitalWrite(Pins::EN1, Pins::DISABLE_LEVEL);
  digitalWrite(Pins::DIR1, HIGH);

  // I2C + EEPROM + subsystems
  Drivers::initI2C();
  Drivers::initI2C2();
  //Drivers::i2cScanBoth();
  Util::initStorage();

  Bases::init();
  Axis::init();
  AxisPair::init();

  // --- NEW: encoder init
  EncoderHW::begin();
  Serial.println("Encoder ready. Commands: ENC, ENC ON, ENC OFF, ENC ZERO");

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

  Serial.println("Setup complete. Type commands: on/off, home, move, servo, enc ...");
}

void loop() {
  CommandRouter::handleSerial();
  RFID::tick(); // active only if enabled
  BaseRFID::tick();

  // --- NEW: let encoder do periodic printing when polling is on
  EncoderHW::service();
}

} // namespace App
