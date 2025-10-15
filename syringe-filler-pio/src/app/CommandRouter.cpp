#include <Arduino.h>
#include "app/CommandRouter.hpp"

#include "hw/Pins.hpp"
#include "hw/Bases.hpp"
#include "hw/Pots.hpp"
#include "hw/Drivers.hpp"

#include "servo/Toolhead.hpp"

#include "motion/Axis.hpp"
#include "motion/AxisPair.hpp"
#include "motion/Homing.hpp"

namespace CommandRouter {

static String input;

static inline long mmToSteps(float mm) {
  return (long)lround(mm * Pins::STEPS_PER_MM);
}

void handleSerial() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n') {
      input.trim();
      if (input.length() == 0) { input = ""; continue; }

      Serial.print("received: ");
      Serial.println(input);

      // ---------------- Axis #1 (gantry) ----------------
      if (input == "on") {
        Axis::enable(true);
        Serial.println("Motor ON");
      } else if (input == "off") {
        Axis::enable(false);
        Serial.println("Motor OFF");
      } else if (input.startsWith("speed ")) {       // free-run style speed setter (compat)
        long sps = input.substring(6).toInt();
        if (sps > 0) {
          Axis::setSpeedSPS(sps);
          Serial.print("Speed set to "); Serial.print(sps); Serial.println(" steps/sec");
        }
      } else if (input.startsWith("dir ")) {
        int d = input.substring(4).toInt();
        Axis::dir(d ? HIGH : LOW);
        Serial.print("Direction set to "); Serial.println(d);
      } else if (input == "home") {
        Serial.println("Homing...");
        Homing::home();
      } else if (input == "pos") {
        float mm = Axis::current() / Pins::STEPS_PER_MM;
        Serial.print("POS steps="); Serial.print(Axis::current());
        Serial.print("  mm=");      Serial.println(mm, 3);
      } else if (input.startsWith("move ")) {        // relative in steps
        long s = input.substring(5).toInt();
        Axis::enable(true);
        Axis::moveSteps(s);
        Axis::enable(false);
        Serial.println("OK");
      } else if (input.startsWith("movemm ")) {      // relative in mm
        float mm = input.substring(7).toFloat();
        Axis::enable(true);
        Axis::moveSteps(mmToSteps(mm));
        Axis::enable(false);
        Serial.println("OK");
      } else if (input.startsWith("goto ")) {        // absolute in steps
        long tgt = input.substring(5).toInt();
        if (tgt < Pins::MIN_POS_STEPS) tgt = Pins::MIN_POS_STEPS;
        if (tgt > Pins::MAX_POS_STEPS) tgt = Pins::MAX_POS_STEPS;
        Axis::enable(true);
        Axis::moveTo(tgt);
        Axis::enable(false);
        Serial.println("OK");
      } else if (input.startsWith("gotomm ")) {      // absolute in mm
        float mm = input.substring(7).toFloat();
        long tgt = mmToSteps(mm);
        if (tgt < Pins::MIN_POS_STEPS) tgt = Pins::MIN_POS_STEPS;
        if (tgt > Pins::MAX_POS_STEPS) tgt = Pins::MAX_POS_STEPS;
        Axis::enable(true);
        Axis::moveTo(tgt);
        Axis::enable(false);
        Serial.println("OK");

      // ---------------- Bases ----------------
      } else if (input.startsWith("base ")) {
        int idx = input.substring(5).toInt(); // 0..NUM_BASES
        if (!Bases::select((uint8_t)idx)) Serial.println("Usage: base <0..5>");
      } else if (input.startsWith("gobase ")) {
        long idx = input.substring(7).toInt(); // 1..NUM_BASES
        if (idx < 1 || idx > Pins::NUM_BASES) {
          Serial.print("Usage: gobase <1.."); Serial.print(Pins::NUM_BASES); Serial.println(">");
        } else {
          long tgt = Bases::getPos((uint8_t)idx);
          if (tgt < 0) {
            Serial.println("ERROR: failed to move to base (index).");
          } else {
            if (tgt < Pins::MIN_POS_STEPS || tgt > Pins::MAX_POS_STEPS) {
              Serial.println("ERROR: base target outside soft limits.");
            } else {
              Axis::enable(true);
              Axis::moveTo(tgt);
              Axis::enable(false);
              Serial.print("At Base #"); Serial.print(idx);
              Serial.print(" (steps="); Serial.print(tgt); Serial.println(")");
            }
          }
        }

      // ---------------- Servos / Toolhead ----------------
      } else if (input.startsWith("servopulse")) {
        // "servopulse <channel> <us>"
        int firstSpace = input.indexOf(' ');
        if (firstSpace >= 0) {
          String rest = input.substring(firstSpace + 1);
          int sp = rest.indexOf(' ');
          if (sp > 0) {
            int ch  = rest.substring(0, sp).toInt();
            int us  = rest.substring(sp + 1).toInt();
            Toolhead::setPulseRaw(ch, us);
            Serial.print("Servo "); Serial.print(ch);
            Serial.print(" set to raw pulse length "); Serial.println(us);
          } else Serial.println("Usage: servopulse <channel> <us>");
        } else Serial.println("Usage: servopulse <channel> <us>");
      } else if (input.startsWith("servo ")) {
        // "servo <channel> <angle>"
        int firstSpace = input.indexOf(' ', 6);
        if (firstSpace > 0) {
          int channel = input.substring(6, firstSpace).toInt();
          int angle   = input.substring(firstSpace + 1).toInt();
          if (channel < 0 || channel > 15) {
            Serial.println("Error: channel must be 0–15");
          } else {
            Toolhead::setAngle(channel, angle);
          }
        } else {
          Serial.println("Usage: servo <channel 0-15> <angle 0-180>");
        }
      } else if (input == "raise") {
        Toolhead::raise();
      } else if (input.startsWith("servoslow ")) {
        // "servoslow <channel> <angle> [delay_ms]"
        int firstSpace = input.indexOf(' ', 10);
        if (firstSpace > 0) {
          int channel = input.substring(10, firstSpace).toInt();
          String rest = input.substring(firstSpace + 1);
          int secondSpace = rest.indexOf(' ');
          int angle, delayMs;
          if (secondSpace > 0) {
            angle   = rest.substring(0, secondSpace).toInt();
            delayMs = rest.substring(secondSpace + 1).toInt();
          } else {
            angle = rest.toInt();
            delayMs = 15;
          }
          if (channel < 0 || channel > 15) Serial.println("Error: channel must be 0–15");
          else Toolhead::setAngleSlow(channel, angle, delayMs);
        } else {
          Serial.println("Usage: servoslow <channel> <angle 0-180> [delay_ms]");
        }
      } else if (input == "couple" || input == "couplesyringes") {
        Toolhead::couple();
        Serial.println("OK coupleSyringes");

      // ---------------- Axis 2 & 3 ----------------
      } else if (input.startsWith("move2 ")) {
        long s = input.substring(6).toInt();
        AxisPair::move2(s);
        Serial.println("OK move2");
      } else if (input.startsWith("move3 ")) {
        long s = input.substring(6).toInt();
        AxisPair::move3(s);
        Serial.println("OK move3");
      } else if (input.startsWith("speed23 ")) {
        long sps = input.substring(8).toInt();
        AxisPair::setSpeedSPS(sps);
      } else if (input.startsWith("m23 ")) {
        int sp = input.indexOf(' ', 4);
        if (sp > 0) {
          long s2 = input.substring(4, sp).toInt();
          long s3 = input.substring(sp + 1).toInt();
          AxisPair::moveSync(s2, s3);
          Serial.println("OK m23");
        } else {
          Serial.println("Usage: m23 <steps2> <steps3>");
        }
      } else if (input.startsWith("link ")) {
        long s = input.substring(5).toInt();
        AxisPair::link(s);
        Serial.println("OK link");
      } else if (input == "pos2") {
        float mm = AxisPair::pos2() / Pins::STEPS_PER_MM;
        Serial.print("POS2 steps="); Serial.print(AxisPair::pos2());
        Serial.print("  mm=");       Serial.println(mm, 3);
      } else if (input == "pos3") {
        float mm = AxisPair::pos3() / Pins::STEPS_PER_MM;
        Serial.print("POS3 steps="); Serial.print(AxisPair::pos3());
        Serial.print("  mm=");       Serial.println(mm, 3);

      // ---------------- Pots / ADS1115 ----------------
      } else if (input == "pots" || input == "potlist") {
        for (uint8_t i = 0; i < Pots::NUM_POTS; ++i) {
          Serial.print("pot["); Serial.print(i); Serial.print("] raw=");
          Serial.print(Pots::raw(i));
          Serial.print(" filt=");
          Serial.print(Pots::filt(i));
          Serial.print(" pct=");
          Serial.println(Pots::percent(i), 1);
        }
      } else if (input.startsWith("potmove ")) {
        // "potmove <target_adc 0-1023> <sps>"
        int sp = input.indexOf(' ', 8);
        if (sp < 0) {
          Serial.println("Usage: potmove <target_adc 0-1023> <sps>");
        } else {
          uint16_t target = (uint16_t)input.substring(8, sp).toInt();
          long sps = input.substring(sp + 1).toInt();
          bool ok = AxisPair::move2UntilPotSimple(target, sps);
          if (ok) Serial.println("OK potmove");
          else    Serial.println("ERR potmove");
        }

      } else {
        Serial.println("Invalid command.");
      }

      input = "";
    } else if (c != '\r') {
      input += c;
    }
  }
}

} // namespace CommandRouter
