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
#include "hw/RFID.hpp"

namespace CommandRouter {

static String input;

static inline long mmToSteps(float mm) {
  return (long)lround(mm * Pins::STEPS_PER_MM);
}


// ---- Debug helpers for ADS1115 ------------------------------------------------
static constexpr uint8_t ADS_ADDRS[] = { 0x48, 0x49 };   // ADDR->GND, ADDR->VDD
static constexpr float   ADS_LSB_VOLTS_GAIN_ONE = 0.000125f; // 125 µV/LSB at ±4.096V

static bool readADSOnce(uint8_t i2c_addr, int16_t out_counts[4]) {
  Adafruit_ADS1115 ads;
  if (!ads.begin(i2c_addr)) {
    return false;
  }
  ads.setGain(GAIN_ONE); // ±4.096V so 3.3V fits comfortably

  for (int ch = 0; ch < 4; ++ch) {
    int16_t v = ads.readADC_SingleEnded(ch);
    if (v < 0) v = 0; // clamp negatives (shouldn't happen for single-ended)
    out_counts[ch] = v;
  }
  return true;
}

static inline float countsToVolts(int16_t counts) {
  // At GAIN_ONE the ADS1115 returns up to ~32767 for +4.096 V
  // LSB is 125 µV -> volts = counts * 0.000125
  return counts * ADS_LSB_VOLTS_GAIN_ONE;
}

// --- Debug helper: show which EN pins are LOW/HIGH right now.
static void dumpBaseEN(const char* label = "EN") {
  Serial.print("[EN] "); Serial.print(label);
  Serial.print(" sel="); Serial.print(Bases::selected());
  Serial.print("  pins: ");
  for (uint8_t i = 0; i < Pins::NUM_BASES; ++i) {
    int lvl = digitalRead(Pins::BASE_EN[i]);
    Serial.print(Pins::BASE_EN[i]); Serial.print('=');
    Serial.print(lvl);
    if (i + 1 < Pins::NUM_BASES) Serial.print(' ');
  }
  Serial.println();
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
        if (!Bases::select((uint8_t)idx)) {
          Serial.print("Usage: base <0.."); Serial.print(Pins::NUM_BASES); Serial.println(">");
        } else {
          Serial.print("[Base] selected="); Serial.println(Bases::selected());
          dumpBaseEN("after base");
        }
      } else if (input == "whichbase") {
        Serial.print("[Base] selected="); Serial.println(Bases::selected());
        dumpBaseEN("whichbase");
      } else if (input == "baseoff") {
        Bases::select(0);
        dumpBaseEN("after baseoff");
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
        // Axis 2 = TOOLHEAD syringe (EN2)
        AxisPair::move2(s);
        Serial.println("OK move2");
      } else if (input.startsWith("move3 ")) {
        // Accepts: "move3 <steps>" or "move3 <steps> <sps>"
        if (Bases::selected() == 0) {
          Serial.println("ERR move3: no base selected. Use 'base <1..5>' first.");
        } else {
          String rest = input.substring(6);
          int sp = rest.indexOf(' ');
          if (sp < 0) {
            long steps = rest.toInt();
            AxisPair::move3(steps);              // default speed
            Serial.println("OK move3");
          } 
        }

      } else if (input.startsWith("speed23 ")) {
        long sps = input.substring(8).toInt();
        AxisPair::setSpeedSPS(sps);
      } else if (input.startsWith("m23 ")) {
        int sp = input.indexOf(' ', 4);
        if (sp > 0) {
          long s2 = input.substring(4, sp).toInt();
          long s3 = input.substring(sp + 1).toInt();
          if (s3 != 0 && Bases::selected() == 0) {
            Serial.println("ERR m23: base steps requested but no base selected. Use 'base <1..5>'.");
          } else {
            AxisPair::moveSync(s2, s3);
            Serial.println("OK m23");
          }
        } else {
          Serial.println("Usage: m23 <steps2> <steps3>");
        }
      } else if (input.startsWith("link ")) {
        long s = input.substring(5).toInt();
        if (Bases::selected() == 0) {
          Serial.println("ERR link: no base selected. Use 'base <1..5>' first.");
        } else {
          AxisPair::link(s);
          Serial.println("OK link");
        }
      } else if (input == "pos2") {
        float mm = AxisPair::pos2() / Pins::STEPS_PER_MM;
        Serial.print("POS2 steps="); Serial.print(AxisPair::pos2());
        Serial.print("  mm=");       Serial.println(mm, 3);
      } else if (input == "pos3") {
        float mm = AxisPair::pos3() / Pins::STEPS_PER_MM;
        Serial.print("POS3 steps="); Serial.print(AxisPair::pos3());
        Serial.print("  mm=");       Serial.println(mm, 3);

      // ---------------- Pots / ADS1115 ----------------
      } else if (input == "pots") {
        // Snapshot ADS1115 channels (both chips), and also print current Pots API view.
        for (uint8_t i = 0; i < sizeof(ADS_ADDRS); ++i) {
          uint8_t addr = ADS_ADDRS[i];
          int16_t counts[4] = {0,0,0,0};
          bool ok = readADSOnce(addr, counts);

          Serial.print("ADS@0x"); Serial.print(addr, HEX);
          if (!ok) {
            Serial.println("  (not found)");
            continue;
          }
          Serial.println();

          for (int ch = 0; ch < 4; ++ch) {
            float volts = countsToVolts(counts[ch]);
            Serial.print("  A"); Serial.print(ch);
            Serial.print(": counts="); Serial.print(counts[ch]);
            Serial.print("  volts="); Serial.println(volts, 5);
          }          
        }
      }
      
      else if (input.startsWith("potmove ")) {
        // "potmove <target_adc 0-1023> <sps>"  (toolhead/Axis2 only)
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
