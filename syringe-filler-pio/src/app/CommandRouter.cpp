#include <Arduino.h>
#include <Wire.h>

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
#include "hw/BaseRFID.hpp"
#include "hw/Encoder.hpp"  
#include "app/SyringeFillController.hpp"


namespace CommandRouter {

static String input;
static App::SyringeFillController g_sfc;


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
        int idx = input.substring(5).toInt(); // 0..NUM_BASES (0=off)
        if (!Bases::select((uint8_t)idx)) {
          Serial.print("Usage: base <0.."); Serial.print(Pins::NUM_BASES); Serial.println(">");
        } else {
          Serial.print("[Base] selected="); Serial.println(Bases::selected());
        }
      } else if (input == "whichbase") {
        Serial.print("[Base] selected="); Serial.println(Bases::selected());
      } else if (input == "baseoff") {
        Bases::select(0);
        Serial.println("[Base] off");
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
            angle   = rest.toInt();
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
        AxisPair::move2(s);  // toolhead syringe
        Serial.println("OK move2");
      } else if (input.startsWith("move3 ")) {
        if (Bases::selected() == 0) {
          Serial.println("ERR move3: no base selected. Use 'base <1..5>' first.");
        } else {
          String rest = input.substring(6);
          int sp = rest.indexOf(' ');
          if (sp < 0) {
            long steps = rest.toInt();
            AxisPair::move3(steps);
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


      } else if (input == "rfid" || input.startsWith("rfid ")) {
  String arg = "";
  if (input.length() > 5)
    arg = input.substring(5);

  if (arg == "on") {
    RFID::enable(true);
    Serial.println(F("[rfid] polling enabled"));
  } else if (arg == "off") {
    RFID::enable(false);
    Serial.println(F("[rfid] polling disabled"));
  } else if (arg == "once") {
    Serial.println(F("[rfid] detecting once..."));
    RFID::detectOnce(30, 100); // ~3 s total
  } else {
    Serial.println(F("[rfid] usage: rfid on | off | once"));
  }

} else if (input == "rfid2" || input.startsWith("rfid2 ")) {
  String arg = (input.length() > 6) ? input.substring(6) : "";

  if (arg == "on") {
    BaseRFID::enable(true);
    Serial.println(F("[rfid2] polling enabled (SPI PN532)"));
  } 
  else if (arg == "off") {
    BaseRFID::enable(false);
    Serial.println(F("[rfid2] polling disabled"));
  } 
  else if (arg.startsWith("once")) {
    // Example:  rfid2 once 50   → 50 tries with 100 ms delay each
    int tries = 30;
    int spaceIndex = arg.indexOf(' ');
    if (spaceIndex > 0) {
      int val = arg.substring(spaceIndex + 1).toInt();
      if (val > 0 && val < 200) tries = val;
    }
    Serial.print(F("[rfid2] detecting once (~"));
    Serial.print(tries * 100 / 1000.0, 1);
    Serial.println(F(" s window)…"));
    BaseRFID::detectOnce(tries, 100);
  } 
  else {
    Serial.println(F("[rfid2] usage: rfid2 on | off | once [tries]"));
  }

      // ---------------- Encoder ----------------
      } else if (input == "enc" || input.startsWith("enc ")) {
        // enc
        // enc on
        // enc off
        // enc zero
        String arg = "";
        if (input.length() > 4) {
          arg = input.substring(4);
          arg.trim();
          arg.toLowerCase();
        }

        if (arg.length() == 0) {
          long c = EncoderHW::count();
          float mm = EncoderHW::mm();
          Serial.printf("[enc] count=%ld pos=%.3f mm\n", c, mm);
        } else if (arg == "on") {
          EncoderHW::setPolling(true);
          Serial.println("[enc] polling ON");
        } else if (arg == "off") {
          EncoderHW::setPolling(false);
          Serial.println("[enc] polling OFF");
        } else if (arg == "zero" || arg == "reset") {
          EncoderHW::reset();
          Serial.println("[enc] zeroed");
        } else {
          Serial.println("[enc] usage: enc | enc on | enc off | enc zero");
        }

              // ---------------- SFC (Syringe Fill Controller) ----------------
      } else if (input == "sfc.scanBases") {
        Serial.println("[SFC] scanning all base syringes...");
        g_sfc.scanAllBaseSyringes();
        Serial.println("[SFC] scan complete.");
      } else if (input == "sfc.scanToolhead") {
        Serial.println("[SFC] scanning toolhead syringe...");
        g_sfc.scanToolheadSyringe();
        Serial.println("[SFC] scan complete.");
      } else if (input == "sfc.run") {
        Serial.println("[SFC] running current recipe...");
        g_sfc.runRecipe();
        Serial.println("[SFC] recipe done.");
      } else if (input == "sfc.load") {
        // load recipe for whatever toolhead RFID is currently known
        if (g_sfc.loadToolheadRecipeFromFS()) {
          Serial.println("[SFC] recipe loaded from FS.");
        } else {
          Serial.println("[SFC] ERROR: no recipe for this toolhead in FS.");
        }
      } else if (input == "sfc.save") {
        if (g_sfc.saveToolheadRecipeToFS()) {
          Serial.println("[SFC] recipe saved to FS.");
        } else {
          Serial.println("[SFC] ERROR: could not save recipe (missing toolhead RFID?).");
        }
      } else if (input == "sfc.status") {
        Serial.println("[SFC] status:");
        Serial.print("  toolhead RFID: 0x");
        // we don't have a getter, so hacky: add a quick lambda to print
        // but we can’t access private members here, so just say we don’t know:
        Serial.println("(not exposed)");
        Serial.println("  (tip: enable SFC debug in SyringeFillController.cpp to see details.)");

      } else if (input.startsWith("sfc.scanbase ")) {
        uint8_t slot = input.substring(strlen("sfc.scanbase ")).toInt();
        g_sfc.scanBaseSyringe(slot);
      


      // ---------------- SFC CALIBRATION ----------------
      } else if (input == "sfc.cal.t.empty") {
        if (g_sfc.captureToolheadEmpty()) {
          Serial.println("[SFC] toolhead EMPTY point captured.");
        } else {
          Serial.println("[SFC] ERROR: capture empty failed (scan toolhead first).");
        }
      } else if (input.startsWith("sfc.cal.t.full ")) {
        // sfc.cal.t.full 12.0
        float ml = input.substring(strlen("sfc.cal.t.full ")).toFloat();
        if (ml <= 0.0f) {
          Serial.println("[SFC] usage: sfc.cal.t.full <ml>");
        } else {
          if (g_sfc.captureToolheadFull(ml)) {
            Serial.print("[SFC] toolhead FULL point captured at ");
            Serial.print(ml, 3);
            Serial.println(" mL.");
          } else {
            Serial.println("[SFC] ERROR: capture full failed (scan toolhead first).");
          }
        }
      } else if (input == "sfc.cal.t.save") {
        if (g_sfc.saveToolheadCalibration()) {
          Serial.println("[SFC] toolhead calibration saved to NVS.");
        } else {
          Serial.println("[SFC] ERROR: could not save toolhead calibration.");
        }

      // make recipe names clearer
      } else if (input == "sfc.recipe.save") {
        if (g_sfc.saveToolheadRecipeToFS()) {
          Serial.println("[SFC] recipe saved to FS.");
        } else {
          Serial.println("[SFC] ERROR: no toolhead RFID, cannot save recipe.");
        }
      } else if (input == "sfc.recipe.load") {
        if (g_sfc.loadToolheadRecipeFromFS()) {
          Serial.println("[SFC] recipe loaded from FS.");
        } else {
          Serial.println("[SFC] ERROR: no recipe for this toolhead.");
        }

      // ----- SFC base-level calibration -----
      } else if (input == "sfc.base.setemptypos") {
        int8_t slot = g_sfc.currentSlot();
        if (slot < 0) {
          Serial.println("[SFC] no current base – run sfc.scanbase N or gobase N first");
        } else {
          if (g_sfc.captureBaseEmpty((uint8_t)slot)) {
            Serial.print("[SFC] empty position captured for base ");
            Serial.println(slot);
          } else {
            Serial.println("[SFC] failed to capture empty position");
          }
        }
      } else if (input == "sfc.base.save") {
        if (g_sfc.saveCurrentBaseToNVS()) {
          Serial.println("[SFC] current base saved to NVS");
        } else {
          Serial.println("[SFC] ERROR saving current base to NVS");
        }
      } else if (input == "sfc.base.show") {
        int8_t slot = g_sfc.currentSlot();
        if (slot < 0) {
          Serial.println("[SFC] no current base");
        } else {
          g_sfc.printBaseInfo((uint8_t)slot, Serial);
        }


        
      // ---------------- Pot-driven motion (kept) ----------------
      } else if (input.startsWith("potmove ")) {
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
}
 // namespace CommandRouter
