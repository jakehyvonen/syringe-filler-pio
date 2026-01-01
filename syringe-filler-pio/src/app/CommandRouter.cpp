#include <Arduino.h>
#include <Wire.h>

#include "app/CommandRouter.hpp"
#include "app/DeviceActions.hpp"
#include "app/SyringeFillController.hpp"

#include "hw/Bases.hpp"
#include "hw/Pins.hpp"
#include "hw/Pots.hpp"

namespace CommandRouter {

using App::ActionResult;
using App::PositionResult;
using App::DeviceActions::axis2Position;
using App::DeviceActions::axis3Position;
using App::DeviceActions::coupleSyringes;
using App::DeviceActions::gantryPosition;
using App::DeviceActions::handleBaseRfidCommand;
using App::DeviceActions::handleRfidCommand;
using App::DeviceActions::homeGantry;
using App::DeviceActions::moveAxis2;
using App::DeviceActions::moveAxis3;
using App::DeviceActions::moveAxisSync;
using App::DeviceActions::moveGantryToMm;
using App::DeviceActions::moveGantryToSteps;
using App::DeviceActions::moveToBase;
using App::DeviceActions::potMove;
using App::DeviceActions::raiseToolhead;
using App::DeviceActions::sfcCaptureBaseCalPoint;
using App::DeviceActions::sfcCaptureToolCalPoint;
using App::DeviceActions::sfcClearBaseCalPoints;
using App::DeviceActions::sfcClearToolCalPoints;
using App::DeviceActions::sfcForceBaseCalZero;
using App::DeviceActions::sfcForceToolCalZero;
using App::DeviceActions::sfcLoadRecipe;
using App::DeviceActions::sfcRecipeLoad;
using App::DeviceActions::sfcRecipeSave;
using App::DeviceActions::sfcRunRecipe;
using App::DeviceActions::sfcSaveCurrentBase;
using App::DeviceActions::sfcSaveRecipe;
using App::DeviceActions::sfcSaveToolCalibration;
using App::DeviceActions::sfcScanBase;
using App::DeviceActions::sfcScanBases;
using App::DeviceActions::sfcScanTool;
using App::DeviceActions::sfcShowCurrentBase;
using App::DeviceActions::sfcShowTool;
using App::DeviceActions::showVolumes;
using App::DeviceActions::sfcStatus;
using App::DeviceActions::selectedBase;
using App::DeviceActions::selectBase;
using App::DeviceActions::setAxis23Speed;
using App::DeviceActions::setGantrySpeed;
using App::DeviceActions::setServoAngle;
using App::DeviceActions::setServoAngleSlow;
using App::DeviceActions::setServoPulseRaw;
using App::DeviceActions::readBasePot;
using App::DeviceActions::readPot;

namespace {
struct CommandDescriptor {
  const char *name;
  const char *help;
  void (*handler)(const String &args);
};

static String input;
static App::SyringeFillController g_sfc;

void printStructured(const char *cmd, const ActionResult &res, const String &data = "") {
  Serial.print("{\"cmd\":\"");
  Serial.print(cmd);
  Serial.print("\",\"status\":\"");
  Serial.print(res.ok ? "ok" : "error");
  Serial.print("\"");
  if (res.message.length()) {
    Serial.print(",\"message\":\"");
    Serial.print(res.message);
    Serial.print("\"");
  }
  if (data.length()) {
    Serial.print(",\"data\":");
    Serial.print(data);
  }
  Serial.println("}");
}

void printStructured(const char *cmd, const PositionResult &res) {
  String data = "{";
  data += "\"steps\":" + String(res.steps) + ",";
  data += "\"mm\":" + String(res.mm, 3) + "}";
  printStructured(cmd, static_cast<const ActionResult &>(res), data);
}

void handleSpeed(const String &args) {
  long sps = args.toInt();
  if (sps > 0) {
    printStructured("speed", setGantrySpeed(sps));
  } else {
    printStructured("speed", {false, "missing speed"});
  }
}

void handleHome(const String &args) { printStructured("home", homeGantry()); }

void handlePos(const String &args) { printStructured("pos", gantryPosition()); }

void handleGoto(const String &args) { printStructured("goto", moveGantryToSteps(args.toInt())); }

void handleGotoMm(const String &args) { printStructured("gotomm", moveGantryToMm(args.toFloat())); }

void handleBase(const String &args) {
  int idx = args.toInt();
  printStructured("base", selectBase((uint8_t)idx), "{\"selected\":" + String(selectedBase()) + "}");
}

void handleWhichBase(const String &args) {
  ActionResult res{true, "base query"};
  printStructured("whichbase", res, "{\"selected\":" + String(selectedBase()) + "}");
}

void handleGoBase(const String &args) {
  long target = 0;
  ActionResult res = moveToBase((uint8_t)args.toInt(), target);
  String data;
  if (res.ok) { data = "{\"targetSteps\":" + String(target) + "}"; }
  printStructured("gobase", res, data);
}

void handleServoPulse(const String &args) {
  int sp = args.indexOf(' ');
  if (sp > 0) {
    int ch = args.substring(0, sp).toInt();
    int us = args.substring(sp + 1).toInt();
    printStructured("servopulse", setServoPulseRaw(ch, us));
  } else {
    printStructured("servopulse", {false, "usage: servopulse <channel> <us>"});
  }
}

void handleServo(const String &args) {
  int sp = args.indexOf(' ');
  if (sp > 0) {
    int ch = args.substring(0, sp).toInt();
    int angle = args.substring(sp + 1).toInt();
    printStructured("servo", setServoAngle(ch, angle));
  } else {
    printStructured("servo", {false, "usage: servo <ch> <angle>"});
  }
}

void handleRaise(const String &args) { printStructured("raise", raiseToolhead()); }

void handleServoSlow(const String &args) {
  int sp = args.indexOf(' ');
  if (sp > 0) {
    int ch = args.substring(0, sp).toInt();
    String rest = args.substring(sp + 1);
    int sp2 = rest.indexOf(' ');
    int angle = 0;
    int delayMs = 15;
    if (sp2 > 0) {
      angle = rest.substring(0, sp2).toInt();
      delayMs = rest.substring(sp2 + 1).toInt();
    } else {
      angle = rest.toInt();
    }
    printStructured("servoslow", setServoAngleSlow(ch, angle, delayMs));
  } else {
    printStructured("servoslow", {false, "usage: servoslow <ch> <angle> [delay]"});
  }
}

void handleCouple(const String &args) { printStructured("couple", coupleSyringes()); }

void handleMove2(const String &args) { printStructured("move2", moveAxis2(args.toInt())); }

void handleMove3(const String &args) {
  if (selectedBase() == 0) {
    printStructured("move3", {false, "no base selected"});
    return;
  }
  printStructured("move3", moveAxis3(args.toInt()));
}

void handleSpeed23(const String &args) { printStructured("speed23", setAxis23Speed(args.toInt())); }

void handleM23(const String &args) {
  int sp = args.indexOf(' ');
  if (sp > 0) {
    long s2 = args.substring(0, sp).toInt();
    long s3 = args.substring(sp + 1).toInt();
    printStructured("m23", moveAxisSync(s2, s3, true));
  } else {
    printStructured("m23", {false, "usage: m23 <steps2> <steps3>"});
  }
}

void handlePos2(const String &args) { printStructured("pos2", axis2Position()); }

void handlePos3(const String &args) { printStructured("pos3", axis3Position()); }

void handleRfid(const String &args) { printStructured("rfid", handleRfidCommand(args)); }

void handleRfid2(const String &args) { printStructured("rfid2", handleBaseRfidCommand(args)); }

void handleSfcScanBases(const String &args) { printStructured("sfc.scanBases", sfcScanBases(g_sfc)); }

void handleSfcRun(const String &args) { printStructured("sfc.run", sfcRunRecipe(g_sfc)); }

void handleSfcLoad(const String &args) { printStructured("sfc.load", sfcLoadRecipe(g_sfc)); }

void handleSfcSave(const String &args) { printStructured("sfc.save", sfcSaveRecipe(g_sfc)); }

void handleSfcStatus(const String &args) { printStructured("sfc.status", sfcStatus()); }

void handleSfcScanBase(const String &args) { printStructured("sfc.scanbase", sfcScanBase(g_sfc, (uint8_t)args.toInt())); }

void handleSfcScanTool(const String &args) { printStructured("sfc.scanTool", sfcScanTool(g_sfc)); }

void handleSfcCalTSave(const String &args) { printStructured("sfc.cal.t.save", sfcSaveToolCalibration(g_sfc)); }

void handleSfcCalTPoint(const String &args) {
  if (args.length() == 0) {
    printStructured("sfc.cal.t.point", {false, "usage: sfc.cal.t.point <ml>"} );
    return;
  }
  float ml = args.toFloat();
  if (ml < 0.0f) {
    printStructured("sfc.cal.t.point", {false, "volume must be >= 0"} );
    return;
  }
  printStructured("sfc.cal.t.point", sfcCaptureToolCalPoint(g_sfc, ml));
}

void handleSfcToolShow(const String &args) { printStructured("sfc.tool.show", sfcShowTool(g_sfc)); }

void handleShowVolumes(const String &args) {
  String data;
  ActionResult res = showVolumes(g_sfc, data);
  printStructured("showvolumes", res, data);
}

void handleSfcRecipeSave(const String &args) { printStructured("sfc.recipe.save", sfcRecipeSave(g_sfc)); }

void handleSfcRecipeLoad(const String &args) { printStructured("sfc.recipe.load", sfcRecipeLoad(g_sfc)); }

void handleSfcBaseSave(const String &args) { printStructured("sfc.base.save", sfcSaveCurrentBase(g_sfc)); }

void handleSfcBaseShow(const String &args) { printStructured("sfc.base.show", sfcShowCurrentBase(g_sfc)); }

void handleSfcCalBasePoint(const String &args) {
  int sp = args.indexOf(' ');
  float ml = 0.0f;
  int slot = -1;
  if (sp < 0) {
    if (args.length() == 0) {
      printStructured("sfc.cal.base.point", {false, "usage: sfc.cal.base.point <ml> [slot]"} );
      return;
    }
    ml = args.toFloat();
  } else {
    ml = args.substring(0, sp).toFloat();
    slot = args.substring(sp + 1).toInt();
  }

  if (ml < 0.0f) {
    printStructured("sfc.cal.base.point", {false, "volume must be >= 0"} );
    return;
  }
  printStructured("sfc.cal.base.point", sfcCaptureBaseCalPoint(g_sfc, ml, slot));
}

void handleSfcCalBaseClear(const String &args) {
  printStructured("sfc.cal.base.clear", sfcClearBaseCalPoints(g_sfc));
}

void handleSfcCalToolClear(const String &args) {
  printStructured("sfc.cal.t.clear", sfcClearToolCalPoints(g_sfc));
}

void handleSfcCalBaseForceZero(const String &args) {
  printStructured("sfc.cal.base.force0", sfcForceBaseCalZero(g_sfc));
}

void handleSfcCalToolForceZero(const String &args) {
  printStructured("sfc.cal.t.force0", sfcForceToolCalZero(g_sfc));
}

void handlePotRaw(const String &args) {
  uint16_t counts = 0, scaled = 0;
  ActionResult res = readPot((uint8_t)args.toInt(), counts, scaled);
  String data = "{\"counts\":" + String(counts) + ",\"scaled\":" + String(scaled) + "}";
  printStructured("potraw", res, data);
}

void handleBasePot(const String &args) {
  uint8_t potIdx = 0;
  uint16_t counts = 0, scaled = 0;
  ActionResult res = readBasePot((uint8_t)args.toInt(), potIdx, counts, scaled);
  String data = "{\"base\":" + String(args.toInt()) + ",\"potIndex\":" + String(potIdx) + ",\"counts\":" + String(counts) + ",\"scaled\":" + String(scaled) + "}";
  printStructured("basepot", res, data);
}

void handlePotMove(const String &args) {
  int sp = args.indexOf(' ');
  if (sp < 0) {
    printStructured("potmove", {false, "usage: potmove <target_adc> <sps>"});
    return;
  }
  uint16_t target = (uint16_t)args.substring(0, sp).toInt();
  long sps = args.substring(sp + 1).toInt();
  printStructured("potmove", potMove(target, sps));
}

const CommandDescriptor COMMANDS[] = {
    {"speed", "set gantry speed (steps/sec)", handleSpeed},
    {"home", "home gantry", handleHome},
    {"pos", "report gantry position", handlePos},
    {"goto", "move gantry to steps", handleGoto},
    {"gotomm", "move gantry to mm", handleGotoMm},
    {"base", "select base", handleBase},
    {"whichbase", "report selected base", handleWhichBase},
    {"gobase", "move to base", handleGoBase},
    {"servopulse", "set servo pulse", handleServoPulse},
    {"servo", "set servo angle", handleServo},
    {"raise", "raise toolhead", handleRaise},
    {"servoslow", "set servo slowly", handleServoSlow},
    {"couple", "couple syringes", handleCouple},
    {"couplesyringes", "couple syringes", handleCouple},
    {"move2", "move axis2", handleMove2},
    {"move3", "move axis3", handleMove3},
    {"speed23", "set axis2/3 speed", handleSpeed23},
    {"m23", "move axis2 and 3", handleM23},
    {"pos2", "report axis2 position", handlePos2},
    {"pos3", "report axis3 position", handlePos3},
    {"rfid", "rfid controls", handleRfid},
    {"rfid2", "base rfid controls", handleRfid2},
    {"sfc.scanBases", "scan all base syringes", handleSfcScanBases},
    {"sfc.run", "run current recipe", handleSfcRun},
    {"sfc.load", "load recipe", handleSfcLoad},
    {"sfc.save", "save recipe", handleSfcSave},
    {"sfc.status", "sfc status", handleSfcStatus},
    {"sfc.scanbase", "scan a base slot", handleSfcScanBase},
    {"sfc.scanTool", "scan toolhead syringe", handleSfcScanTool},
    {"sfc.cal.t.save", "save toolhead calibration", handleSfcCalTSave},
    {"sfc.cal.t.point", "add toolhead calibration point <ml>", handleSfcCalTPoint},
    {"sfc.cal.t.clear", "clear toolhead calibration points", handleSfcCalToolClear},
    {"sfc.cal.t.force0", "force toolhead calibration to 0 mL", handleSfcCalToolForceZero},
    {"sfc.tool.show", "print toolhead info", handleSfcToolShow},
    {"showvolumes", "show volumes for scanned syringes", handleShowVolumes},
    {"sfc.recipe.save", "save toolhead recipe", handleSfcRecipeSave},
    {"sfc.recipe.load", "load toolhead recipe", handleSfcRecipeLoad},
    {"sfc.base.save", "save current base", handleSfcBaseSave},
    {"sfc.base.show", "show current base", handleSfcBaseShow},
    {"sfc.cal.base.point", "add base calibration point <ml> [slot]", handleSfcCalBasePoint},
    {"sfc.cal.base.clear", "clear current base calibration points", handleSfcCalBaseClear},
    {"sfc.cal.base.force0", "force base calibration to 0 mL", handleSfcCalBaseForceZero},
    {"potraw", "read pot", handlePotRaw},
    {"basepot", "read base pot", handleBasePot},
    {"potmove", "pot driven move", handlePotMove},
};

const size_t COMMAND_COUNT = sizeof(COMMANDS) / sizeof(COMMANDS[0]);

const CommandDescriptor *lookupCommand(const String &verb) {
  for (size_t i = 0; i < COMMAND_COUNT; ++i) {
    if (verb == COMMANDS[i].name) return &COMMANDS[i];
  }
  return nullptr;
}

}  // namespace

void handleSerial() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n') {
      input.trim();
      if (input.length() == 0) {
        input = "";
        continue;
      }

      int spaceIndex = input.indexOf(' ');
      String verb = (spaceIndex > 0) ? input.substring(0, spaceIndex) : input;
      String args = (spaceIndex > 0) ? input.substring(spaceIndex + 1) : "";
      args.trim();

      const CommandDescriptor *cmd = lookupCommand(verb);
      if (cmd) {
        cmd->handler(args);
      } else {
        ActionResult res{false, "unknown command"};
        printStructured(verb.c_str(), res);
      }

      input = "";
    } else if (c != '\r') {
      input += c;
    }
  }
}

}  // namespace CommandRouter
