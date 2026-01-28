/**
 * @file CommandRouter.cpp
 * @brief Serial command parsing and JSON-formatted responses.
 */
#include <Arduino.h>
#include <Wire.h>

#include "app/CommandRouter.hpp"
#include "app/DeviceActions.hpp"
#include "app/SyringeFillController.hpp"
#include "app/WifiManager.hpp"

#include "hw/Bases.hpp"
#include "hw/Pins.hpp"
#include "hw/Pots.hpp"
#include "util/Storage.hpp"

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
using App::DeviceActions::sfcSaveRecipe;
using App::DeviceActions::sfcScanBase;
using App::DeviceActions::sfcScanBases;
using App::DeviceActions::sfcScanTool;
using App::DeviceActions::sfcTransferFromBase;
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
using App::DeviceActions::readAllPots;
using App::DeviceActions::readPot;
using App::DeviceActions::i2cScanBoth;

namespace {
struct CommandDescriptor {
  const char *name;
  const char *help;
  void (*handler)(const String &args);
};

static String input;
static App::SyringeFillController g_sfc;
static App::WifiManager g_wifi;

// Emit a JSON response for a simple action result.
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

// Emit a JSON response for a position result with steps/mm.
void printStructured(const char *cmd, const PositionResult &res) {
  String data = "{";
  data += "\"steps\":" + String(res.steps) + ",";
  data += "\"mm\":" + String(res.mm, 3) + "}";
  printStructured(cmd, static_cast<const ActionResult &>(res), data);
}

bool parseRecipeIdArg(const String &args, uint32_t &recipeIdOut) {
  if (args.length() == 0) return false;
  char *end = nullptr;
  recipeIdOut = strtoul(args.c_str(), &end, 16);
  if (end == args.c_str()) return false;
  return recipeIdOut != 0;
}

void handleWifiStatus(const String &args) {
  (void)args;
  printStructured("wifi.status", {true, ""}, g_wifi.buildStatusJson());
}

void handleWifiSet(const String &args) {
  int sp = args.indexOf(' ');
  String ssid = (sp < 0) ? args : args.substring(0, sp);
  String password = (sp < 0) ? "" : args.substring(sp + 1);
  ssid.trim();
  password.trim();
  if (ssid.length() == 0) {
    printStructured("wifi.set", {false, "usage: wifi.set <ssid> [password]"});
    return;
  }

  if (!Util::saveWiFiCredentials(ssid, password)) {
    printStructured("wifi.set", {false, "failed to save credentials"});
    return;
  }

  bool connected = g_wifi.connect(ssid, password);
  printStructured("wifi.set", {connected, connected ? "connected" : "connect failed"}, g_wifi.buildStatusJson());
}

void handleWifiConnect(const String &args) {
  (void)args;
  String ssid;
  String password;
  if (!Util::loadWiFiCredentials(ssid, password)) {
    printStructured("wifi.connect", {false, "no saved credentials"});
    return;
  }
  bool connected = g_wifi.connect(ssid, password);
  printStructured("wifi.connect", {connected, connected ? "connected" : "connect failed"}, g_wifi.buildStatusJson());
}

void handleWifiClear(const String &args) {
  (void)args;
  if (!Util::clearWiFiCredentials()) {
    printStructured("wifi.clear", {false, "failed to clear credentials"});
    return;
  }
  printStructured("wifi.clear", {true, "credentials cleared"});
}

void handleWifiAp(const String &args) {
  (void)args;
  g_wifi.startAccessPoint();
  printStructured("wifi.ap", {true, "ap started"});
}

void handleWifiScan(const String &args) {
  (void)args;
  printStructured("wifi.scan", {true, ""}, g_wifi.buildScanJson());
}

// Handle "speed" command for gantry speed changes.
void handleSpeed(const String &args) {
  long sps = args.toInt();
  if (sps > 0) {
    printStructured("speed", setGantrySpeed(sps));
  } else {
    printStructured("speed", {false, "missing speed"});
  }
}

// Handle "home" command for gantry homing.
void handleHome(const String &args) { printStructured("home", homeGantry()); }

// Handle "pos" command for gantry position reporting.
void handlePos(const String &args) { printStructured("pos", gantryPosition()); }

// Handle "goto" command for absolute step moves.
void handleGoto(const String &args) { printStructured("goto", moveGantryToSteps(args.toInt())); }

// Handle "gotomm" command for absolute mm moves.
void handleGotoMm(const String &args) { printStructured("gotomm", moveGantryToMm(args.toFloat())); }

// Handle "base" command to select a base.
void handleBase(const String &args) {
  int idx = args.toInt();
  printStructured("base", selectBase((uint8_t)idx), "{\"selected\":" + String(selectedBase()) + "}");
}

// Handle "whichbase" command to report current base.
void handleWhichBase(const String &args) {
  ActionResult res{true, "base query"};
  printStructured("whichbase", res, "{\"selected\":" + String(selectedBase()) + "}");
}

// Handle "gobase" command to move to a stored base position.
void handleGoBase(const String &args) {
  long target = 0;
  ActionResult res = moveToBase((uint8_t)args.toInt(), target);
  String data;
  if (res.ok) { data = "{\"targetSteps\":" + String(target) + "}"; }
  printStructured("gobase", res, data);
}

// Handle "servopulse" command to set raw servo pulse width.
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

// Handle "servo" command to set a servo angle.
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

// Handle "raise" command to lift the toolhead.
void handleRaise(const String &args) { printStructured("raise", raiseToolhead()); }

// Handle "servoslow" command for slow servo sweeps.
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

// Handle "couple" command for toolhead coupling sequence.
void handleCouple(const String &args) { printStructured("couple", coupleSyringes()); }

// Handle "move2" command for axis 2 motion.
void handleMove2(const String &args) { printStructured("move2", moveAxis2(args.toInt())); }

// Handle "move3" command for axis 3 motion.
void handleMove3(const String &args) {
  if (selectedBase() == 0) {
    printStructured("move3", {false, "no base selected"});
    return;
  }
  printStructured("move3", moveAxis3(args.toInt()));
}

// Handle "speed23" command for axis 2/3 speed changes.
void handleSpeed23(const String &args) { printStructured("speed23", setAxis23Speed(args.toInt())); }

// Handle "m23" command for synchronized axis 2/3 moves.
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

// Handle "pos2" command for axis 2 position reporting.
void handlePos2(const String &args) { printStructured("pos2", axis2Position()); }

// Handle "pos3" command for axis 3 position reporting.
void handlePos3(const String &args) { printStructured("pos3", axis3Position()); }

// Handle "rfid" command for toolhead RFID control.
void handleRfid(const String &args) { printStructured("rfid", handleRfidCommand(args)); }

// Handle "rfid2" command for base RFID control.
void handleRfid2(const String &args) { printStructured("rfid2", handleBaseRfidCommand(args)); }

// Handle "sfc.scanBases" command to scan all bases.
void handleSfcScanBases(const String &args) { printStructured("sfc.scanBases", sfcScanBases(g_sfc)); }

// Handle "sfc.scanAllBaseSyringes" command to scan all bases.
void handleSfcScanAllBaseSyringes(const String &args) {
  printStructured("sfc.scanAllBaseSyringes", sfcScanBases(g_sfc));
}

// Handle "sfc.run" command to execute the recipe.
void handleSfcRun(const String &args) { printStructured("sfc.run", sfcRunRecipe(g_sfc)); }

// Handle "sfc.load" command to load a recipe.
void handleSfcLoad(const String &args) {
  uint32_t recipeId = 0;
  if (!parseRecipeIdArg(args, recipeId)) {
    printStructured("sfc.load", {false, "usage: sfc.load <recipe_id>"});
    return;
  }
  printStructured("sfc.load", sfcLoadRecipe(g_sfc, recipeId));
}

// Handle "sfc.save" command to save the recipe.
void handleSfcSave(const String &args) {
  uint32_t recipeId = 0;
  if (!parseRecipeIdArg(args, recipeId)) {
    printStructured("sfc.save", {false, "usage: sfc.save <recipe_id>"});
    return;
  }
  printStructured("sfc.save", sfcSaveRecipe(g_sfc, recipeId));
}

// Handle "sfc.status" command to report controller status.
void handleSfcStatus(const String &args) {
  (void)args;
  String data;
  ActionResult res = sfcStatus(g_sfc, data);
  printStructured("sfc.status", res, data);
}

// Handle "sfc.scanbase" command to scan a single base slot.
void handleSfcScanBase(const String &args) { printStructured("sfc.scanbase", sfcScanBase(g_sfc, (uint8_t)args.toInt())); }

// Handle "sfc.scanTool" command to scan the toolhead.
void handleSfcScanTool(const String &args) { printStructured("sfc.scanTool", sfcScanTool(g_sfc)); }

// Handle "transfer" command to move volume from a base syringe to the toolhead.
void handleTransfer(const String &args) {
  int sp = args.indexOf(' ');
  if (sp < 0) {
    printStructured("transfer", {false, "usage: transfer <slot> <ml>"});
    return;
  }
  int slot = args.substring(0, sp).toInt();
  float ml = args.substring(sp + 1).toFloat();
  if (slot < 0) {
    printStructured("transfer", {false, "slot must be >= 0"});
    return;
  }
  if (ml < 0.0f) {
    printStructured("transfer", {false, "volume must be >= 0"});
    return;
  }
  printStructured("transfer", sfcTransferFromBase(g_sfc, (uint8_t)slot, ml));
}

// Handle "cal.tool.point" command to add a toolhead calibration point.
void handleSfcCalTPoint(const String &args) {
  if (args.length() == 0) {
    printStructured("cal.tool.point", {false, "usage: cal.tool.point <ml>"} );
    return;
  }
  float ml = args.toFloat();
  if (ml < 0.0f) {
    printStructured("cal.tool.point", {false, "volume must be >= 0"} );
    return;
  }
  printStructured("cal.tool.point", sfcCaptureToolCalPoint(g_sfc, ml));
}

// Handle "sfc.tool.show" command to print toolhead info.
void handleSfcToolShow(const String &args) { printStructured("sfc.tool.show", sfcShowTool(g_sfc)); }

// Handle "showvolumes" command to output volume report JSON.
void handleShowVolumes(const String &args) {
  String data;
  ActionResult res = showVolumes(g_sfc, data);
  printStructured("showvolumes", res, data);
}

// Handle "sfc.recipe.save" command to save the recipe.
void handleSfcRecipeSave(const String &args) {
  uint32_t recipeId = 0;
  if (!parseRecipeIdArg(args, recipeId)) {
    printStructured("sfc.recipe.save", {false, "usage: sfc.recipe.save <recipe_id>"});
    return;
  }
  printStructured("sfc.recipe.save", sfcRecipeSave(g_sfc, recipeId));
}

// Handle "sfc.recipe.load" command to load the recipe.
void handleSfcRecipeLoad(const String &args) {
  uint32_t recipeId = 0;
  if (!parseRecipeIdArg(args, recipeId)) {
    printStructured("sfc.recipe.load", {false, "usage: sfc.recipe.load <recipe_id>"});
    return;
  }
  printStructured("sfc.recipe.load", sfcRecipeLoad(g_sfc, recipeId));
}

// Handle "sfc.base.show" command to print current base info.
void handleSfcBaseShow(const String &args) { printStructured("sfc.base.show", sfcShowCurrentBase(g_sfc)); }

// Handle "cal.base.point" command to add a base calibration point.
void handleSfcCalBasePoint(const String &args) {
  int sp = args.indexOf(' ');
  float ml = 0.0f;
  int slot = -1;
  if (sp < 0) {
    if (args.length() == 0) {
      printStructured("cal.base.point", {false, "usage: cal.base.point <ml> [slot]"} );
      return;
    }
    ml = args.toFloat();
  } else {
    ml = args.substring(0, sp).toFloat();
    slot = args.substring(sp + 1).toInt();
  }

  if (ml < 0.0f) {
    printStructured("cal.base.point", {false, "volume must be >= 0"} );
    return;
  }
  printStructured("cal.base.point", sfcCaptureBaseCalPoint(g_sfc, ml, slot));
}

// Handle "cal.base.clear" command to clear base calibration points.
void handleSfcCalBaseClear(const String &args) {
  printStructured("cal.base.clear", sfcClearBaseCalPoints(g_sfc));
}

// Handle "cal.tool.clear" command to clear toolhead calibration points.
void handleSfcCalToolClear(const String &args) {
  printStructured("cal.tool.clear", sfcClearToolCalPoints(g_sfc));
}

// Handle "cal.base.force0" command to force base calibration through zero.
void handleSfcCalBaseForceZero(const String &args) {
  printStructured("cal.base.force0", sfcForceBaseCalZero(g_sfc));
}

// Handle "cal.tool.force0" command to force toolhead calibration through zero.
void handleSfcCalToolForceZero(const String &args) {
  printStructured("cal.tool.force0", sfcForceToolCalZero(g_sfc));
}

// Handle "potraw" command to read a pot value.
void handlePotRaw(const String &args) {
  uint16_t counts = 0, scaled = 0;
  ActionResult res = readPot((uint8_t)args.toInt(), counts, scaled);
  String data = "{\"counts\":" + String(counts) + ",\"scaled\":" + String(scaled) + "}";
  printStructured("potraw", res, data);
}

// Handle "basepot" command to read a base pot value.
void handleBasePot(const String &args) {
  uint8_t potIdx = 0;
  uint16_t counts = 0, scaled = 0;
  ActionResult res = readBasePot((uint8_t)args.toInt(), potIdx, counts, scaled);
  float percent = Pots::ratioFromCounts(counts);
  float ratio = percent / 100.0f;
  String data = "{\"base\":" + String(args.toInt()) + ",\"potIndex\":" + String(potIdx) + ",\"counts\":" + String(counts) + ",\"scaled\":" + String(scaled);
  data += ",\"ratio\":" + String(ratio, 5) + ",\"percent\":" + String(percent, 3) + "}";
  printStructured("basepot", res, data);
}

// Handle "pots" command to report all pot values.
void handlePotReport(const String &args) {
  (void)args;
  String data;
  ActionResult res = readAllPots(data);
  printStructured("pots", res, data);
}

// Handle "potmove" command to run pot-driven motion.
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

// Handle "i2cscan" command to scan both I2C buses.
void handleI2cScan(const String &args) {
  (void)args;
  printStructured("i2cscan", i2cScanBoth());
}

void handleSfcRecipeList(const String &args) {
  (void)args;
  String data;
  ActionResult res{true, ""};
  if (!Util::listRecipes(data)) {
    res = {false, "unable to list recipes"};
  }
  printStructured("sfc.recipe.list", res, data);
}

void handleSfcRecipeListDesc(const String &args) {
  (void)args;
  uint32_t ids[32];
  const size_t maxIds = sizeof(ids) / sizeof(ids[0]);
  size_t count = 0;
  ActionResult res{true, ""};
  if (!Util::listRecipeIds(ids, maxIds, count)) {
    res = {false, "unable to list recipes"};
    printStructured("sfc.recipe.list.desc", res);
    return;
  }
  for (size_t i = 0; i + 1 < count; ++i) {
    for (size_t j = i + 1; j < count; ++j) {
      if (ids[j] > ids[i]) {
        uint32_t tmp = ids[i];
        ids[i] = ids[j];
        ids[j] = tmp;
      }
    }
  }
  String data = "[";
  for (size_t i = 0; i < count; ++i) {
    char buf[16];
    snprintf(buf, sizeof(buf), "%08X", ids[i]);
    if (i > 0) data += ",";
    data += "\"";
    data += buf;
    data += "\"";
  }
  data += "]";
  printStructured("sfc.recipe.list.desc", res, data);
}

void handleSfcRecipeShow(const String &args) {
  uint32_t recipeId = 0;
  if (!parseRecipeIdArg(args, recipeId)) {
    printStructured("sfc.recipe.show", {false, "usage: sfc.recipe.show <recipe_id>"});
    return;
  }
  String data;
  ActionResult res{true, ""};
  if (!Util::readRecipeJson(recipeId, data)) {
    res = {false, "recipe not found"};
  }
  printStructured("sfc.recipe.show", res, data);
}

void handleSfcRecipeDelete(const String &args) {
  uint32_t recipeId = 0;
  if (!parseRecipeIdArg(args, recipeId)) {
    printStructured("sfc.recipe.delete", {false, "usage: sfc.recipe.delete <recipe_id>"});
    return;
  }
  ActionResult res{true, ""};
  if (!Util::deleteRecipe(recipeId)) {
    res = {false, "delete failed"};
  }
  String data;
  if (res.ok) {
    char buf[16];
    snprintf(buf, sizeof(buf), "%08X", recipeId);
    data = "{\"recipe_id\":\"";
    data += buf;
    data += "\"}";
  }
  printStructured("sfc.recipe.delete", res, data);
}

const CommandDescriptor COMMANDS[] = {
    {"wifi.status", "show WiFi status, IP, and mDNS hostname", handleWifiStatus},
    {"wifi.set", "save WiFi credentials and connect", handleWifiSet},
    {"wifi.connect", "connect using saved WiFi credentials", handleWifiConnect},
    {"wifi.clear", "clear saved WiFi credentials", handleWifiClear},
    {"wifi.ap", "start access point for setup", handleWifiAp},
    {"wifi.scan", "scan for nearby WiFi networks", handleWifiScan},
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
    {"move2", "move axis2", handleMove2},
    {"move3", "move axis3", handleMove3},
    {"speed23", "set axis2/3 speed", handleSpeed23},
    {"m23", "move axis2 and 3", handleM23},
    {"pos2", "report axis2 position", handlePos2},
    {"pos3", "report axis3 position", handlePos3},
    {"rfid", "rfid controls", handleRfid},
    {"rfid2", "base rfid controls", handleRfid2},
    {"sfc.scanall", "scan all base syringes", handleSfcScanAllBaseSyringes},
    {"sfc.run", "run current recipe", handleSfcRun},
    {"sfc.load", "load recipe by recipe ID", handleSfcLoad},
    {"sfc.save", "save recipe by recipe ID", handleSfcSave},
    {"sfc.status", "sfc status", handleSfcStatus},
    {"sfc.scanbase", "scan a base slot", handleSfcScanBase},
    {"scantool", "scan toolhead syringe", handleSfcScanTool},
    {"transfer", "transfer <slot> <ml> from base to toolhead", handleTransfer},
    {"cal.tool.point", "add toolhead syringe calibration point <ml>", handleSfcCalTPoint},
    {"cal.tool.clear", "clear toolhead syringe calibration points", handleSfcCalToolClear},
    {"cal.tool.force0", "force toolhead syringe calibration to 0 mL", handleSfcCalToolForceZero},
    {"sfc.tool.show", "print toolhead info", handleSfcToolShow},
    {"showvolumes", "show volumes for scanned syringes", handleShowVolumes},
    {"sfc.recipe.save", "save recipe by recipe ID", handleSfcRecipeSave},
    {"sfc.recipe.load", "load recipe by recipe ID", handleSfcRecipeLoad},
    {"sfc.base.show", "show current base", handleSfcBaseShow},
    {"cal.base.point", "add base calibration point <ml> [slot]", handleSfcCalBasePoint},
    {"cal.base.clear", "clear current base calibration points", handleSfcCalBaseClear},
    {"cal.base.force0", "force base calibration to 0 mL", handleSfcCalBaseForceZero},
    {"potraw", "read pot", handlePotRaw},
    {"basepot", "read base pot", handleBasePot},
    {"pots", "read all pots", handlePotReport},
    {"potmove", "pot driven move", handlePotMove},
    {"i2cscan", "scan both I2C buses", handleI2cScan},
    {"sfc.recipe.list", "list recipe IDs in storage", handleSfcRecipeList},
    {"sfc.recipe.list.desc", "list recipe IDs in descending order", handleSfcRecipeListDesc},
    {"sfc.recipe.show", "show recipe JSON for a recipe ID", handleSfcRecipeShow},
    {"sfc.recipe.delete", "delete recipe for a recipe ID", handleSfcRecipeDelete},
};

const size_t COMMAND_COUNT = sizeof(COMMANDS) / sizeof(COMMANDS[0]);

// Look up a command descriptor by verb.
const CommandDescriptor *lookupCommand(const String &verb) {
  for (size_t i = 0; i < COMMAND_COUNT; ++i) {
    if (verb == COMMANDS[i].name) return &COMMANDS[i];
  }
  return nullptr;
}

}  // namespace

// Read serial input and dispatch the matching command handler.
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
