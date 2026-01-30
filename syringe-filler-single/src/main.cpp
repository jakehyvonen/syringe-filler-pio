/*
#include <Arduino.h>

static String line;
static uint32_t lastBeat = 0;

static void printBanner() {
  Serial.println();
  Serial.println("=== ESP32-C3 Serial Echo Test ===");
  Serial.println("Type something and press Enter.");
  Serial.println("Commands: ping | help | baud?");
  Serial.println();
}

void setup() {
  Serial.begin(115200);

  // Give the host time to open the port, then spam banner briefly
  for (int i = 0; i < 10; i++) {
    printBanner();
    delay(200);
  }

  line.reserve(128);
}

void loop() {
  // Heartbeat so you always know it's alive
  if (millis() - lastBeat > 1000) {
    lastBeat = millis();
    Serial.println("tick");
  }

  // Read bytes and build a line
  while (Serial.available() > 0) {
    char c = (char)Serial.read();

    // Optional: show every character immediately (true echo)
    Serial.write(c);

    // Line handling
    if (c == '\r') continue;          // ignore CR
    if (c == '\n') {
      String cmd = line;
      line = "";

      cmd.trim();
      if (cmd.length() == 0) {
        Serial.println();
        continue;
      }

      Serial.print("\r\nYou said: ");
      Serial.println(cmd);

      if (cmd.equalsIgnoreCase("ping")) {
        Serial.println("pong");
      } else if (cmd.equalsIgnoreCase("help")) {
        Serial.println("Commands: ping | help | baud?");
      } else if (cmd.equalsIgnoreCase("baud?")) {
        Serial.println("115200 (fixed in sketch)");
      } else {
        Serial.println("(no command matched)");
      }

      Serial.println();
    } else {
      // Protect against runaway input
      if (line.length() < 200) line += c;
      else {
        line = "";
        Serial.println("\r\n(line too long; cleared)");
      }
    }
  }
}
*/
/**
 * @file main.cpp
 * @brief Single-syringe firmware: stepper + buttons + PN532 + web UI.
 */
 #include <Arduino.h>

#include <WifiCredentials.hpp>
#include <WifiManager.hpp>

#include "Pins.hpp"
#include "RfidReader.hpp"
#include "Storage.hpp"
#include "WebUI.hpp"

namespace {
constexpr uint32_t kDefaultSpeedSps = 800;
constexpr uint32_t kMinSpeedSps = 1;
constexpr uint32_t kMaxSpeedSps = 20000;
constexpr uint16_t kStepPulseWidthUs = 10;
constexpr uint16_t kDebounceMs = 15;
constexpr bool kWithdrawDirHigh = true;

const uint8_t kEnableLevel = LOW;
const uint8_t kDisableLevel = HIGH;

Shared::WifiManager g_wifi;
RfidReader g_rfid;

struct DebouncedButton {
  bool stable_state = HIGH;
  bool last_read = HIGH;
  uint32_t last_change_ms = 0;

  bool update(bool raw, uint32_t now_ms) {
    if (raw != last_read) {
      last_read = raw;
      last_change_ms = now_ms;
    }
    if ((now_ms - last_change_ms) >= kDebounceMs) {
      stable_state = raw;
    }
    return stable_state == LOW;
  }
};

class StepperControl {
 public:
  void begin() {
    pinMode(Pins::STEPPER_STEP, OUTPUT);
    pinMode(Pins::STEPPER_DIR, OUTPUT);
    digitalWrite(Pins::STEPPER_STEP, LOW);
    digitalWrite(Pins::STEPPER_DIR, LOW);
    if (Pins::STEPPER_ENABLE >= 0) {
      pinMode(Pins::STEPPER_ENABLE, OUTPUT);
      digitalWrite(Pins::STEPPER_ENABLE, kDisableLevel);
    }
    applySpeed(kDefaultSpeedSps);
  }

  void setDirection(bool withdraw) { setDirectionLevel(withdraw ? HIGH : LOW); }

  void setMoving(bool moving) { m_moving = moving; }

  void applySpeed(uint32_t sps) {
    if (sps < kMinSpeedSps) sps = kMinSpeedSps;
    if (sps > kMaxSpeedSps) sps = kMaxSpeedSps;
    m_speed_sps = sps;
    m_step_interval_us = 1000000UL / sps;
  }

  void motorEnable(bool enable) {
    if (Pins::STEPPER_ENABLE >= 0) {
      digitalWrite(Pins::STEPPER_ENABLE, enable ? kEnableLevel : kDisableLevel);
    }
    m_driver_enabled = enable;
  }

  void setForceEnable(bool force) { m_force_enable = force; }

  void update() {
    if (!m_moving) {
      if (!m_force_enable) {
        motorEnable(false);
      }
      return;
    }

    motorEnable(true);
    uint32_t now = micros();
    if ((uint32_t)(now - m_lastStepUs) < m_step_interval_us) return;
    m_lastStepUs = now;
    digitalWrite(Pins::STEPPER_STEP, HIGH);
    delayMicroseconds(kStepPulseWidthUs);
    digitalWrite(Pins::STEPPER_STEP, LOW);
  }

  uint32_t speedSps() const { return m_speed_sps; }
  uint32_t stepIntervalUs() const { return m_step_interval_us; }
  bool driverEnabled() const { return m_driver_enabled; }
  bool forceEnable() const { return m_force_enable; }
  int directionLevel() const { return m_dir_level; }

 private:
  void setDirectionLevel(int level) {
    digitalWrite(Pins::STEPPER_DIR, level);
    m_dir_level = level;
  }

  bool m_moving = false;
  uint32_t m_lastStepUs = 0;
  uint32_t m_speed_sps = kDefaultSpeedSps;
  uint32_t m_step_interval_us = 1000000UL / kDefaultSpeedSps;
  bool m_driver_enabled = false;
  bool m_force_enable = false;
  int m_dir_level = LOW;
};

StepperControl g_stepper;

String g_input;

DebouncedButton g_withdraw_button;
DebouncedButton g_dispense_button;

bool g_last_withdraw_pressed = false;
bool g_last_dispense_pressed = false;
bool g_last_moving = false;
uint32_t g_last_status_ms = 0;
uint32_t g_last_button_report_ms = 0;

void printStepperState(const char* context) {
  Serial.print("[Stepper] ");
  Serial.print(context);
  Serial.print(" speed_sps=");
  Serial.print(g_stepper.speedSps());
  Serial.print(" interval_us=");
  Serial.print(g_stepper.stepIntervalUs());
  Serial.print(" dir=");
  Serial.print(g_stepper.directionLevel());
  Serial.print(" enabled=");
  Serial.print(g_stepper.driverEnabled() ? "1" : "0");
  Serial.print(" force=");
  Serial.print(g_stepper.forceEnable() ? "1" : "0");
  Serial.print(" enable_pin=");
  Serial.println(Pins::STEPPER_ENABLE);
}

void printStructured(const char* cmd, bool ok, const String& message = "", const String& data = "") {
  Serial.print("{\"cmd\":\"");
  Serial.print(cmd);
  Serial.print("\",\"status\":\"");
  Serial.print(ok ? "ok" : "error");
  Serial.print("\"");
  if (message.length()) {
    Serial.print(",\"message\":\"");
    Serial.print(message);
    Serial.print("\"");
  }
  if (data.length()) {
    Serial.print(",\"data\":");
    Serial.print(data);
  }
  Serial.println("}");
}

void handleWifiStatus() {
  printStructured("wifi.status", true, "", g_wifi.buildStatusJson());
}

void handleWifiSet(const String& args) {
  int sp = args.indexOf(' ');
  String ssid = (sp < 0) ? args : args.substring(0, sp);
  String password = (sp < 0) ? "" : args.substring(sp + 1);
  ssid.trim();
  password.trim();
  if (ssid.length() == 0) {
    printStructured("wifi.set", false, "usage: wifi.set <ssid> [password]");
    return;
  }

  if (!Shared::WiFiCredentials::save(ssid, password)) {
    printStructured("wifi.set", false, "failed to save credentials");
    return;
  }

  bool connected = g_wifi.connect(ssid, password);
  printStructured("wifi.set", connected, connected ? "connected" : "connect failed", g_wifi.buildStatusJson());
}

void handleWifiConnect() {
  String ssid;
  String password;
  if (!Shared::WiFiCredentials::load(ssid, password)) {
    printStructured("wifi.connect", false, "no saved credentials");
    return;
  }
  bool connected = g_wifi.connect(ssid, password);
  printStructured("wifi.connect", connected, connected ? "connected" : "connect failed", g_wifi.buildStatusJson());
}

void handleWifiClear() {
  if (!Shared::WiFiCredentials::clear()) {
    printStructured("wifi.clear", false, "failed to clear credentials");
    return;
  }
  printStructured("wifi.clear", true, "credentials cleared");
}

void handleWifiAp() {
  g_wifi.startAccessPoint();
  printStructured("wifi.ap", true, "ap started");
}

void handleWifiScan() {
  printStructured("wifi.scan", true, "", g_wifi.buildScanJson());
}

bool handleStepperCommand(const String& line) {
  if (line.startsWith("speed ")) {
    uint32_t sps = static_cast<uint32_t>(line.substring(6).toInt());
    g_stepper.applySpeed(sps);
    Serial.print("OK speed ");
    Serial.println(g_stepper.speedSps());
    printStepperState("speed");
    return true;
  }
  if (line.startsWith("dir ")) {
    int dir_level = line.substring(4).toInt() ? HIGH : LOW;
    g_stepper.setDirection(dir_level == HIGH);
    Serial.print("OK dir ");
    Serial.println(dir_level == HIGH ? 1 : 0);
    printStepperState("dir");
    return true;
  }
  if (line == "on") {
    g_stepper.setForceEnable(true);
    g_stepper.motorEnable(true);
    Serial.println("OK on");
    printStepperState("on");
    return true;
  }
  if (line == "off") {
    g_stepper.setForceEnable(false);
    g_stepper.motorEnable(false);
    Serial.println("OK off");
    printStepperState("off");
    return true;
  }
  if (line == "state") {
    printStepperState("state");
    return true;
  }
  return false;
}

void handleCommand(const String& line) {
  if (handleStepperCommand(line)) return;

  int sp = line.indexOf(' ');
  String cmd = (sp < 0) ? line : line.substring(0, sp);
  String args = (sp < 0) ? "" : line.substring(sp + 1);
  cmd.trim();
  args.trim();

  if (cmd == "wifi.status") {
    handleWifiStatus();
  } else if (cmd == "wifi.set") {
    handleWifiSet(args);
  } else if (cmd == "wifi.connect") {
    handleWifiConnect();
  } else if (cmd == "wifi.clear") {
    handleWifiClear();
  } else if (cmd == "wifi.ap") {
    handleWifiAp();
  } else if (cmd == "wifi.scan") {
    handleWifiScan();
  } else if (cmd.length()) {
    printStructured(cmd.c_str(), false, "unknown command");
  }
}

void readSerialCommands() {
  while (Serial.available()) {
    char c = static_cast<char>(Serial.read());
    if (c == '\r') continue;
    if (c == '\n') {
      String line = g_input;
      g_input = "";
      line.trim();
      if (line.length()) handleCommand(line);
      continue;
    }
    g_input += c;
  }
}

void startWiFi() {
  String ssid;
  String password;
  if (Shared::WiFiCredentials::load(ssid, password)) {
    if (g_wifi.connect(ssid, password)) {
      return;
    }
    Serial.println("[WiFi] Falling back to AP mode.");
  } else {
    Serial.println("[WiFi] No saved WiFi credentials found.");
  }
  g_wifi.startAccessPoint();
  Serial.println("[WiFi] Open http://192.168.4.1/ to configure.");
}

}  // namespace

void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.println("\n[Single] Booting...");
  Serial.println("[Single] Stepper debug enabled.");
  Serial.println("[Single] Stepper commands: speed <sps>, dir <0|1>, on, off, state");

  pinMode(Pins::BUTTON_WITHDRAW, INPUT_PULLUP);
  pinMode(Pins::BUTTON_DISPENSE, INPUT_PULLUP);
  g_stepper.begin();
  printStepperState("init");
  if (Pins::STEPPER_ENABLE < 0) {
    Serial.println("[Stepper] Warning: STEPPER_ENABLE is -1 (driver always enabled or wired differently).");
  }

  if (!Storage::init()) {
    Serial.println("[Storage] Failed to init LittleFS.");
  }

  g_rfid.begin();

  startWiFi();
  WebUI::begin();
}

void loop() {
  readSerialCommands();

  static uint32_t lastRfidPoll = 0;
  uint32_t nowMs = millis();
  if (nowMs - lastRfidPoll >= 200) {
    lastRfidPoll = nowMs;
    g_rfid.poll();
    WebUI::setCurrentRfid(g_rfid.currentTag());
  }

  bool withdrawPressed = g_withdraw_button.update(digitalRead(Pins::BUTTON_WITHDRAW), nowMs);
  bool dispensePressed = g_dispense_button.update(digitalRead(Pins::BUTTON_DISPENSE), nowMs);

  if (withdrawPressed != g_last_withdraw_pressed || dispensePressed != g_last_dispense_pressed) {
    Serial.print("[Buttons] withdraw=");
    Serial.print(withdrawPressed ? "pressed" : "released");
    Serial.print(" dispense=");
    Serial.println(dispensePressed ? "pressed" : "released");
    g_last_withdraw_pressed = withdrawPressed;
    g_last_dispense_pressed = dispensePressed;
  }

  if (nowMs - g_last_button_report_ms >= 2000) {
    g_last_button_report_ms = nowMs;
    Serial.print("[Buttons] poll withdraw=");
    Serial.print(withdrawPressed ? "1" : "0");
    Serial.print(" dispense=");
    Serial.println(dispensePressed ? "1" : "0");
  }

  if (withdrawPressed && !dispensePressed) {
    g_stepper.setDirection(kWithdrawDirHigh);
    g_stepper.setMoving(true);
  } else if (dispensePressed && !withdrawPressed) {
    g_stepper.setDirection(!kWithdrawDirHigh);
    g_stepper.setMoving(true);
  } else {
    g_stepper.setMoving(false);
  }

  bool moving = withdrawPressed != dispensePressed;
  if (moving != g_last_moving) {
    g_last_moving = moving;
    Serial.println(moving ? "[Stepper] motion start" : "[Stepper] motion stop");
    printStepperState("motion");
  }

  g_stepper.update();

  if (nowMs - g_last_status_ms >= 5000) {
    g_last_status_ms = nowMs;
    printStepperState("heartbeat");
  }
  //WebUI::handle();
}
