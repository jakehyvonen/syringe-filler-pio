/**
 * @file main.cpp
 * @brief Single-syringe firmware: dual steppers + buttons + PN532 + web UI.
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
  StepperControl(int step_pin, int dir_pin, int enable_pin)
      : m_step_pin(step_pin), m_dir_pin(dir_pin), m_enable_pin(enable_pin) {}

  bool enablePinConfigured() const { return m_enable_pin_configured; }

  void begin() {
    pinMode(m_step_pin, OUTPUT);
    pinMode(m_dir_pin, OUTPUT);
    digitalWrite(m_step_pin, LOW);
    digitalWrite(m_dir_pin, LOW);
    if (m_enable_pin >= 0 && digitalPinCanOutput(m_enable_pin)) {
      pinMode(m_enable_pin, OUTPUT);
      digitalWrite(m_enable_pin, kDisableLevel);
      m_enable_pin_configured = true;
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
    if (m_enable_pin_configured) {
      digitalWrite(m_enable_pin, enable ? kEnableLevel : kDisableLevel);
    }
    m_driver_enabled = enable;
  }

  void setButtonEnable(bool enable) { m_button_enable = enable; }

  void update() {
    bool should_enable = m_button_enable;
    if (!m_moving) {
      motorEnable(should_enable);
      return;
    }

    motorEnable(true);
    uint32_t now = micros();
    if ((uint32_t)(now - m_lastStepUs) < m_step_interval_us) return;
    m_lastStepUs = now;
    digitalWrite(m_step_pin, HIGH);
    delayMicroseconds(kStepPulseWidthUs);
    digitalWrite(m_step_pin, LOW);
  }

  uint32_t speedSps() const { return m_speed_sps; }
  uint32_t stepIntervalUs() const { return m_step_interval_us; }
  bool driverEnabled() const { return m_driver_enabled; }
  bool buttonEnable() const { return m_button_enable; }
  int directionLevel() const { return m_dir_level; }
  int enablePin() const { return m_enable_pin; }

 private:
  void setDirectionLevel(int level) {
    digitalWrite(m_dir_pin, level);
    m_dir_level = level;
  }

  int m_step_pin;
  int m_dir_pin;
  int m_enable_pin;
  bool m_moving = false;
  uint32_t m_lastStepUs = 0;
  uint32_t m_speed_sps = kDefaultSpeedSps;
  uint32_t m_step_interval_us = 1000000UL / kDefaultSpeedSps;
  bool m_driver_enabled = false;
  bool m_button_enable = false;
  bool m_enable_pin_configured = false;
  int m_dir_level = LOW;
};

StepperControl g_stepper1(Pins::STEPPER1_STEP, Pins::STEPPER1_DIR, Pins::STEPPER1_ENABLE);
StepperControl g_stepper2(Pins::STEPPER2_STEP, Pins::STEPPER2_DIR, Pins::STEPPER2_ENABLE);

String g_input;

DebouncedButton g_withdraw_button1;
DebouncedButton g_dispense_button1;
DebouncedButton g_withdraw_button2;
DebouncedButton g_dispense_button2;

bool g_last_withdraw_pressed1 = false;
bool g_last_dispense_pressed1 = false;
bool g_last_withdraw_pressed2 = false;
bool g_last_dispense_pressed2 = false;
bool g_last_moving1 = false;
bool g_last_moving2 = false;
uint32_t g_last_status_ms = 0;
uint32_t g_last_button_report_ms = 0;

void printStepperState(const char* context, const char* name, const StepperControl& stepper) {
  Serial.print("[Stepper-");
  Serial.print(name);
  Serial.print("] ");
  Serial.print(context);
  Serial.print(" speed_sps=");
  Serial.print(stepper.speedSps());
  Serial.print(" interval_us=");
  Serial.print(stepper.stepIntervalUs());
  Serial.print(" dir=");
  Serial.print(stepper.directionLevel());
  Serial.print(" enabled=");
  Serial.print(stepper.driverEnabled() ? "1" : "0");
  Serial.print(" btn_en=");
  Serial.print(stepper.buttonEnable() ? "1" : "0");
  Serial.print(" enable_pin=");
  Serial.println(stepper.enablePin());
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

void handleWifiStatus() { printStructured("wifi.status", true, "", g_wifi.buildStatusJson()); }

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

void handleWifiScan() { printStructured("wifi.scan", true, "", g_wifi.buildScanJson()); }

bool handleStepperCommand(const String& line) {
  if (line.startsWith("speed ")) {
    uint32_t sps = static_cast<uint32_t>(line.substring(6).toInt());
    g_stepper1.applySpeed(sps);
    g_stepper2.applySpeed(sps);
    Serial.print("OK speed ");
    Serial.println(g_stepper1.speedSps());
    printStepperState("speed", "1", g_stepper1);
    printStepperState("speed", "2", g_stepper2);
    return true;
  }
  if (line.startsWith("dir ")) {
    int dir_level = line.substring(4).toInt() ? HIGH : LOW;
    g_stepper1.setDirection(dir_level == HIGH);
    Serial.print("OK dir ");
    Serial.println(dir_level == HIGH ? 1 : 0);
    printStepperState("dir", "1", g_stepper1);
    return true;
  }
  if (line.startsWith("dir2 ")) {
    int dir_level = line.substring(5).toInt() ? HIGH : LOW;
    g_stepper2.setDirection(dir_level == HIGH);
    Serial.print("OK dir2 ");
    Serial.println(dir_level == HIGH ? 1 : 0);
    printStepperState("dir2", "2", g_stepper2);
    return true;
  }
  if (line == "on") {
    Serial.println("OK on (ignored; enable follows buttons only)");
    printStepperState("on", "1", g_stepper1);
    printStepperState("on", "2", g_stepper2);
    return true;
  }
  if (line == "off") {
    g_stepper1.motorEnable(false);
    g_stepper2.motorEnable(false);
    Serial.println("OK off");
    printStepperState("off", "1", g_stepper1);
    printStepperState("off", "2", g_stepper2);
    return true;
  }
  if (line == "state") {
    printStepperState("state", "1", g_stepper1);
    printStepperState("state", "2", g_stepper2);
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
  Serial.println("[Single] Stepper commands: speed <sps>, dir <0|1>, dir2 <0|1>, on, off, state");

  pinMode(Pins::BUTTON1_WITHDRAW, INPUT_PULLUP);
  pinMode(Pins::BUTTON1_DISPENSE, INPUT_PULLUP);
  pinMode(Pins::BUTTON2_WITHDRAW, INPUT_PULLUP);
  pinMode(Pins::BUTTON2_DISPENSE, INPUT_PULLUP);

  g_stepper1.begin();
  g_stepper2.begin();

  printStepperState("init", "1", g_stepper1);
  printStepperState("init", "2", g_stepper2);

  if (Pins::STEPPER1_ENABLE < 0 || Pins::STEPPER2_ENABLE < 0) {
    Serial.println("[Stepper] Warning: at least one STEPPER_ENABLE is -1 (driver always enabled or wired differently).");
  }
  if (!g_stepper1.enablePinConfigured()) {
    Serial.print("[Stepper-1] Warning: STEPPER_ENABLE pin ");
    Serial.print(Pins::STEPPER1_ENABLE);
    Serial.println(" does not support output on esp32dev; EN will not be driven.");
  }
  if (!g_stepper2.enablePinConfigured()) {
    Serial.print("[Stepper-2] Warning: STEPPER_ENABLE pin ");
    Serial.print(Pins::STEPPER2_ENABLE);
    Serial.println(" does not support output on esp32dev; EN will not be driven.");
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

  bool withdrawPressed1 = g_withdraw_button1.update(digitalRead(Pins::BUTTON1_WITHDRAW), nowMs);
  bool dispensePressed1 = g_dispense_button1.update(digitalRead(Pins::BUTTON1_DISPENSE), nowMs);
  bool withdrawPressed2 = g_withdraw_button2.update(digitalRead(Pins::BUTTON2_WITHDRAW), nowMs);
  bool dispensePressed2 = g_dispense_button2.update(digitalRead(Pins::BUTTON2_DISPENSE), nowMs);

  if (withdrawPressed1 != g_last_withdraw_pressed1 || dispensePressed1 != g_last_dispense_pressed1) {
    Serial.print("[Buttons-1] withdraw=");
    Serial.print(withdrawPressed1 ? "pressed" : "released");
    Serial.print(" dispense=");
    Serial.println(dispensePressed1 ? "pressed" : "released");
    g_last_withdraw_pressed1 = withdrawPressed1;
    g_last_dispense_pressed1 = dispensePressed1;
  }

  if (withdrawPressed2 != g_last_withdraw_pressed2 || dispensePressed2 != g_last_dispense_pressed2) {
    Serial.print("[Buttons-2] withdraw=");
    Serial.print(withdrawPressed2 ? "pressed" : "released");
    Serial.print(" dispense=");
    Serial.println(dispensePressed2 ? "pressed" : "released");
    g_last_withdraw_pressed2 = withdrawPressed2;
    g_last_dispense_pressed2 = dispensePressed2;
  }

  if (nowMs - g_last_button_report_ms >= 2000) {
    g_last_button_report_ms = nowMs;
    Serial.print("[Buttons-1] poll withdraw=");
    Serial.print(withdrawPressed1 ? "1" : "0");
    Serial.print(" dispense=");
    Serial.println(dispensePressed1 ? "1" : "0");

    Serial.print("[Buttons-2] poll withdraw=");
    Serial.print(withdrawPressed2 ? "1" : "0");
    Serial.print(" dispense=");
    Serial.println(dispensePressed2 ? "1" : "0");
  }

  bool buttonPressed1 = withdrawPressed1 || dispensePressed1;
  g_stepper1.setButtonEnable(buttonPressed1);

  if (withdrawPressed1 && !dispensePressed1) {
    g_stepper1.setDirection(kWithdrawDirHigh);
    g_stepper1.setMoving(true);
  } else if (dispensePressed1 && !withdrawPressed1) {
    g_stepper1.setDirection(!kWithdrawDirHigh);
    g_stepper1.setMoving(true);
  } else {
    g_stepper1.setMoving(false);
    g_stepper1.motorEnable(false);
  }

  bool buttonPressed2 = withdrawPressed2 || dispensePressed2;
  g_stepper2.setButtonEnable(buttonPressed2);

  if (withdrawPressed2 && !dispensePressed2) {
    g_stepper2.setDirection(kWithdrawDirHigh);
    g_stepper2.setMoving(true);
  } else if (dispensePressed2 && !withdrawPressed2) {
    g_stepper2.setDirection(!kWithdrawDirHigh);
    g_stepper2.setMoving(true);
  } else {
    g_stepper2.setMoving(false);
    g_stepper2.motorEnable(false);
  }

  bool moving1 = withdrawPressed1 != dispensePressed1;
  if (moving1 != g_last_moving1) {
    g_last_moving1 = moving1;
    Serial.println(moving1 ? "[Stepper-1] motion start" : "[Stepper-1] motion stop");
    printStepperState("motion", "1", g_stepper1);
  }

  bool moving2 = withdrawPressed2 != dispensePressed2;
  if (moving2 != g_last_moving2) {
    g_last_moving2 = moving2;
    Serial.println(moving2 ? "[Stepper-2] motion start" : "[Stepper-2] motion stop");
    printStepperState("motion", "2", g_stepper2);
  }

  g_stepper1.update();
  g_stepper2.update();

  if (nowMs - g_last_status_ms >= 5000) {
    g_last_status_ms = nowMs;
    printStepperState("heartbeat", "1", g_stepper1);
    printStepperState("heartbeat", "2", g_stepper2);
  }
  //WebUI::handle();
}
