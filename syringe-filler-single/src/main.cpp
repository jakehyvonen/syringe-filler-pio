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
constexpr uint32_t kStepIntervalUs = 800;  // ~1250 steps/sec
constexpr uint16_t kStepPulseWidthUs = 3;
constexpr bool kWithdrawDirHigh = true;

Shared::WifiManager g_wifi;
RfidReader g_rfid;

class StepperControl {
 public:
  void begin() {
    pinMode(Pins::STEPPER_STEP, OUTPUT);
    pinMode(Pins::STEPPER_DIR, OUTPUT);
    digitalWrite(Pins::STEPPER_STEP, LOW);
    digitalWrite(Pins::STEPPER_DIR, LOW);
  }

  void setDirection(bool withdraw) {
    digitalWrite(Pins::STEPPER_DIR, withdraw ? HIGH : LOW);
  }

  void setMoving(bool moving) { m_moving = moving; }

  void update() {
    if (!m_moving) return;
    uint32_t now = micros();
    if (now - m_lastStepUs < kStepIntervalUs) return;
    m_lastStepUs = now;
    digitalWrite(Pins::STEPPER_STEP, HIGH);
    delayMicroseconds(kStepPulseWidthUs);
    digitalWrite(Pins::STEPPER_STEP, LOW);
  }

 private:
  bool m_moving = false;
  uint32_t m_lastStepUs = 0;
};

StepperControl g_stepper;

String g_input;

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

void handleCommand(const String& line) {
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

  pinMode(Pins::BUTTON_WITHDRAW, INPUT_PULLUP);
  pinMode(Pins::BUTTON_DISPENSE, INPUT_PULLUP);
  g_stepper.begin();

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

  bool withdrawPressed = digitalRead(Pins::BUTTON_WITHDRAW) == LOW;
  bool dispensePressed = digitalRead(Pins::BUTTON_DISPENSE) == LOW;

  if (withdrawPressed && !dispensePressed) {
    g_stepper.setDirection(kWithdrawDirHigh);
    g_stepper.setMoving(true);
  } else if (dispensePressed && !withdrawPressed) {
    g_stepper.setDirection(!kWithdrawDirHigh);
    g_stepper.setMoving(true);
  } else {
    g_stepper.setMoving(false);
  }

  g_stepper.update();
  WebUI::handle();
}
