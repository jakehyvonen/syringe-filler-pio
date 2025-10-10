// ===== Pins (adjust if needed) =====
const uint8_t stepPin    = 22;
const uint8_t dirPin     = 23;
const uint8_t enablePin  = 24;
const uint8_t buttonPin  = 7;   // to GND, uses INPUT_PULLUP

// ===== Driver levels (A4988/DRV8825 typical) =====
const uint8_t ENABLE_LEVEL  = LOW;   // active enable
const uint8_t DISABLE_LEVEL = HIGH;  // disable

// ===== Motion config =====
volatile long   speed_sps      = 800;            // steps per second (default)
volatile uint32_t stepInterval = 1000000UL / 800; // µs per full step
const uint8_t  STEP_PULSE_US   = 4;              // STEP high time (≥2–5us typical)

// ===== Button debounce =====
const uint16_t DEBOUNCE_MS     = 15;
bool           btnStableState  = HIGH;  // with INPUT_PULLUP, HIGH = not pressed
bool           btnLastRead     = HIGH;
uint32_t       btnLastChangeMs = 0;

// ===== Step timer =====
uint32_t lastStepUs = 0;

// ===== Helpers =====
void applySpeed(long sps) {
  if (sps < 1)     sps = 1;
  if (sps > 20000) sps = 20000;     // keep ISR-less timing sane
  speed_sps  = sps;
  stepInterval = 1000000UL / (uint32_t)sps; // full-step interval
}

void motorEnable(bool en) {
  digitalWrite(enablePin, en ? ENABLE_LEVEL : DISABLE_LEVEL);
}

bool buttonPressed() {
  // simple debounce
  bool raw = digitalRead(buttonPin);
  uint32_t now = millis();
  if (raw != btnLastRead) {
    btnLastRead = raw;
    btnLastChangeMs = now;
  }
  if ((now - btnLastChangeMs) >= DEBOUNCE_MS) {
    btnStableState = raw;
  }
  return (btnStableState == LOW); // pressed when pulled low
}

void stepTickIfDue() {
  uint32_t now = micros();
  if ((uint32_t)(now - lastStepUs) >= stepInterval) {
    lastStepUs = now;
    // One full step: short HIGH pulse, then LOW
    digitalWrite(stepPin, HIGH);
    delayMicroseconds(STEP_PULSE_US);
    digitalWrite(stepPin, LOW);
  }
}

// ===== Serial control =====
// Commands (newline-terminated):
//   speed <sps>   e.g. "speed 1200"
//   dir <0|1>     0=LOW, 1=HIGH
//   on            (forces enable; still requires button press to run pulses)
//   off           (forces disable)
//   state         (prints settings)
void handleSerial() {
  static String line;
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\r') continue;
    if (c != '\n') { line += c; continue; }

    line.trim();
    if (line.length() == 0) { line = ""; continue; }

    if (line.startsWith("speed ")) {
      long s = line.substring(6).toInt();
      applySpeed(s);
      Serial.print("OK speed "); Serial.println(speed_sps);
    } else if (line.startsWith("dir ")) {
      int d = line.substring(4).toInt();
      digitalWrite(dirPin, d ? HIGH : LOW);
      Serial.print("OK dir "); Serial.println(d ? 1 : 0);
    } else if (line == "on") {
      motorEnable(true);
      Serial.println("OK on");
    } else if (line == "off") {
      motorEnable(false);
      Serial.println("OK off");
    } else if (line == "state") {
      Serial.print("speed_sps="); Serial.print(speed_sps);
      Serial.print("  dir=");      Serial.print(digitalRead(dirPin));
      Serial.print("  enabled=");  Serial.println(digitalRead(enablePin) == ENABLE_LEVEL ? 1 : 0);
    } else {
      Serial.println("ERR unknown (use: speed <sps> | dir <0|1> | on | off | state)");
    }
    line = "";
  }
}

// ===== Arduino setup/loop =====
void setup() {
  pinMode(stepPin,   OUTPUT);
  pinMode(dirPin,    OUTPUT);
  pinMode(enablePin, OUTPUT);
  pinMode(buttonPin, INPUT_PULLUP);

  digitalWrite(stepPin, LOW);
  digitalWrite(dirPin, HIGH);          // default direction
  motorEnable(false);                  // start disabled
  applySpeed(speed_sps);               // compute interval

  Serial.begin(115200);
  Serial.println("Ready. Commands: speed <sps>, dir <0|1>, on, off, state");
  Serial.println("Hold the button to run.");
}

void loop() {
  handleSerial();

  // Press & hold to run at constant speed
  if (buttonPressed()) {
    motorEnable(true);
    stepTickIfDue();
  } else {
    motorEnable(false);
  }
}
