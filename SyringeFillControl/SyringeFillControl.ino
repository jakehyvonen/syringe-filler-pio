#define dirPin 2
#define stepPin 3
#define enablePin 8

#define STEP2_PIN   4
#define DIR2_PIN    5
#define EN2_PIN     6

#define STEP3_PIN   7
#define DIR3_PIN    10
#define EN3_PIN     11

#include <EEPROM.h>   // for optional persistence

#define NUM_BASES 5   // number of base syringes

// Default/calibrated positions (in steps). Edit these once;
// or set them at runtime and save to EEPROM.
long basePos[NUM_BASES] = {
  3330, // 1
  5480, // 2
  3330, // 3
  3330, // 4
  3330  // 5
};

// Where in EEPROM we store them (each long = 4 bytes)
const int EEPROM_BASE_ADDR = 0; // uses 4*NUM_BASES bytes starting here


// ---- Limit switch wiring (RAMPS 1.4 endstop) ----
#define limitPin 9     // green 'S' wire from the endstop
#define raisedPin 12   //  "toolhead raised" switch

#define HOME_DIR LOW          // set the direction that *moves toward* the switch (LOW or HIGH)
#define DISABLE_LEVEL HIGH    // A4988/DRV8825: ENABLE pin is active-LOW. Use HIGH to disable.
#define ENABLE_LEVEL  LOW     // Use LOW to enable the driver.

// Steps/mm helps print pretty units and enforce soft limits (tune this!)
const float STEPS_PER_MM = 80.0f; // <- EXAMPLE for 1/16 microstep + GT2(20T) belt, change to your machine

// Soft limits (in steps). Comment out CHECK_SOFT_LIMITS to disable.
#define CHECK_SOFT_LIMITS
const long MIN_POS_STEPS = 0;             // home at 0
const long MAX_POS_STEPS = (long)(200.0f * STEPS_PER_MM);  // e.g. 200 mm travel

//stepper Position state
volatile long currentPositionSteps = 0;  // signed, increases with DIR=HIGH (see below)
volatile long currentPositionSteps2 = 0;
volatile long currentPositionSteps3 = 0;
// Pulse width for STEP high (us). 2–5us is typical for A4988/DRV8825.
#define STEP_PULSE_US 4

// Shared speed for simultaneous moves of 2 & 3 (steps/sec)
static long speed23_sps = 800;             // default
static unsigned long interval23_us = 1000000UL / (2UL * 800UL); // µs between toggles

bool homed = false;

long lastStepTime = 0;
unsigned long stepInterval = 1000;  // microseconds between steps (500 steps/sec)
bool motorEnabled = false;
bool stepState = false;



///// START SERVO SECTION ///////

#include <Adafruit_PWMServoDriver.h>

// --- PCA9685 servo driver ---
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

// Servo calibration (these are typical; adjust to your servos)
#define SERVO_MIN  150  // pulse length out of 4096 for ~0°
#define SERVO_MAX  600  // pulse length out of 4096 for ~180°
#define raisedPos 99    // toolhead vertical servo (3) position that is calibrated to activate the limit switch
#define couplingPos1 129 // position of toolhead servo before rotary servo is activated


// Track commanded angles so slow sweeps can start from last setpoint.
static int currentAngles[16];
static bool currentAnglesInit = false;

static inline void _ensureAnglesInit() {
  if (!currentAnglesInit) {
    for (int i = 0; i < 16; ++i) currentAngles[i] = 90; // default
    currentAnglesInit = true;
  }
}


bool isToolheadRaised() {
  return (digitalRead(raisedPin) == LOW);  // LOW means pressed
}

// Try to ensure the toolhead is physically raised.
// Returns true if raised (already or after trying), false if it failed within timeout.
bool ensureToolheadRaised(uint16_t timeout_ms = 1200) {
  if (isToolheadRaised()) return true;

  // Attempt to raise
  setServoAngle(3, raisedPos);  // quick command; you could use setServoAngleSlow if you prefer

  unsigned long start = millis();
  while (!isToolheadRaised() && (millis() - start) < timeout_ms) {
    // Nudge again just in case the first command hit a deadband
    // (won't hurt servos at 60Hz; this simply reasserts the target)
    setServoAngle(3, raisedPos);
    delay(100);
  }

  if (!isToolheadRaised()) {
    Serial.println("ERROR: Toolhead not raised (timeout). Movement blocked.");
    return false;
  }
  return true;
}


// Function to set servo pulse directly in microseconds
void setServoPulseRaw(uint8_t servoNum, int pulseLength) {
  // pulseLength is in microseconds, e.g. 1500 = neutral
  pwm.writeMicroseconds(servoNum, pulseLength);
}

void setServoAngle(uint8_t channel, int angle) {
  _ensureAnglesInit();
  if (angle < 0) angle = 0;
  if (angle > 180) angle = 180;

  uint16_t pulselen = map(angle, 0, 180, SERVO_MIN, SERVO_MAX);
  pwm.setPWM(channel, 0, pulselen);

  currentAngles[channel] = angle;  // <-- keep state
  // (optional) comment out prints if you want it quieter
  Serial.print("Servo ");
  Serial.print(channel);
  Serial.print(" -> ");
  Serial.print(angle);
  Serial.println(" deg");
}

void raiseToolhead(){
      setServoAngle(3, raisedPos);
}

// Slowly sweep servo from its current position to target
void setServoAngleSlow(uint8_t channel, int targetAngle, int stepDelay = 23) {
  _ensureAnglesInit();
  if (targetAngle < 0) targetAngle = 0;
  if (targetAngle > 180) targetAngle = 180;

  int startAngle = currentAngles[channel];
  if (startAngle == targetAngle) return;

  int step = (targetAngle > startAngle) ? 1 : -1;
  for (int angle = startAngle; angle != targetAngle; angle += step) {
    uint16_t pulselen = map(angle, 0, 180, SERVO_MIN, SERVO_MAX);
    pwm.setPWM(channel, 0, pulselen);
    delay(stepDelay);
  }
  uint16_t pulselen = map(targetAngle, 0, 180, SERVO_MIN, SERVO_MAX);
  pwm.setPWM(channel, 0, pulselen);
  currentAngles[channel] = targetAngle;

  Serial.print("Servo ");
  Serial.print(channel);
  Serial.print(" slowly moved to ");
  Serial.print(targetAngle);
  Serial.println(" deg");
}

// Move two servos simultaneously to their targets (blocking).
// Each servo has its own step delay (ms per step) and step size (deg per step).
// Example: setServoAnglesDual(0, 150, 1, 60, 10, 20);  // ch0 fast, ch1 slower
void setServoAnglesDual(uint8_t chA, int tgtA,
                        uint8_t chB, int tgtB,
                        uint16_t stepDelayA = 20, uint16_t stepDelayB = 20,
                        uint8_t stepSizeA = 1,  uint8_t stepSizeB = 1)
{
  _ensureAnglesInit();

  // clamp & sanitize
  if (tgtA < 0) tgtA = 0; if (tgtA > 180) tgtA = 180;
  if (tgtB < 0) tgtB = 0; if (tgtB > 180) tgtB = 180;
  if (stepDelayA == 0) stepDelayA = 1;
  if (stepDelayB == 0) stepDelayB = 1;
  if (stepSizeA == 0)  stepSizeA  = 1;
  if (stepSizeB == 0)  stepSizeB  = 1;

  int curA = currentAngles[chA];
  int curB = currentAngles[chB];

  // quick exit if nothing to do
  if (curA == tgtA && curB == tgtB) return;

  // direction (sign)
  int dirA = (tgtA > curA) ? +1 : -1;
  int dirB = (tgtB > curB) ? +1 : -1;
  if (tgtA == curA) dirA = 0;
  if (tgtB == curB) dirB = 0;

  unsigned long lastA = millis();
  unsigned long lastB = millis();

  auto writeA = [&](int a){
    uint16_t p = map(a, 0, 180, SERVO_MIN, SERVO_MAX);
    pwm.setPWM(chA, 0, p);
  };
  auto writeB = [&](int b){
    uint16_t p = map(b, 0, 180, SERVO_MIN, SERVO_MAX);
    pwm.setPWM(chB, 0, p);
  };

  // initial write to ensure consistent start
  writeA(curA);
  writeB(curB);

  while (true) {
    unsigned long now = millis();
    bool progressed = false;

    // A due?
    if (dirA != 0 && (now - lastA) >= stepDelayA) {
      lastA = now;
      // step toward target with bounded step size
      int nextA = curA + dirA * (int)stepSizeA;
      if ((dirA > 0 && nextA > tgtA) || (dirA < 0 && nextA < tgtA)) nextA = tgtA;
      if (nextA != curA) {
        curA = nextA;
        writeA(curA);
        progressed = true;
      }
      if (curA == tgtA) dirA = 0;
    }

    // B due?
    if (dirB != 0 && (now - lastB) >= stepDelayB) {
      lastB = now;
      int nextB = curB + dirB * (int)stepSizeB;
      if ((dirB > 0 && nextB > tgtB) || (dirB < 0 && nextB < tgtB)) nextB = tgtB;
      if (nextB != curB) {
        curB = nextB;
        writeB(curB);
        progressed = true;
      }
      if (curB == tgtB) dirB = 0;
    }

    if (dirA == 0 && dirB == 0) break;   // both done
    if (!progressed) delay(1);           // yield a bit
  }

  currentAngles[chA] = curA;
  currentAngles[chB] = curB;

  Serial.print("Servos ");
  Serial.print(chA); Serial.print("->"); Serial.print(tgtA);
  Serial.print(", ");
  Serial.print(chB); Serial.print("->"); Serial.print(tgtB);
  Serial.println(" (dual slow)");
}


//////// END SERVO SECTION ////////


// Block until a step pulse at the current stepInterval elapses, then toggle STEP.
// Uses an internal static edge state so you can call repeatedly in a loop.
static inline void stepOnceTimed() {
  static bool localStepState = false;
  static unsigned long last = micros();
  unsigned long now;
  do { now = micros(); } while ((unsigned long)(now - last) < stepInterval);
  last = now;

  localStepState = !localStepState;
  digitalWrite(stepPin, localStepState ? HIGH : LOW);
}
// One timed toggle on a given step pin (uses its own timer)
static inline void stepOnceTimedOnPin(uint8_t pin, unsigned long interval_us) {
  static unsigned long last2 = 0, last3 = 0;
  static bool state2 = false, state3 = false;

  unsigned long &last = (pin == STEP2_PIN) ? last2 : last3;
  bool &state         = (pin == STEP2_PIN) ? state2 : state3;

  unsigned long now;
  do { now = micros(); } while ((unsigned long)(now - last) < interval_us);
  last = now;

  state = !state;
  digitalWrite(pin, state ? HIGH : LOW);
}


// Move a signed number of steps at the current stepInterval (blocking).
// Positive 'steps' moves with DIR=HIGH, negative with DIR=LOW.
// Updates the global currentPositionSteps on EACH full step edge-pair.
static void moveSteps(long steps) {
  if (steps == 0) return;


  // SAFETY: make sure we're raised (try to raise; error out if we can't)
  if (!ensureToolheadRaised()) return;

  bool dirHigh = (steps > 0);
  long todo = labs(steps);

  digitalWrite(dirPin, dirHigh ? HIGH : LOW);

  // One full "step" for a STEP/DIR driver requires two edges (H/L),
  // but our stepOnceTimed() already toggles. So we count *toggles* pairs.
  // We'll update position on the rising edge only.
  for (long i = 0; i < todo; i++) {
    // Rising edge
    stepOnceTimed();
    // ---- Position sign convention ----
    // By default, DIR=HIGH increments ( +1 ), DIR=LOW decrements ( -1 ).
    // If this is opposite to your mechanical sense, flip the +/- here.
    currentPositionSteps += dirHigh ? +1 : -1;

    // Falling edge (complete the pulse)
    stepOnceTimed();
  }
}


// Blocking move of N steps on motor #2
static void moveSteps2(long steps) {
  if (steps == 0) return;
  bool dirHigh = (steps > 0);
  long todo = labs(steps);

  digitalWrite(EN2_PIN, ENABLE_LEVEL);
  digitalWrite(DIR2_PIN, dirHigh ? HIGH : LOW);

  for (long i = 0; i < todo; i++) {
    stepOnceTimedOnPin(STEP2_PIN, interval23_us); // reuse interval23_us for simplicity
    currentPositionSteps2 += dirHigh ? +1 : -1;
    stepOnceTimedOnPin(STEP2_PIN, interval23_us); // complete pulse
  }

  digitalWrite(EN2_PIN, DISABLE_LEVEL);
}

// Blocking move of N steps on motor #3
static void moveSteps3(long steps) {
  if (steps == 0) return;
  bool dirHigh = (steps > 0);
  long todo = labs(steps);

  digitalWrite(EN3_PIN, ENABLE_LEVEL);
  digitalWrite(DIR3_PIN, dirHigh ? HIGH : LOW);

  for (long i = 0; i < todo; i++) {
    stepOnceTimedOnPin(STEP3_PIN, interval23_us);
    currentPositionSteps3 += dirHigh ? +1 : -1;
    stepOnceTimedOnPin(STEP3_PIN, interval23_us);
  }

  digitalWrite(EN3_PIN, DISABLE_LEVEL);
}

// Convenience: absolute move to target position in steps (blocking)
static void moveToSteps(long targetSteps) {
#ifdef CHECK_SOFT_LIMITS
  if (targetSteps < MIN_POS_STEPS) targetSteps = MIN_POS_STEPS;
  if (targetSteps > MAX_POS_STEPS) targetSteps = MAX_POS_STEPS;
#endif
  long delta = targetSteps - currentPositionSteps;
  moveSteps(delta);
}


// Simultaneous blocking move for motors 2 & 3
// steps2 and steps3 can be positive or negative; motion is proportional.
static void moveStepsSync23(long steps2, long steps3) {
  long a = labs(steps2);
  long b = labs(steps3);
  if (a == 0 && b == 0) return;

  bool dir2High = (steps2 >= 0);
  bool dir3High = (steps3 >= 0);

  // Direction & enable
  digitalWrite(EN2_PIN, ENABLE_LEVEL);
  digitalWrite(EN3_PIN, ENABLE_LEVEL);
  digitalWrite(DIR2_PIN, dir2High ? HIGH : LOW);
  digitalWrite(DIR3_PIN, dir3High ? HIGH : LOW);

  // Bresenham-style DDA
  long n = (a > b) ? a : b;
  long acc2 = 0, acc3 = 0;

  unsigned long last = micros();
  for (long i = 0; i < n; i++) {
    // Wait until the next slot
    unsigned long now;
    do { now = micros(); } while ((unsigned long)(now - last) < interval23_us);
    last = now;

    bool do2 = false, do3 = false;
    acc2 += a;
    if (acc2 >= n) { acc2 -= n; do2 = true; }
    acc3 += b;
    if (acc3 >= n) { acc3 -= n; do3 = true; }

    // Rising edge(s)
    if (do2) digitalWrite(STEP2_PIN, HIGH);
    if (do3) digitalWrite(STEP3_PIN, HIGH);

    delayMicroseconds(STEP_PULSE_US);

    // Falling edge(s)
    if (do2) {
      digitalWrite(STEP2_PIN, LOW);
      currentPositionSteps2 += dir2High ? +1 : -1;
    }
    if (do3) {
      digitalWrite(STEP3_PIN, LOW);
      currentPositionSteps3 += dir3High ? +1 : -1;
    }
  }

  // Disable after motion
  digitalWrite(EN2_PIN, DISABLE_LEVEL);
  digitalWrite(EN3_PIN, DISABLE_LEVEL);
}

// Set shared speed for sync moves (steps/sec)
void setSpeed23SPS(long sps) {
  if (sps < 1) sps = 1;
  if (sps > 20000) sps = 20000;
  speed23_sps = sps;
  interval23_us = (unsigned long)(1000000.0 / (2.0 * (double)sps)); // 2 toggles per full step
  Serial.print("speed23 set to ");
  Serial.print(speed23_sps);
  Serial.print(" sps (interval=");
  Serial.print(interval23_us);
  Serial.println(" us)");
}





void HomeAxis() {

  // SAFETY: must be raised before homing moves
  if (!ensureToolheadRaised()) {
    Serial.println("HOMING ABORTED: toolhead not raised.");
    return;
  }

  digitalWrite(enablePin, ENABLE_LEVEL);
  motorEnabled = true;

  // If sitting on switch, back off first
  if (digitalRead(limitPin) == LOW) {
    digitalWrite(dirPin, !HOME_DIR);
    for (int i = 0; i < 800; i++) stepOnceTimed(); // ~1–2 mm, adjust
    // Update position (we moved away from HOME_DIR)
    currentPositionSteps -= 800;
  }

  // Fast approach
  digitalWrite(dirPin, HOME_DIR);
  unsigned long start = millis();
  const unsigned long timeoutMs = 15000;
  while (digitalRead(limitPin) == HIGH) {
    stepOnceTimed();
    // We’re moving toward HOME_DIR -> decrement or increment?
    currentPositionSteps += (HOME_DIR == HIGH) ? +1 : -1;

    if ((millis() - start) > timeoutMs) {
      Serial.println("HOMING ERROR: timeout (fast approach).");
      digitalWrite(enablePin, DISABLE_LEVEL);
      motorEnabled = false;
      return;
    }
  }
  Serial.println("Switch hit (fast).");

  // Back off to clear switch
  {
    const int backOff = 600;
    digitalWrite(dirPin, !HOME_DIR);
    for (int i = 0; i < backOff; i++) {
      stepOnceTimed();
      currentPositionSteps += (HOME_DIR == HIGH) ? -1 : +1; // opposite direction
    }
  }
  delay(10);

  // Slow re-approach
  unsigned long savedInterval = stepInterval;
  stepInterval = savedInterval * 3;   // slower for precision
  digitalWrite(dirPin, HOME_DIR);
  start = millis();
  while (digitalRead(limitPin) == HIGH) {
    stepOnceTimed();
    currentPositionSteps += (HOME_DIR == HIGH) ? +1 : -1;
    if ((millis() - start) > timeoutMs) {
      Serial.println("HOMING ERROR: timeout (slow approach).");
      stepInterval = savedInterval;
      digitalWrite(enablePin, DISABLE_LEVEL);
      motorEnabled = false;
      return;
    }
  }

  // We are ON the switch: define this exact electrical edge as 0.
  currentPositionSteps = 0;
  homed = true;

  // Optionally release the switch slightly while keeping position consistent
  {
    const int releaseSteps = 5; // tiny nudge off the switch
    digitalWrite(dirPin, !HOME_DIR);
    for (int i = 0; i < releaseSteps; i++) {
      stepOnceTimed();
      currentPositionSteps += (HOME_DIR == HIGH) ? -1 : +1;
    }
  }

  stepInterval = savedInterval;
  Serial.println("HOMING COMPLETE. pos=0");

  // Disable if you like:
  digitalWrite(enablePin, DISABLE_LEVEL);
  motorEnabled = false;
}





// Bounds-checking accessor (1-indexed for UI; returns -1 if invalid)
long getBasePos(uint8_t idx1) {
  if (idx1 == 0 || idx1 > NUM_BASES) return -1;
  return basePos[idx1 - 1];
}

// Set by value (1-indexed). Returns false if out of range.
bool baseSet(uint8_t idx1, long steps) {
  if (idx1 == 0 || idx1 > NUM_BASES) return false;
  basePos[idx1 - 1] = steps;
  return true;
}

// Set from current axis position (1-indexed). Returns false if out of range.
bool baseSetHere(uint8_t idx1) {
  if (idx1 == 0 || idx1 > NUM_BASES) return false;
  basePos[idx1 - 1] = currentPositionSteps;
  return true;
}

// Move to a base syringe position (1-indexed).
// Wraps your moveToSteps() which already enforces soft limits and toolhead-raise safety.
bool goToBase(uint8_t idx1) {
  long tgt = getBasePos(idx1);
  if (tgt < 0) {
    Serial.println("ERROR: base index out of range.");
    return false;
  }
#ifdef CHECK_SOFT_LIMITS
  if (tgt < MIN_POS_STEPS || tgt > MAX_POS_STEPS) {
    Serial.println("ERROR: base target outside soft limits.");
    return false;
  }
#endif
  digitalWrite(enablePin, ENABLE_LEVEL);
  moveToSteps(tgt);
  digitalWrite(enablePin, DISABLE_LEVEL);
  Serial.print("At Base #");
  Serial.print(idx1);
  Serial.print(" (steps=");
  Serial.print(tgt);
  Serial.println(")");
  return true;
}



void setup() {
  pinMode(stepPin, OUTPUT);
  pinMode(dirPin, OUTPUT);
  pinMode(enablePin, OUTPUT);

  pinMode(STEP2_PIN, OUTPUT);
  pinMode(DIR2_PIN, OUTPUT);
  pinMode(EN2_PIN, OUTPUT);

  pinMode(STEP3_PIN, OUTPUT);
  pinMode(DIR3_PIN, OUTPUT);
  pinMode(EN3_PIN, OUTPUT);

  digitalWrite(EN2_PIN, DISABLE_LEVEL);
  digitalWrite(EN3_PIN, DISABLE_LEVEL);

  pinMode(limitPin, INPUT_PULLUP);  // switch = LOW when pressed
  pinMode(raisedPin, INPUT_PULLUP);  // pressed = LOW


  digitalWrite(enablePin, HIGH); // Start disabled
  digitalWrite(dirPin, HIGH);    // Set direction

  Serial.begin(115200);
  Serial.println("Commands: on, off, speed <steps/sec>, dir <0|1>");

  pwm.begin();
  pwm.setPWMFreq(60); // standard analog servos ~60 Hz
  delay(10);
}


void loop() {
  

  handleSerial();

  if (motorEnabled) {
    unsigned long now = micros();
    if (now - lastStepTime >= stepInterval) {
      lastStepTime = now;
      digitalWrite(stepPin, HIGH);
      delayMicroseconds(2);      // Short pulse
      digitalWrite(stepPin, LOW);
    }
  }
}

void handleSerial() {
  static String input = "";
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n') {
      input.trim();
      if (input.length() == 0) { input = ""; continue; }

      Serial.print("received: ");
      Serial.println(input);

      // ----- Helpers for parsing -----
      auto getArg = [](const String& s, int startPos = 0) -> String {
        int sp = s.indexOf(' ', startPos);
        if (sp < 0) return "";
        int next = sp + 1;
        // skip extra spaces
        while (next < (int)s.length() && s[next] == ' ') next++;
        return s.substring(next);
      };
      auto splitTwo = [](const String& s, int startPos, long &a, long &b) -> bool {
        int sp = s.indexOf(' ', startPos);
        if (sp < 0) return false;
        String A = s.substring(startPos, sp); A.trim();
        String B = s.substring(sp + 1);      B.trim();
        if (A.length() == 0 || B.length() == 0) return false;
        a = A.toInt();
        b = B.toInt();
        return true;
      };

      // ================= CORE CONTROL =================
      if (input == "on") {
        if (!ensureToolheadRaised()) {
          motorEnabled = false;
          digitalWrite(enablePin, DISABLE_LEVEL);
          Serial.println("Motor NOT enabled: toolhead not raised.");
        } else {
          digitalWrite(enablePin, ENABLE_LEVEL);
          motorEnabled = true;
          Serial.println("Motor ON");
        }
      }
      else if (input == "off") {
        motorEnabled = false;
        digitalWrite(enablePin, DISABLE_LEVEL);
        Serial.println("Motor OFF");
      }
      else if (input.startsWith("speed ")) {
        int sps = getArg(input, 5).toInt(); // steps per second
        if (sps > 0) {
          stepInterval = 1000000UL / (unsigned long)sps;
          Serial.print("Speed set to ");
          Serial.print(sps);
          Serial.println(" steps/sec");
        } else {
          Serial.println("Usage: speed <steps_per_sec>");
        }
      }
      else if (input.startsWith("dir ")) {
        int d = getArg(input, 3).toInt();
        digitalWrite(dirPin, d ? HIGH : LOW);
        Serial.print("Direction set to ");
        Serial.println(d);
      }
      else if (input == "home") {
        Serial.println("Homing...");
        HomeAxis();
      }

      // ================= POSITION / MOTION STEP #1 =================
      else if (input == "pos") {
        float mm = currentPositionSteps / STEPS_PER_MM;
        Serial.print("POS steps=");
        Serial.print(currentPositionSteps);
        Serial.print("  mm=");
        Serial.println(mm, 3);
      }
      else if (input.startsWith("move ")) {              // relative steps
        long steps = getArg(input, 4).toInt();
        digitalWrite(enablePin, ENABLE_LEVEL);
        moveSteps(steps);
        digitalWrite(enablePin, DISABLE_LEVEL);
        Serial.println("OK");
      }
      else if (input.startsWith("movemm ")) {            // relative mm
        float mm = getArg(input, 6).toFloat();
        long steps = (long)lround(mm * STEPS_PER_MM);
        digitalWrite(enablePin, ENABLE_LEVEL);
        moveSteps(steps);
        digitalWrite(enablePin, DISABLE_LEVEL);
        Serial.println("OK");
      }
      else if (input.startsWith("goto ")) {              // absolute steps
        long tgt = getArg(input, 4).toInt();
#ifdef CHECK_SOFT_LIMITS
        if (tgt < MIN_POS_STEPS) tgt = MIN_POS_STEPS;
        if (tgt > MAX_POS_STEPS) tgt = MAX_POS_STEPS;
#endif
        digitalWrite(enablePin, ENABLE_LEVEL);
        moveToSteps(tgt);
        digitalWrite(enablePin, DISABLE_LEVEL);
        Serial.println("OK");
      }
      else if (input.startsWith("gotomm ")) {            // absolute mm
        float mm = getArg(input, 6).toFloat();
        long tgt = (long)lround(mm * STEPS_PER_MM);
#ifdef CHECK_SOFT_LIMITS
        if (tgt < MIN_POS_STEPS) tgt = MIN_POS_STEPS;
        if (tgt > MAX_POS_STEPS) tgt = MAX_POS_STEPS;
#endif
        digitalWrite(enablePin, ENABLE_LEVEL);
        moveToSteps(tgt);
        digitalWrite(enablePin, DISABLE_LEVEL);
        Serial.println("OK");
      }

      // ===================== BASE POSITIONS =======================
      // Move to Base Syringe i (1-indexed). Both names supported:
      // "gobase <i>"  or  "base <i>"
      else if (input.startsWith("gobase ") || input.startsWith("base ")) {
        int idx = input.startsWith("gobase ") ? getArg(input, 6).toInt()
                                              : getArg(input, 4).toInt();
        goToBase((uint8_t)idx);
      }
      else if (input.startsWith("baseset ")) {           // baseset <i> <steps>
        long i, s; 
        if (splitTwo(input, 8, i, s) && baseSet((uint8_t)i, s)) {
          Serial.println("OK baseset");
        } else {
          Serial.println("Usage: baseset <idx 1..N> <steps>");
        }
      }
      else if (input.startsWith("basesethere ")) {       // basesethere <i>
        int idx = getArg(input, 11).toInt();
        if (baseSetHere((uint8_t)idx)) Serial.println("OK basesethere");
        else Serial.println("ERROR: bad base index.");
      }
    
      else if (input == "baselist") {
        for (uint8_t i = 1; i <= NUM_BASES; i++) {
          Serial.print("#"); Serial.print(i);
          Serial.print(" = "); Serial.println(getBasePos(i));
        }
      }

      // ===================== SERVOS =======================
      else if (input.startsWith("servopulse ")) {
        // servopulse <channel> <microseconds>
        long ch, us;
        if (splitTwo(input, 10, ch, us) && ch >= 0 && ch <= 15) {
          setServoPulseRaw((uint8_t)ch, (int)us);
          Serial.print("Servo "); Serial.print(ch);
          Serial.print(" set to raw pulse ");
          Serial.println(us);
        } else {
          Serial.println("Usage: servopulse <channel 0-15> <pulse_us>");
        }
      }
      else if (input.startsWith("servo ")) {
        // servo <channel> <angle>
        String rest = getArg(input, 5);
        int sp = rest.indexOf(' ');
        if (sp > 0) {
          int ch = rest.substring(0, sp).toInt();
          int ang = rest.substring(sp + 1).toInt();
          if (ch < 0 || ch > 15) Serial.println("Error: channel must be 0–15");
          else setServoAngle((uint8_t)ch, ang);
        } else {
          Serial.println("Usage: servo <channel 0-15> <angle 0-180>");
        }
      }
      else if (input == "raise") {
        raiseToolhead();
      }
      else if (input.startsWith("servoslow ")) {
        // servoslow <channel> <angle> [delay_ms]
        String rest = getArg(input, 9);
        int sp = rest.indexOf(' ');
        if (sp > 0) {
          int ch = rest.substring(0, sp).toInt();
          String rest2 = rest.substring(sp + 1);
          int sp2 = rest2.indexOf(' ');
          int ang = (sp2 > 0) ? rest2.substring(0, sp2).toInt() : rest2.toInt();
          int dly = (sp2 > 0) ? rest2.substring(sp2 + 1).toInt() : 15;
          if (ch < 0 || ch > 15) Serial.println("Error: channel must be 0–15");
          else setServoAngleSlow((uint8_t)ch, ang, dly);
        } else {
          Serial.println("Usage: servoslow <channel> <angle 0-180> [delay_ms]");
        }
      }

      else if (input.startsWith("servodual ")) {
        // servodual <chA> <tgtA> <chB> <tgtB> [delayA_ms] [delayB_ms] [stepA_deg] [stepB_deg]
        // Examples:
        //   servodual 0 150 1 60
        //   servodual 2 30  4 120 10 25
        //   servodual 3 99  5 10  15 30 1 2
        String rest = getArg(input, 9);
        // tokenization
        int p1 = rest.indexOf(' '); if (p1 < 0) { Serial.println("Usage: servodual <chA> <tgtA> <chB> <tgtB> [dA] [dB] [sA] [sB]"); }
        else {
          String t1 = rest.substring(0, p1); rest = rest.substring(p1 + 1); rest.trim();
          int p2 = rest.indexOf(' '); if (p2 < 0) { Serial.println("Usage: servodual <chA> <tgtA> <chB> <tgtB> [dA] [dB] [sA] [sB]"); }
          else {
            String t2 = rest.substring(0, p2); rest = rest.substring(p2 + 1); rest.trim();
            int p3 = rest.indexOf(' '); if (p3 < 0) { Serial.println("Usage: servodual <chA> <tgtA> <chB> <tgtB> [dA] [dB] [sA] [sB]"); }
            else {
              String t3 = rest.substring(0, p3); rest = rest.substring(p3 + 1); rest.trim();
              int p4 = rest.indexOf(' ');
              String t4, tail;
              if (p4 < 0) { t4 = rest; tail = ""; }
              else { t4 = rest.substring(0, p4); tail = rest.substring(p4 + 1); tail.trim(); }

              int chA = t1.toInt();
              int tgtA = t2.toInt();
              int chB = t3.toInt();
              int tgtB = t4.toInt();

              // defaults
              uint16_t dA = 20, dB = 20;
              uint8_t  sA = 1,  sB = 1;

              // parse optional tail: up to 4 ints
              if (tail.length() > 0) {
                // split by spaces
                int q1 = tail.indexOf(' ');
                if (q1 < 0) { dA = tail.toInt(); }
                else {
                  String u1 = tail.substring(0, q1); tail = tail.substring(q1 + 1); tail.trim();
                  dA = u1.toInt();
                  int q2 = tail.indexOf(' ');
                  if (q2 < 0) { dB = tail.toInt(); }
                  else {
                    String u2 = tail.substring(0, q2); tail = tail.substring(q2 + 1); tail.trim();
                    dB = u2.toInt();
                    int q3 = tail.indexOf(' ');
                    if (q3 < 0) { sA = (uint8_t)tail.toInt(); }
                    else {
                      String u3 = tail.substring(0, q3); tail = tail.substring(q3 + 1); tail.trim();
                      sA = (uint8_t)u3.toInt();
                      if (tail.length() > 0) sB = (uint8_t)tail.toInt();
                    }
                  }
                }
              }

              // bounds checking
              if (chA < 0 || chA > 15 || chB < 0 || chB > 15) {
                Serial.println("Error: channel must be 0–15");
              } else {
                setServoAnglesDual((uint8_t)chA, tgtA, (uint8_t)chB, tgtB, dA, dB, sA, sB);
              }
            }
          }
        }
      }


      // ===================== STEPPERS 2 & 3 =======================
      else if (input.startsWith("move2 ")) {
        long s = getArg(input, 5).toInt();
        moveSteps2(s);
        Serial.println("OK move2");
      }
      else if (input.startsWith("move3 ")) {
        long s = getArg(input, 5).toInt();
        moveSteps3(s);
        Serial.println("OK move3");
      }
      else if (input.startsWith("speed23 ")) {
        long sps = getArg(input, 7).toInt();
        setSpeed23SPS(sps);
      }
      else if (input.startsWith("m23 ")) {
        // m23 <steps2> <steps3>
        long s2, s3;
        if (splitTwo(input, 4, s2, s3)) {
          moveStepsSync23(s2, s3);
          Serial.println("OK m23");
        } else {
          Serial.println("Usage: m23 <steps2> <steps3>");
        }
      }
      else if (input.startsWith("link ")) {
        long s = getArg(input, 4).toInt();
        moveStepsSync23(s, -s);
        Serial.println("OK link");
      }

      // ===================== FALLBACK =======================
      else {
        Serial.println("Invalid command.");
      }

      input = "";
    } else if (c != '\r') {
      input += c;
    }
  }
}

