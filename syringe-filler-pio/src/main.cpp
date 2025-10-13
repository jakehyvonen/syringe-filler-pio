#include <Arduino.h>


// ====== ESP32 PIN REMAP (choose GPIOs that suit your board/wiring) ======
// Original Mega pins -> ESP32 GPIOs
#define stepPin     4    // was 22
#define dirPin      16   // was 23
#define enablePin   17   // was 24

#define STEP2_PIN   18   // was 25
#define DIR2_PIN    19   // was 26
#define EN2_PIN     5    // was 27

#define STEP3_PIN   12   // was 28
#define DIR3_PIN    14   // was 29
#define EN3_PIN     27   // was 30

// Base stepper ENABLE lines (5 channels)
// Pick 5 outputs; adjust to your wiring
#define NUM_BASES 5
const uint8_t BASE_EN_PINS[NUM_BASES] = {23, 26, 32, 33, 13}; // was {43,45,47,49,51}

// ---- Limit switch wiring ----
// NOTE: GPIO34/35 are input-only and have NO internal pullups on ESP32.
// Either: (A) keep these and add external pull-ups to 3.3V, or
//         (B) move to pullup-capable pins and keep INPUT_PULLUP below.
#define limitPin   34    // was 39
#define raisedPin  35    // was 40

// ================== LIBS ==================
#include <Wire.h>
#include <EEPROM.h>                  // works on ESP32 with EEPROM.begin()
#include <Adafruit_PWMServoDriver.h> // PCA9685
#include <Adafruit_ADS1X15.h>        // ADS1115
#include "esp_log.h"


// === forward declarations ===
bool ensureToolheadRaised(uint16_t timeout_ms = 1200);
bool isToolheadRaised();
bool goToBase(uint8_t idx1);
long getBasePos(uint8_t idx1);
bool baseSet(uint8_t idx1, long steps);
bool baseSetHere(uint8_t idx1);


// ===== Defaults/calibration/EEPROM (unchanged structs) =====
long basePos[NUM_BASES] = { 3330, 5480, 3330, 3330, 3330 };
const int EEPROM_BASE_ADDR = 0;

// ====== ADS1115 setup (I2C) ======
Adafruit_ADS1115 ads;     // default address 0x48
#define NUM_POTS 1
// Use ADS channel numbers instead of A0..A5. Keep your array API unchanged.
const uint8_t POT_PINS[NUM_POTS] = { 0 }; // ADS1115 channel 0 == "A0" logically

// Exponential moving average strength and print hysteresis
const uint8_t POT_EMA_SHIFT = 3;    // 1/8 new sample each poll
const uint8_t POT_REPORT_HYST = 3;  // print only if change >= this many ADC counts

static uint16_t pot_raw[NUM_POTS];
static uint16_t pot_filtered[NUM_POTS];
static uint16_t pot_last_reported[NUM_POTS];
static bool pots_inited = false;

// ---- motion & config (unchanged) ----
#define HOME_DIR LOW
#define DISABLE_LEVEL HIGH
#define ENABLE_LEVEL  LOW
const float STEPS_PER_MM = 80.0f;

#define CHECK_SOFT_LIMITS
const long MIN_POS_STEPS = 0;
const long MAX_POS_STEPS = (long)(200.0f * STEPS_PER_MM);

volatile long currentPositionSteps  = 0;
volatile long currentPositionSteps2 = 0;
volatile long currentPositionSteps3 = 0;

#define STEP_PULSE_US 4

static long speed23_sps = 800;
static unsigned long interval23_us = 1000000UL / (2UL * 800UL);

bool homed = false;

long lastStepTime = 0;
unsigned long stepInterval = 1000;  // microseconds between steps (500 sps)
bool motorEnabled = false;
bool stepState = false;

// ---------- I2C presence flags ----------


// Presence probe
bool i2cPresent(uint8_t addr) {
  Wire.beginTransmission(addr);
  return (Wire.endTransmission() == 0);
}


// globals
bool g_hasPCA = false;
bool g_hasADS = false;   // true if any ADS1115 we intend to use is present

// Simple wrappers so we can guard calls with readable names
inline bool PCA_OK() { return g_hasPCA; }
inline bool ADS_OK() { return g_hasADS; }




// ===== SERVO / PCA9685 (unchanged API) =====
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

#define SERVO_MIN 150
#define SERVO_MAX 600
#define raisedPos 99
#define couplingPos1 132
#define coupledPos 11
#define decoupledPos 151
#define TOOLHEAD_SERVO 3
#define COUPLING_SERVO 5

static int currentAngles[16];
static bool currentAnglesInit = false;

static inline void _ensureAnglesInit() {
  if (!currentAnglesInit) {
    for (int i = 0; i < 16; ++i) currentAngles[i] = 90;
    currentAnglesInit = true;
  }
}

bool isToolheadRaised() {
  // If you kept GPIO34/35, ensure external pull-up resistor to 3.3V.
  return (digitalRead(raisedPin) == LOW);
}

void setServoPulseRaw(uint8_t servoNum, int pulseLength) {
  if (!PCA_OK()) return;
  pwm.writeMicroseconds(servoNum, pulseLength);
}

void setServoAngle(uint8_t channel, int angle) {
  if (!PCA_OK()) return;
  _ensureAnglesInit();
  if (angle < 0) angle = 0;
  if (angle > 180) angle = 180;

  uint16_t pulselen = map(angle, 0, 180, SERVO_MIN, SERVO_MAX);
  pwm.setPWM(channel, 0, pulselen);

  currentAngles[channel] = angle;
  Serial.print("Servo "); Serial.print(channel);
  Serial.print(" -> ");   Serial.print(angle);
  Serial.println(" deg");
}

void raiseToolhead() {
  setServoAngle(TOOLHEAD_SERVO, raisedPos);
  delay(100);
  setServoAngle(COUPLING_SERVO, decoupledPos);
}

void setServoAngleSlow(uint8_t channel, int targetAngle, int stepDelay = 23) {
  if (!PCA_OK()) return;
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

  Serial.print("Servo "); Serial.print(channel);
  Serial.print(" slowly moved to "); Serial.println(targetAngle);
}

void setServoAnglesDual(uint8_t chA, int tgtA,
                        uint8_t chB, int tgtB,
                        uint16_t stepDelayA = 20, uint16_t stepDelayB = 20,
                        uint8_t stepSizeA = 1, uint8_t stepSizeB = 1) {
  if (!PCA_OK()) return;
  _ensureAnglesInit();

  if (tgtA < 0) tgtA = 0; if (tgtA > 180) tgtA = 180;
  if (tgtB < 0) tgtB = 0; if (tgtB > 180) tgtB = 180;
  if (stepDelayA == 0) stepDelayA = 1;
  if (stepDelayB == 0) stepDelayB = 1;
  if (stepSizeA == 0) stepSizeA = 1;
  if (stepSizeB == 0) stepSizeB = 1;

  int curA = currentAngles[chA];
  int curB = currentAngles[chB];

  if (curA == tgtA && curB == tgtB) return;

  int dirA = (tgtA > curA) ? +1 : -1;
  int dirB = (tgtB > curB) ? +1 : -1;
  if (tgtA == curA) dirA = 0;
  if (tgtB == curB) dirB = 0;

  unsigned long lastA = millis(), lastB = millis();

  auto writeA = [&](int a) {
    uint16_t p = map(a, 0, 180, SERVO_MIN, SERVO_MAX);
    pwm.setPWM(chA, 0, p);
  };
  auto writeB = [&](int b) {
    uint16_t p = map(b, 0, 180, SERVO_MIN, SERVO_MAX);
    pwm.setPWM(chB, 0, p);
  };

  writeA(curA);
  writeB(curB);

  while (true) {
    unsigned long now = millis();
    bool progressed = false;

    if (dirA != 0 && (now - lastA) >= stepDelayA) {
      lastA = now;
      int nextA = curA + dirA * (int)stepSizeA;
      if ((dirA > 0 && nextA > tgtA) || (dirA < 0 && nextA < tgtA)) nextA = tgtA;
      if (nextA != curA) { curA = nextA; writeA(curA); progressed = true; }
      if (curA == tgtA) dirA = 0;
    }
    if (dirB != 0 && (now - lastB) >= stepDelayB) {
      lastB = now;
      int nextB = curB + dirB * (int)stepSizeB;
      if ((dirB > 0 && nextB > tgtB) || (dirB < 0 && nextB < tgtB)) nextB = tgtB;
      if (nextB != curB) { curB = nextB; writeB(curB); progressed = true; }
      if (curB == tgtB) dirB = 0;
    }

    if (dirA == 0 && dirB == 0) break;
    if (!progressed) delay(1);
  }

  currentAngles[chA] = curA;
  currentAngles[chB] = curB;

  Serial.print("Servos "); Serial.print(chA); Serial.print("->"); Serial.print(tgtA);
  Serial.print(", ");     Serial.print(chB); Serial.print("->"); Serial.print(tgtB);
  Serial.println(" (dual slow)");
}

void coupleSyringes() {
  setServoAngle(TOOLHEAD_SERVO, couplingPos1);
  delay(71);
  setServoAngle(COUPLING_SERVO, 151);
  delay(71);
  setServoPulseRaw(TOOLHEAD_SERVO, 0);
  delay(71);
  setServoAngle(COUPLING_SERVO, 11);
}

// ===== Bases (unchanged API) =====
void initBasesNoMCP() {
  for (uint8_t i = 0; i < NUM_BASES; ++i) {
    pinMode(BASE_EN_PINS[i], OUTPUT);
    digitalWrite(BASE_EN_PINS[i], HIGH); // default disabled
  }
}

void disableAllBases() {
  for (uint8_t i = 0; i < NUM_BASES; ++i) digitalWrite(BASE_EN_PINS[i], HIGH);
}

bool selectBaseStepper(uint8_t idx1) {
  if (idx1 == 0) {
    disableAllBases();
    Serial.println("Base: none");
    return true;
  }
  if (idx1 > NUM_BASES) {
    Serial.println("ERROR: base index out of range");
    return false;
  }
  for (uint8_t i = 0; i < NUM_BASES; ++i)
    digitalWrite(BASE_EN_PINS[i], (i == (idx1 - 1)) ? LOW : HIGH);

  Serial.print("Base selected: "); Serial.println(idx1);
  return true;
}

// ===== Step timing (unchanged) =====
static inline void stepOnceTimed() {
  static bool localStepState = false;
  static unsigned long last = micros();
  unsigned long now;
  do { now = micros(); } while ((unsigned long)(now - last) < stepInterval);
  last = now;

  localStepState = !localStepState;
  digitalWrite(stepPin, localStepState ? HIGH : LOW);
}

static inline void stepOnceTimedOnPin(uint8_t pin, unsigned long interval_us) {
  static unsigned long last2 = 0, last3 = 0;
  static bool state2 = false, state3 = false;

  unsigned long &last = (pin == STEP2_PIN) ? last2 : last3;
  bool &state = (pin == STEP2_PIN) ? state2 : state3;

  unsigned long now;
  do { now = micros(); } while ((unsigned long)(now - last) < interval_us);
  last = now;

  state = !state;
  digitalWrite(pin, state ? HIGH : LOW);
}

// ===== Movement (unchanged logic) =====
static void moveSteps(long steps) {
  if (steps == 0) return;
  if (!ensureToolheadRaised()) return;

  bool dirHigh = (steps > 0);
  long todo = labs(steps);
  digitalWrite(dirPin, dirHigh ? HIGH : LOW);

  for (long i = 0; i < todo; i++) {
    stepOnceTimed();
    currentPositionSteps += dirHigh ? +1 : -1;
    stepOnceTimed();
  }
}

static void moveSteps2(long steps) {
  if (steps == 0) return;
  bool dirHigh = (steps > 0);
  long todo = labs(steps);

  digitalWrite(EN2_PIN, ENABLE_LEVEL);
  digitalWrite(DIR2_PIN, dirHigh ? HIGH : LOW);

  for (long i = 0; i < todo; i++) {
    stepOnceTimedOnPin(STEP2_PIN, interval23_us);
    currentPositionSteps2 += dirHigh ? +1 : -1;
    stepOnceTimedOnPin(STEP2_PIN, interval23_us);
  }

  digitalWrite(EN2_PIN, DISABLE_LEVEL);
}

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

static void moveToSteps(long targetSteps) {
#ifdef CHECK_SOFT_LIMITS
  if (targetSteps < MIN_POS_STEPS) targetSteps = MIN_POS_STEPS;
  if (targetSteps > MAX_POS_STEPS) targetSteps = MAX_POS_STEPS;
#endif
  long delta = targetSteps - currentPositionSteps;
  moveSteps(delta);
}

static void moveStepsSync23(long steps2, long steps3) {
  long a = labs(steps2), b = labs(steps3);
  if (a == 0 && b == 0) return;

  bool dir2High = (steps2 >= 0);
  bool dir3High = (steps3 >= 0);

  digitalWrite(EN2_PIN, ENABLE_LEVEL);
  digitalWrite(EN3_PIN, ENABLE_LEVEL);
  digitalWrite(DIR2_PIN, dir2High ? HIGH : LOW);
  digitalWrite(DIR3_PIN, dir3High ? HIGH : LOW);

  long n = (a > b) ? a : b;
  long acc2 = 0, acc3 = 0;

  unsigned long last = micros();
  for (long i = 0; i < n; i++) {
    unsigned long now;
    do { now = micros(); } while ((unsigned long)(now - last) < interval23_us);
    last = now;

    bool do2 = false, do3 = false;
    acc2 += a; if (acc2 >= n) { acc2 -= n; do2 = true; }
    acc3 += b; if (acc3 >= n) { acc3 -= n; do3 = true; }

    if (do2) digitalWrite(STEP2_PIN, HIGH);
    if (do3) digitalWrite(STEP3_PIN, HIGH);
    delayMicroseconds(STEP_PULSE_US);
    if (do2) { digitalWrite(STEP2_PIN, LOW); currentPositionSteps2 += dir2High ? +1 : -1; }
    if (do3) { digitalWrite(STEP3_PIN, LOW); currentPositionSteps3 += dir3High ? +1 : -1; }
  }

  digitalWrite(EN2_PIN, DISABLE_LEVEL);
  digitalWrite(EN3_PIN, DISABLE_LEVEL);
}

void setSpeed23SPS(long sps) {
  if (sps < 1) sps = 1;
  if (sps > 20000) sps = 20000;
  speed23_sps = sps;
  interval23_us = (unsigned long)(1000000.0 / (2.0 * (double)sps));
  Serial.print("speed23 set to "); Serial.print(speed23_sps);
  Serial.print(" sps (interval=");  Serial.print(interval23_us);
  Serial.println(" us)");
}

// ===== Homing (unchanged logic) =====
bool ensureToolheadRaised(uint16_t timeout_ms) {
  if (!PCA_OK()) {
    Serial.println("ERROR: No PCA9685; cannot raise toolhead. Movement blocked.");
    return false;
  }

  if (isToolheadRaised()) return true;

  setServoAngle(COUPLING_SERVO, decoupledPos);
  delay(100);

  setServoAngle(TOOLHEAD_SERVO, raisedPos);

  unsigned long start = millis();
  while (!isToolheadRaised() && (millis() - start) < timeout_ms) {
    setServoAngle(COUPLING_SERVO, decoupledPos);
    delay(100);
    setServoAngle(TOOLHEAD_SERVO, raisedPos);
    delay(100);
  }

  if (!isToolheadRaised()) {
    Serial.println("ERROR: Toolhead not raised (timeout). Movement blocked.");
    return false;
  }
  return true;
}

void HomeAxis() {
  if (!ensureToolheadRaised()) {
    Serial.println("HOMING ABORTED: toolhead not raised.");
    return;
  }

  digitalWrite(enablePin, ENABLE_LEVEL);
  motorEnabled = true;

  if (digitalRead(limitPin) == LOW) {
    digitalWrite(dirPin, !HOME_DIR);
    for (int i = 0; i < 800; i++) stepOnceTimed();
    currentPositionSteps -= 800;
  }

  digitalWrite(dirPin, HOME_DIR);
  unsigned long start = millis();
  const unsigned long timeoutMs = 15000;
  while (digitalRead(limitPin) == HIGH) {
    stepOnceTimed();
    currentPositionSteps += (HOME_DIR == HIGH) ? +1 : -1;

    if ((millis() - start) > timeoutMs) {
      Serial.println("HOMING ERROR: timeout (fast approach).");
      digitalWrite(enablePin, DISABLE_LEVEL);
      motorEnabled = false;
      return;
    }
  }
  Serial.println("Switch hit (fast).");

  {
    const int backOff = 600;
    digitalWrite(dirPin, !HOME_DIR);
    for (int i = 0; i < backOff; i++) {
      stepOnceTimed();
      currentPositionSteps += (HOME_DIR == HIGH) ? -1 : +1;
    }
  }
  delay(10);

  unsigned long savedInterval = stepInterval;
  stepInterval = savedInterval * 3;
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

  currentPositionSteps = 0;
  homed = true;

  {
    const int releaseSteps = 5;
    digitalWrite(dirPin, !HOME_DIR);
    for (int i = 0; i < releaseSteps; i++) {
      stepOnceTimed();
      currentPositionSteps += (HOME_DIR == HIGH) ? -1 : +1;
    }
  }

  stepInterval = savedInterval;
  Serial.println("HOMING COMPLETE. pos=0");

  digitalWrite(enablePin, DISABLE_LEVEL);
  motorEnabled = false;
}

// ===== Bases helpers (unchanged) =====
long getBasePos(uint8_t idx1) {
  if (idx1 == 0 || idx1 > NUM_BASES) return -1;
  return basePos[idx1 - 1];
}

bool baseSet(uint8_t idx1, long steps) {
  if (idx1 == 0 || idx1 > NUM_BASES) return false;
  basePos[idx1 - 1] = steps;
  return true;
}

bool baseSetHere(uint8_t idx1) {
  if (idx1 == 0 || idx1 > NUM_BASES) return false;
  basePos[idx1 - 1] = currentPositionSteps;
  return true;
}

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
  Serial.print("At Base #"); Serial.print(idx1);
  Serial.print(" (steps=");   Serial.print(tgt);
  Serial.println(")");
  return true;
}

// ======== ADS1115 helper: read single-ended channel -> 0..1023 ========
static inline uint16_t readPotADC(uint8_t potIndex) {
  if (!ADS_OK()) return 0;             // no hardware -> stable zero

  // potIndex maps to ADS channel via POT_PINS[]
  int ch = POT_PINS[potIndex];
  int16_t v = ads.readADC_SingleEnded(ch); // signed
  if (v < 0) v = 0;                        // floor at 0 for single-ended
  // ADS1115 full-scale at GAIN_ONE is +/-4.096V => 0..32767 counts for 0..+FS
  // Scale to 0..1023 for compatibility with your prints/math
  uint32_t scaled = (uint32_t)v * 1023UL / 32767UL;
  if (scaled > 1023UL) scaled = 1023UL;
  return (uint16_t)scaled;
}

// ===== Pots (same API; ADC source swapped) =====
void initPots() {
  if (!ADS_OK()) {
    // Seed arrays to zero so prints are stable
    for (uint8_t i=0;i<NUM_POTS;i++) pot_raw[i]=pot_filtered[i]=pot_last_reported[i]=0;
    pots_inited = true;
    Serial.println("Pots: ADS not present; readings disabled.");
    return;
  }
  for (uint8_t i = 0; i < NUM_POTS; ++i) {
    uint16_t r = readPotADC(i);
    pot_raw[i] = r;
    pot_filtered[i] = r;
  }
  for (uint8_t k = 0; k < 8; ++k) {
    for (uint8_t i = 0; i < NUM_POTS; ++i) {
      uint16_t r = readPotADC(i);
      pot_filtered[i] = (pot_filtered[i] + r) >> 1;
    }
    delay(2);
  }
  for (uint8_t i = 0; i < NUM_POTS; ++i) {
    pot_last_reported[i] = pot_filtered[i];
    Serial.print("pot["); Serial.print(i); Serial.print("](init)=");
    Serial.println(pot_filtered[i]);
  }
  pots_inited = true;
}

void pollPots() {
  if (!pots_inited) initPots();
  if (!ADS_OK()) return;   // nothing to do

  for (uint8_t i = 0; i < NUM_POTS; ++i) {
    uint16_t r = readPotADC(i);
    pot_raw[i] = r;

    pot_filtered[i] += (int16_t(r) - int16_t(pot_filtered[i])) >> POT_EMA_SHIFT;

    int diff = int(pot_filtered[i]) - int(pot_last_reported[i]);
    if (diff < 0) diff = -diff;
    if (diff >= POT_REPORT_HYST) {
      pot_last_reported[i] = pot_filtered[i];
      Serial.print("pot["); Serial.print(i); Serial.print("]=");
      Serial.println(pot_filtered[i]);  // now 0..1023-equivalent
    }
  }
}

uint16_t getPotRaw(uint8_t idx) {
  if (idx >= NUM_POTS) return 0;
  return pot_raw[idx];
}
uint16_t getPotFiltered(uint8_t idx) {
  if (idx >= NUM_POTS) return 0;
  return pot_filtered[idx];
}
float getPotPercent(uint8_t idx) {
  if (idx >= NUM_POTS) return 0.0f;
  return (pot_filtered[idx] * 100.0f) / 1023.0f;
}

// ===== Pot-driven move (ADC source swapped, behavior same) =====
bool move2UntilPot_simple(uint16_t target_adc, long sps) {
  if (!ADS_OK()) {
    Serial.println("move2UntilPot: ADS not present.");
    return false;
  }
  if (sps < 1) sps = 1;
  if (sps > 20000) sps = 20000;

  const unsigned long interval_us   = (unsigned long)(1000000.0 / (2.0 * (double)sps));
  const uint8_t       hysteresis    = 3;
  const unsigned long timeout_ms    = 20000UL;

  if (!pots_inited) initPots();

  uint16_t raw0 = readPotADC(0);
  pot_raw[0] = raw0;
  pot_filtered[0] += (int16_t(raw0) - int16_t(pot_filtered[0])) >> POT_EMA_SHIFT;

  bool dirHigh = (target_adc > pot_filtered[0]);

  digitalWrite(EN2_PIN, ENABLE_LEVEL);
  digitalWrite(DIR2_PIN, dirHigh ? HIGH : LOW);

  unsigned long last    = micros();
  unsigned long startMs = millis();

  while (true) {
    if (timeout_ms && (millis() - startMs) > timeout_ms) {
      digitalWrite(EN2_PIN, DISABLE_LEVEL);
      Serial.println("move2UntilPot: TIMEOUT");
      return false;
    }

    unsigned long now;
    do { now = micros(); } while ((unsigned long)(now - last) < interval_us);
    last = now;

    digitalWrite(STEP2_PIN, HIGH);
    delayMicroseconds(STEP_PULSE_US);
    digitalWrite(STEP2_PIN, LOW);

    currentPositionSteps2 += dirHigh ? +1 : -1;

    raw0 = readPotADC(0);
    pot_raw[0] = raw0;
    pot_filtered[0] += (int16_t(raw0) - int16_t(pot_filtered[0])) >> POT_EMA_SHIFT;

    Serial.print("pot[0]="); Serial.println(pot_filtered[0]);

    int diff = int(pot_filtered[0]) - int(pot_last_reported[0]);
    if (diff < 0) diff = -diff;
    if (diff >= POT_REPORT_HYST) {
      pot_last_reported[0] = pot_filtered[0];
    }

    uint16_t pot = pot_filtered[0];
    if (dirHigh) {
      if (pot >= (uint16_t)(target_adc - hysteresis)) break;
    } else {
      if (pot <= (uint16_t)(target_adc + hysteresis)) break;
    }
  }

  digitalWrite(EN2_PIN, DISABLE_LEVEL);
  Serial.print("pot[0]="); Serial.println(pot_filtered[0]);
  return true;
}


// ===== Setup / Loop / Serial (minor ESP32 notes inline) =====

void setup() {
  // --- Basic system setup ---
  esp_log_level_set("*", ESP_LOG_ERROR); // Silence ESP-IDF spam
  Serial.begin(115200);
  delay(200);
  Serial.println("\n=== Syringe Fill Control (ESP32 Safe Init) ===");

  // --- I/O Pin Setup ---
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

  // Limit switches — remember GPIO34/35 are INPUT-ONLY and need external pull-ups!
  pinMode(limitPin, INPUT);
  pinMode(raisedPin, INPUT);

  // Start with main stepper disabled
  digitalWrite(enablePin, HIGH);
  digitalWrite(dirPin, HIGH);

  // --- EEPROM ---
  EEPROM.begin(512);

  // --- I2C (PCA9685 + ADS1115 detection) ---
  Wire.begin(21, 22);      // SDA, SCL
  Wire.setClock(400000);   // 400kHz fast mode

  // --- ADS1115 (0x48 default) ---
  if (i2cPresent(0x48) && ads.begin(0x48)) {
    ads.setGain(GAIN_ONE);
    g_hasADS = true;
    Serial.println("ADS1115 detected @0x48");
  } else {
    g_hasADS = false;
    Serial.println("WARN: ADS1115 not found; pot readings disabled.");
  }

  // --- PCA9685 (0x40 default) ---
  if (i2cPresent(0x40)) {
    pwm.begin();
    pwm.setPWMFreq(60);
    g_hasPCA = true;
    Serial.println("PCA9685 detected @0x40");
  } else {
    g_hasPCA = false;
    Serial.println("WARN: PCA9685 not found; servo control disabled.");
  }

  // --- Bases ---
  initBasesNoMCP();
  disableAllBases();

  // --- Potentiometers ---
  if (g_hasADS) {
    initPots();
  } else {
    // Initialize arrays to 0 for consistency
    for (uint8_t i = 0; i < NUM_POTS; ++i) {
      pot_raw[i] = pot_filtered[i] = pot_last_reported[i] = 0;
    }
    pots_inited = true;
  }

  // --- Toolhead safety ---
  if (g_hasPCA) {
    delay(100);
    if (!ensureToolheadRaised()) {
      Serial.println("ERROR: Toolhead not raised. System halted.");
      while (true) delay(1000);
    }
  } else {
    Serial.println("WARN: No PCA9685 -> cannot raise toolhead. Movement blocked.");
  }

  Serial.println("Setup complete. Type commands: on/off, home, move, servo, etc.");
}


// ================= SERIAL COMMANDS (unchanged) =================


void handleSerial() {
  static String input = "";
  while (Serial.available()) {
    char c = Serial.read(); 
    if (c == '\n') {
      input.trim();
      Serial.print("received: ");
      Serial.println(input);
      if (input == "on") {
        digitalWrite(enablePin, LOW);
        motorEnabled = true;
        Serial.println("Motor ON");
      } else if (input == "off") {
        motorEnabled = false;
        digitalWrite(enablePin, HIGH);
        Serial.println("Motor OFF");
      } else if (input.startsWith("speed ")) {
        int sps = input.substring(6).toInt();  // steps per second
        if (sps > 0) {
          stepInterval = 1000000UL / sps;
          Serial.print("Speed set to ");
          Serial.print(sps);
          Serial.println(" steps/sec");
        }
      } else if (input.startsWith("dir ")) {
        int d = input.substring(4).toInt();
        digitalWrite(dirPin, d ? HIGH : LOW);
        Serial.print("Direction set to ");
        Serial.println(d);
      } else if (input == "home") {
        Serial.println("Homing...");
        HomeAxis();
      } else if (input == "pos") {
        float mm = currentPositionSteps / STEPS_PER_MM;
        Serial.print("POS steps=");
        Serial.print(currentPositionSteps);
        Serial.print("  mm=");
        Serial.println(mm, 3);
      } else if (input.startsWith("move ")) {  // relative, in steps
        long steps = input.substring(5).toInt();
        digitalWrite(enablePin, ENABLE_LEVEL);
        moveSteps(steps);
        digitalWrite(enablePin, DISABLE_LEVEL);
        Serial.println("OK");
      } else if (input.startsWith("movemm ")) {  // relative, in mm
        float mm = input.substring(7).toFloat();
        long steps = (long)lround(mm * STEPS_PER_MM);
        digitalWrite(enablePin, ENABLE_LEVEL);
        moveSteps(steps);
        digitalWrite(enablePin, DISABLE_LEVEL);
        Serial.println("OK");
      } else if (input.startsWith("goto ")) {  // absolute, steps
        long tgt = input.substring(5).toInt();
#ifdef CHECK_SOFT_LIMITS
        if (tgt < MIN_POS_STEPS) tgt = MIN_POS_STEPS;
        if (tgt > MAX_POS_STEPS) tgt = MAX_POS_STEPS;
#endif
        digitalWrite(enablePin, ENABLE_LEVEL);
        moveToSteps(tgt);
        digitalWrite(enablePin, DISABLE_LEVEL);
        Serial.println("OK");
      } else if (input.startsWith("gotomm ")) {  // absolute, mm
        float mm = input.substring(7).toFloat();
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

      else if (input.startsWith("base ")) {
        int idx = input.substring(5).toInt(); // 0..5
        if (!selectBaseStepper((uint8_t)idx)) Serial.println("Usage: base <0..5>");
      }


      else if (input.startsWith("gobase ")) {
        // gobase <idx 1..NUM_BASES>
        // Example: gobase 3
        long idx = input.substring(7).toInt();  // after "gobase "
        if (idx < 1 || idx > NUM_BASES) {
          Serial.print("Usage: gobase <1..");
          Serial.print(NUM_BASES);
          Serial.println(">");
        } else {
          if (!goToBase((uint8_t)idx)) {
            Serial.println("ERROR: failed to move to base (see above).");
          }
        }
      }


      else if (input.startsWith("servopulse")) {
        int servoNum = input.substring(10, 11).toInt();  // e.g. "SERVOPULSE 0 1500"
        int pulseLen = input.substring(12).toInt();
        setServoPulseRaw(servoNum, pulseLen);
        Serial.print("Servo ");
        Serial.print(servoNum);
        Serial.print(" set to raw pulse length ");
        Serial.println(pulseLen);
      }


      else if (input.startsWith("servo ")) {
        // Expecting: "servo <channel> <angle>"
        int firstSpace = input.indexOf(' ', 6);
        if (firstSpace > 0) {
          int channel = input.substring(6, firstSpace).toInt();
          int angle = input.substring(firstSpace + 1).toInt();

          if (channel < 0 || channel > 15) {
            Serial.println("Error: channel must be 0–15");
          } else {
            setServoAngle(channel, angle);
          }
        } else {
          Serial.println("Usage: servo <channel 0-15> <angle 0-180>");
        }
      } else if (input.startsWith("raise")) {
        raiseToolhead();
      } else if (input.startsWith("servoslow ")) {
        // Usage: servoslow <channel> <angle> [delay_ms]
        int firstSpace = input.indexOf(' ', 10);
        if (firstSpace > 0) {
          int channel = input.substring(10, firstSpace).toInt();
          String rest = input.substring(firstSpace + 1);
          int secondSpace = rest.indexOf(' ');
          int angle, delayMs;
          if (secondSpace > 0) {
            angle = rest.substring(0, secondSpace).toInt();
            delayMs = rest.substring(secondSpace + 1).toInt();
          } else {
            angle = rest.toInt();
            delayMs = 15;  // default
          }

          if (channel < 0 || channel > 15) {
            Serial.println("Error: channel must be 0–15");
          } else {
            setServoAngleSlow(channel, angle, delayMs);
          }
        } else {
          Serial.println("Usage: servoslow <channel> <angle 0-180> [delay_ms]");
        }
      }


      else if (input == "couple" || input == "couplesyringes") {
        coupleSyringes();
        Serial.println("OK coupleSyringes");
      }


      // Single-axis moves
      else if (input.startsWith("move2 ")) {
        long s = input.substring(6).toInt();
        moveSteps2(s);
        Serial.println("OK move2");
      } else if (input.startsWith("move3 ")) {
        long s = input.substring(6).toInt();
        moveSteps3(s);
        Serial.println("OK move3");
      }

      // Set shared speed for simultaneous motion
      else if (input.startsWith("speed23 ")) {
        long sps = input.substring(8).toInt();
        setSpeed23SPS(sps);
      }

      // Simultaneous move with arbitrary step counts
      // Example: m23 1600 1600  (both same way)
      //          m23 1600 -1600 (opposite directions)
      else if (input.startsWith("m23 ")) {
        int sp = input.indexOf(' ', 4);
        if (sp > 0) {
          long s2 = input.substring(4, sp).toInt();
          long s3 = input.substring(sp + 1).toInt();
          moveStepsSync23(s2, s3);
          Serial.println("OK m23");
        } else {
          Serial.println("Usage: m23 <steps2> <steps3>");
        }
      }

      // Convenience: LINK = equal & opposite steps (paint transfer)
      // Example: link 2000  -> #2 +2000, #3 -2000
      else if (input.startsWith("link ")) {
        long s = input.substring(5).toInt();
        moveStepsSync23(s, -s);
        Serial.println("OK link");
      }

      // Position readouts
      else if (input == "pos2") {
        float mm = currentPositionSteps2 / STEPS_PER_MM;
        Serial.print("POS2 steps=");
        Serial.print(currentPositionSteps2);
        Serial.print("  mm=");
        Serial.println(mm, 3);
      } else if (input == "pos3") {
        float mm = currentPositionSteps3 / STEPS_PER_MM;
        Serial.print("POS3 steps=");
        Serial.print(currentPositionSteps3);
        Serial.print("  mm=");
        Serial.println(mm, 3);
      }

      else if (input == "pots" || input == "potlist") {
      for (uint8_t i = 0; i < NUM_POTS; ++i) {
        Serial.print("pot["); Serial.print(i); Serial.print("] raw=");
        Serial.print(getPotRaw(i));
        Serial.print(" filt=");
        Serial.print(getPotFiltered(i));
        Serial.print(" pct=");
        Serial.println(getPotPercent(i), 1);
      }
    }

      else if (input.startsWith("potmove ")) {
      // Usage: potmove <target_adc 0-1023> <sps>
      int sp = input.indexOf(' ', 8);
      if (sp < 0) {
        Serial.println("Usage: potmove <target_adc 0-1023> <sps>");
      } else {
        uint16_t target = (uint16_t)input.substring(8, sp).toInt();
        long sps = input.substring(sp + 1).toInt();
        bool ok = move2UntilPot_simple(target, sps);
        if (ok) Serial.println("OK potmove");
        else    Serial.println("ERR potmove");
      }
    }



      else {
        Serial.println("Invalid command.");
      }

      input = "";
    } else if (c != '\r') {
      input += c;
    }
  }
}


void loop() {
  handleSerial();

  if (motorEnabled) {
    unsigned long now = micros();
    if (now - lastStepTime >= stepInterval) {
      lastStepTime = now;
      digitalWrite(stepPin, HIGH);
      delayMicroseconds(2);
      digitalWrite(stepPin, LOW);
    }
  }
}