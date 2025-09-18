#define dirPin 2
#define stepPin 3
#define enablePin 8

#define STEP2_PIN   4
#define DIR2_PIN    5
#define EN2_PIN     6

#define STEP3_PIN   7
#define DIR3_PIN    10
#define EN3_PIN     11

// ---- Limit switch wiring (RAMPS 1.4 endstop) ----
#define limitPin 9            // green 'S' wire from the endstop
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

// Shared speed for simultaneous moves of 2 & 3 (steps/sec)
static long speed23_sps = 800;             // default
static unsigned long interval23_us = 1000000UL / (2UL * 800UL); // µs between toggles

bool homed = false;

long lastStepTime = 0;
unsigned long stepInterval = 1000;  // microseconds between steps (500 steps/sec)
bool motorEnabled = false;
bool stepState = false;

#include <Adafruit_PWMServoDriver.h>

// --- PCA9685 servo driver ---
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

// Servo calibration (these are typical; adjust to your servos)
#define SERVO_MIN  150  // pulse length out of 4096 for ~0°
#define SERVO_MAX  600  // pulse length out of 4096 for ~180°



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




// Move a signed number of steps at the current stepInterval (blocking).
// Positive 'steps' moves with DIR=HIGH, negative with DIR=LOW.
// Updates the global currentPositionSteps on EACH full step edge-pair.
static void moveSteps(long steps) {
  if (steps == 0) return;

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

// One timed toggle on a given step pin (uses its own timer)
static inline void stepOnceTimedOnPin(uint8_t stepPin, unsigned long interval_us) {
  static unsigned long last2 = 0, last3 = 0;
  static bool state2 = false, state3 = false;

  unsigned long &last = (stepPin == STEP2_PIN) ? last2 : last3;
  bool &state = (stepPin == STEP2_PIN) ? state2 : state3;

  unsigned long now;
  do { now = micros(); } while ((unsigned long)(now - last) < interval_us);
  last = now;

  state = !state;
  digitalWrite(stepPin, state ? HIGH : LOW);
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

void HomeAxis() {
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


// Function to set servo pulse directly in microseconds
void setServoPulseRaw(uint8_t servoNum, int pulseLength) {
  // pulseLength is in microseconds, e.g. 1500 = neutral
  pwm.writeMicroseconds(servoNum, pulseLength);
}

void setServoAngle(uint8_t channel, int angle) {
  if (angle < 0) angle = 0;
  if (angle > 180) angle = 180;

  // map angle to PCA9685 pulse
  uint16_t pulselen = map(angle, 0, 180, SERVO_MIN, SERVO_MAX);

  pwm.setPWM(channel, 0, pulselen);

  Serial.print("Servo ");
  Serial.print(channel);
  Serial.print(" -> ");
  Serial.print(angle);
  Serial.println(" deg");
}

// Slowly sweep servo from its current position to target
void setServoAngleSlow(uint8_t channel, int targetAngle, int stepDelay = 23) {
  static int currentAngles[16] = {90}; // store last commanded angle per channel (default 90°)
  
  if (targetAngle < 0) targetAngle = 0;
  if (targetAngle > 180) targetAngle = 180;

  int startAngle = currentAngles[channel];
  int step = (targetAngle > startAngle) ? 1 : -1;

  for (int angle = startAngle; angle != targetAngle; angle += step) {
    uint16_t pulselen = map(angle, 0, 180, SERVO_MIN, SERVO_MAX);
    pwm.setPWM(channel, 0, pulselen);
    delay(stepDelay); // ms between increments
  }

  // Ensure exact target at the end
  uint16_t pulselen = map(targetAngle, 0, 180, SERVO_MIN, SERVO_MAX);
  pwm.setPWM(channel, 0, pulselen);

  currentAngles[channel] = targetAngle;

  Serial.print("Servo ");
  Serial.print(channel);
  Serial.print(" slowly moved to ");
  Serial.print(targetAngle);
  Serial.println(" deg");
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
        int sps = input.substring(6).toInt(); // steps per second
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
      } 
      else if (input == "pos") {
  float mm = currentPositionSteps / STEPS_PER_MM;
  Serial.print("POS steps=");
  Serial.print(currentPositionSteps);
  Serial.print("  mm=");
  Serial.println(mm, 3);
}
else if (input.startsWith("move ")) {  // relative, in steps
  long steps = input.substring(5).toInt();
  digitalWrite(enablePin, ENABLE_LEVEL);
  moveSteps(steps);
  digitalWrite(enablePin, DISABLE_LEVEL);
  Serial.println("OK");
}
else if (input.startsWith("movemm ")) { // relative, in mm
  float mm = input.substring(7).toFloat();
  long steps = (long)lround(mm * STEPS_PER_MM);
  digitalWrite(enablePin, ENABLE_LEVEL);
  moveSteps(steps);
  digitalWrite(enablePin, DISABLE_LEVEL);
  Serial.println("OK");
}
else if (input.startsWith("goto ")) { // absolute, steps
  long tgt = input.substring(5).toInt();
#ifdef CHECK_SOFT_LIMITS
  if (tgt < MIN_POS_STEPS) tgt = MIN_POS_STEPS;
  if (tgt > MAX_POS_STEPS) tgt = MAX_POS_STEPS;
#endif
  digitalWrite(enablePin, ENABLE_LEVEL);
  moveToSteps(tgt);
  digitalWrite(enablePin, DISABLE_LEVEL);
  Serial.println("OK");
}
else if (input.startsWith("gotomm ")) { // absolute, mm
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
    int angle   = input.substring(firstSpace + 1).toInt();

    if (channel < 0 || channel > 15) {
      Serial.println("Error: channel must be 0–15");
    } else {
      setServoAngle(channel, angle);
    }
  } else {
    Serial.println("Usage: servo <channel 0-15> <angle 0-180>");
  }
}

else if (input.startsWith("servoslow ")) {
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
      delayMs = 15; // default
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

      
      else {
        Serial.println("Invalid command.");
      }
      input = "";
    } else if (c != '\r') {
      input += c;
    }
  }
}
