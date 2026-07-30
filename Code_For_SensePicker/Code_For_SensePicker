#include <Servo.h>
#include <IRremote.hpp>


// 1. PIN DEFINITIONS & REMOTE HEX CODES

#define IR_RECEIVE_PIN 6   // IR Receiver Pin
#define TRIG_PIN 11        // Trigger PIN
#define RC_LED_PIN 12      // RC Mode Indicator LED
#define IR_BLINK_PIN 13    // IR Feedback LED (Onboard "L" LED)

#define CENTER_LED_PIN A0  // Sensor detect LED for Center (A2)
#define SIDE_LED_PIN A1    // Sensor detect LED for Left (A3) or Right (A5)

// Small Stepper Pins (28BYJ-48)
#define IN1 2
#define IN2 3
#define IN3 4
#define IN4 5

// Main Stepper Pins (DRV8825 + NEMA 17)
#define STEP_PIN 8
#define DIR_PIN 9
#define ENABLE_PIN 10

// IR Remote Button Codes
#define UP_CODE 0xF609FF00         // Remote UP Button (Forward Drive)
#define DOWN_CODE 0xF807FF00       // Remote DOWN Button (Reverse Drive)
#define RIGHT_CODE 0xBC43FF00      // Servo Steer Right
#define LEFT_CODE 0xBB44FF00       // Servo Steer Left
#define VOL_PLUS_CODE 0xB946FF00   // Speed Up
#define VOL_MINUS_CODE 0xEA15FF00  // Slow Down
#define FUNC_STOP_CODE 0xB847FF00  // Trigger Forklift Assist
#define POWER_CODE 0xBA45FF00      // System Power ON/OFF
#define EQ_CODE 0xE619FF00         // Small Stepper +30 deg (CW)
#define ST_REPT_CODE 0xF20DFF00    // Small Stepper -30 deg (CCW)
#define RC_MODE 0xBF40FF00         // RC Auto Avoidance Toggle


// 2. CONSTANTS & CONFIGURATION VALUES

#define DIR_FORWARD LOW   // LOW physically spins motor FORWARD
#define DIR_REVERSE HIGH  // HIGH physically spins motor BACKWARD

#define servostarting 97
#define servodegree 20
#define ASSIST_STEER_DEGREE 45     // 45 deg assist steering angle
#define DEBOUNCE_MS 60

const int THIRTY_DEG_STEPS = 342;
const int THREE_HUNDRED_DEG_STEPS = 3414;

const float WHEELBASE_MM = 180.0;
const float WHEEL_CIRCUMFERENCE_MM = 267.035;  // PI * 85mm

// Speed tiers: 600us (Fast), 1200us, 2400us, 3600us, 4800us (Super Slow)
const uint32_t speedLevels[5] = { 600, 1200, 2400, 3600, 4800 };
const uint32_t FORKLIFT_SLOW_SPEED = 2400;  // 2400us step delay for assist movements
const uint32_t smallStepIntervalMicros = 1500;

const int echoPins[4] = { A2, A3, A4, A5 };  // 0:Center, 1:Left, 2:Top, 3:Right
const char* sensorNames[4] = { "A2 (Center)", "A3 (Left)", "A4 (Top)", "A5 (Right)" };
const uint32_t sensorIntervalMs = 60;
const uint32_t rcmodecheckinterval = 50;     // 50ms crosstalk dissipation delay

const bool stepSequence[8][4] = {
  { 1, 0, 0, 0 },
  { 1, 1, 0, 0 },
  { 0, 1, 0, 0 },
  { 0, 1, 1, 0 },
  { 0, 0, 1, 0 },
  { 0, 0, 1, 1 },
  { 0, 0, 0, 1 },
  { 1, 0, 0, 1 }
};


// 3. GLOBAL VARIABLES & STATE TRACKERS

Servo servo;

bool systemPower = true;
bool rcmode = false;
int stepState = 0;  // 1 = Forward, -1 = Reverse, 0 = Stop

int speedIndex = 3;                   // Boot default: Index 3 (3600 us)
uint32_t stepIntervalMicros = 3600;   // Boot default: 3600 us
uint32_t lastStepTime = 0;

int smallStepIndex = 0;
int smallStepTarget = 0;
uint32_t lastSmallStepMicros = 0;

int currentSensor = 0;
uint32_t lastSensorCheckMs = 0;
float sensorDistances[4] = { 0.0, 0.0, 0.0, 0.0 };

bool servoAtLeft = false;
bool servoAtRight = false;

uint32_t lastIRTime = 0;
uint32_t lastCode = 0;

uint32_t irBlinkStartTime = 0;
uint32_t autoSteerResetTime = 0;
uint32_t lastrcmodecheck = 0;

int rcStateIndex = 0; // roundrobin index for RC mode (0: Right, 1: Left, 2: Center)


// 4. HELPER FUNCTIONS


// Pulse Driver Step Pin
void pulseStep() {
  digitalWrite(STEP_PIN, HIGH);
  delayMicroseconds(5);
  digitalWrite(STEP_PIN, LOW);
}

// Steering Backlash Compensation (+25 deg overshoot on right side return)
void straightenServo(bool cameFromLeft) {
  if (cameFromLeft) {
    servo.write(servostarting + 25);
  } else {
    servo.write(servostarting - 15);
  }
  delay(120);
  servo.write(servostarting);        // Settle dead center at 97 deg
  delay(50);
}

// Power Abort Override Check
bool checkpowerOverride() {
  if (IrReceiver.decode()) {
    uint32_t code = IrReceiver.decodedIRData.decodedRawData;
    if (code == POWER_CODE) {
      systemPower = false;
      stepState = 0;
      digitalWrite(ENABLE_PIN, HIGH);
      digitalWrite(IN1, LOW);
      digitalWrite(IN2, LOW);
      digitalWrite(IN3, LOW);
      digitalWrite(IN4, LOW);
      digitalWrite(RC_LED_PIN, LOW);
      digitalWrite(IR_BLINK_PIN, LOW);
      digitalWrite(CENTER_LED_PIN, LOW);
      digitalWrite(SIDE_LED_PIN, LOW);
      Serial.println(F("\n>>> POWER OVERRIDE TRIGGERED ABORTING ALL OPERATIONS <<<"));
      IrReceiver.resume();
      return true;
    }
  }
  return !systemPower;
}

// Non Blocking Manual Control for Small Stepper
void runSmallStepperManual() {
  if (!systemPower || smallStepTarget == 0) return;

  uint32_t currentMicros = micros();
  if (currentMicros - lastSmallStepMicros >= smallStepIntervalMicros) {
    lastSmallStepMicros = currentMicros;

    digitalWrite(IN1, stepSequence[smallStepIndex][0]);
    digitalWrite(IN2, stepSequence[smallStepIndex][1]);
    digitalWrite(IN3, stepSequence[smallStepIndex][2]);
    digitalWrite(IN4, stepSequence[smallStepIndex][3]);

    if (smallStepTarget > 0) {
      smallStepIndex++;
      smallStepTarget--;
      if (smallStepIndex > 7) smallStepIndex = 0;
    } else if (smallStepTarget < 0) {
      smallStepIndex--;
      smallStepTarget++;
      if (smallStepIndex < 0) smallStepIndex = 7;
    }

    if (smallStepTarget == 0) {
      digitalWrite(IN1, LOW);
      digitalWrite(IN2, LOW);
      digitalWrite(IN3, LOW);
      digitalWrite(IN4, LOW);
    }
  }
}

// Fast singleping Reader (8000us timeout = 1.3m reach)
float quickReadSensorMm(int index, float maxCapMm = 1000.0) {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  long dur = pulseIn(echoPins[index], HIGH, 8000);
  if (dur > 0) {
    float dist = (dur * 0.343) / 2.0;
    if (dist >= 10.0 && dist <= maxCapMm) {
      return dist;
    }
  }
  return 0.0;
}

// 3 ping Median Filter Ultrasonic Sensor Helper
float readSensorMm(int index, float maxCapMm = 1000.0) {
  float reads[3];
  int validCount = 0;

  for (int i = 0; i < 3; i++) {
    digitalWrite(TRIG_PIN, LOW);
    delayMicroseconds(2);
    digitalWrite(TRIG_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIG_PIN, LOW);

    long dur = pulseIn(echoPins[index], HIGH, 8000);
    if (dur > 0) {
      float dist = (dur * 0.343) / 2.0;
      if (dist >= 10.0 && dist <= maxCapMm) {
        reads[validCount++] = dist;
      }
    }
    delay(4);
  }

  if (validCount == 0) return 0.0;
  if (validCount == 1) return reads[0];
  if (validCount == 2) return (reads[0] + reads[1]) / 2.0;

  if (reads[0] > reads[1]) { float t = reads[0]; reads[0] = reads[1]; reads[1] = t; }
  if (reads[1] > reads[2]) { float t = reads[1]; reads[1] = reads[2]; reads[2] = t; }
  if (reads[0] > reads[1]) { float t = reads[0]; reads[0] = reads[1]; reads[1] = t; }
  return reads[1];
}

// Background Ultrasonic Sensor Scanner (Manual Mode)
void checkUltrasonicSensors() {
  if (!systemPower || rcmode) return;

  uint32_t currentMillis = millis();
  if (currentMillis - lastSensorCheckMs >= sensorIntervalMs) {
    lastSensorCheckMs = currentMillis;

    float distMm = quickReadSensorMm(currentSensor, 1000.0);
    sensorDistances[currentSensor] = (distMm > 0.0) ? distMm : 0.0;

    currentSensor++;
    if (currentSensor > 3) {
      currentSensor = 0;

      Serial.print(F("[SENSORS mm] Center: "));
      Serial.print(sensorDistances[0], 1);
      Serial.print(F(" | Left: "));
      Serial.print(sensorDistances[1], 1);
      Serial.print(F(" | Top: "));
      Serial.print(sensorDistances[2], 1);
      Serial.print(F(" | Right: "));
      Serial.println(sensorDistances[3], 1);
    }

    bool centerOn = (sensorDistances[0] > 0.0 && sensorDistances[0] <= 250.0);
    bool sideOn   = ((sensorDistances[1] > 0.0 && sensorDistances[1] <= 125.0) || 
                     (sensorDistances[3] > 0.0 && sensorDistances[3] <= 125.0));

    digitalWrite(CENTER_LED_PIN, centerOn ? HIGH : LOW);
    digitalWrite(SIDE_LED_PIN, sideOn ? HIGH : LOW);
  }
}

// RC Mode Auto Avoidance
void rcmodecheck() {
  if (checkpowerOverride() || !rcmode) return;

  uint32_t currentMillis = millis();
  if ((currentMillis - lastrcmodecheck) >= rcmodecheckinterval) {
    lastrcmodecheck = currentMillis;

    // Round robin sensor scanning (1 sensor per pass)
    if (rcStateIndex == 0) {
      sensorDistances[3] = quickReadSensorMm(3, 1000.0); // Right
      rcStateIndex = 1;
    } else if (rcStateIndex == 1) {
      sensorDistances[1] = quickReadSensorMm(1, 1000.0); // Left
      rcStateIndex = 2;
    } else {
      sensorDistances[0] = quickReadSensorMm(0, 1000.0); // Center
      rcStateIndex = 0;
    }

    float distanceright  = sensorDistances[3];
    float distanceleft   = sensorDistances[1];
    float distancecenter = sensorDistances[0];

    // Drive Indicator LEDs (250mm center, 350mm side)
    bool centerOn = (distancecenter > 0.0 && distancecenter <= 250.0);
    bool sideOn   = ((distanceleft > 0.0 && distanceleft <= 350.0) || 
                     (distanceright > 0.0 && distanceright <= 350.0));
    digitalWrite(CENTER_LED_PIN, centerOn ? HIGH : LOW);
    digitalWrite(SIDE_LED_PIN, sideOn ? HIGH : LOW);

    // 1. Auto Brake on Center Obstacle (< 250mm)
    if (distancecenter > 0.0 && distancecenter < 250.0) {
      digitalWrite(ENABLE_PIN, HIGH);
      stepState = 0;
      servo.write(servostarting);
      Serial.println(F("[RC MODE AUTO BRAKE] Center obstacle detected!"));
    }
    // 2. Early Side Nudges (< 350mm)
    else if (distanceright > 0.0 && distanceright < 350.0) {
      Serial.println(F("[RC MODE AUTO STEER] Nudging Left"));
      servo.write(servostarting - servodegree);
      autoSteerResetTime = currentMillis + 400;
    } else if (distanceleft > 0.0 && distanceleft < 350.0) {
      Serial.println(F("[RC MODE AUTO STEER] Nudging Right"));
      servo.write(servostarting + servodegree);
      autoSteerResetTime = currentMillis + 400;
    }
    // 3. Danger Zone Safety Reverse (< 50mm)
    else if ((distanceleft <= 50.0 && distanceleft > 0.0) || 
             (distancecenter <= 50.0 && distancecenter > 0.0) || 
             (distanceright <= 50.0 && distanceright > 0.0)) {
      Serial.println(F("[RC MODE SAFETY REVERSE] Reversing away..."));
      digitalWrite(ENABLE_PIN, LOW);
      digitalWrite(DIR_PIN, DIR_REVERSE);
      delay(10);

      for (int i = 0; i < 350; i++) {
        pulseStep();
        delayMicroseconds(FORKLIFT_SLOW_SPEED);
      }
      stepState = 0;
      digitalWrite(ENABLE_PIN, HIGH);
    }
  }
}

// Forklift Assist: Final Alignment & Dynamic Payload Grab
void onceLinedUpDriveAndGrab(float dCenter) {
  if (checkpowerOverride()) return;

  smallStepTarget = 0;

  // Take a fresh precise reading from center sensor up to 400mm
  float preciseCenter = readSensorMm(0, 400.0);
  if (preciseCenter > 0.0) {
    dCenter = preciseCenter;
  }

  Serial.print(F("[CENTER TARGET CONFIRMED] Distance: "));
  Serial.print(dCenter, 1);
  Serial.println(F(" mm."));

  float targetBuffer = 55.0; // Stop when 55mm away from payload
  float currentDist = dCenter;

  // Drive forward if farther than 55mm
  if (currentDist > targetBuffer) {
    digitalWrite(ENABLE_PIN, LOW);
    digitalWrite(DIR_PIN, DIR_FORWARD); // Sets LOW = drives forward
    delay(10);

    Serial.print(F("[DRIVING TOWARDS PAYLOAD] Current distance: "));
    Serial.print(currentDist, 1);
    Serial.println(F(" mm..."));

    int stepCounter = 0;

    // Dynamic closed loop approach toward payload
    while (currentDist > targetBuffer && stepCounter < 800) {
      if (checkpowerOverride()) return;

      // Sample center sensor every 17 steps (3567us average delay / 0.37 m/s speed)
      if (stepCounter % 17 == 0) {
        float threepingmedian = readSensorMm(0, 400.0);
        if (threepingmedian > 0.0) {
          currentDist = threepingmedian; // Live updates distance as car moves closer
        }
      }

      pulseStep();
      delayMicroseconds(FORKLIFT_SLOW_SPEED);
      stepCounter++;
    }

    stepState = 0;
    digitalWrite(ENABLE_PIN, HIGH);
    Serial.println(F("[DRIVE COMPLETE] Arrived at payload threshold."));
  } else {
    Serial.println(F("[POSITION OK] Already at target threshold."));
  }

  // Execute Uninterrupted Claw Grab
  if (systemPower) {
    Serial.println(F("[CLAW] Rotating front stepper 300 degrees clockwise to GRAB..."));

    for (int s = 0; s < THREE_HUNDRED_DEG_STEPS; s++) {
      if (checkpowerOverride()) {
        stepState = 0;
        digitalWrite(ENABLE_PIN, HIGH);
        return;
      }

      digitalWrite(IN1, stepSequence[smallStepIndex][0]);
      digitalWrite(IN2, stepSequence[smallStepIndex][1]);
      digitalWrite(IN3, stepSequence[smallStepIndex][2]);
      digitalWrite(IN4, stepSequence[smallStepIndex][3]);

      smallStepIndex++;
      if (smallStepIndex > 7) smallStepIndex = 0;
      delayMicroseconds(smallStepIntervalMicros);
    }

    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, LOW);

    smallStepTarget = 0;

    Serial.println(F("=== FORKLIFT ASSIST COMPLETE: PAYLOAD SECURED ==="));
  }

  stepState = 0;
  digitalWrite(ENABLE_PIN, HIGH);
}

// Forklift Assist: Search & Align Decision Engine
void executeForkliftAssist() {
  Serial.println(F("\n=== FORKLIFT ASSIST INITIATED ==="));

  if (checkpowerOverride()) return;

  float distTop = readSensorMm(2, 90.0);
  if (distTop > 0.0) {
    Serial.print(F("[ABORT] Low ceiling detected: "));
    Serial.print(distTop, 1);
    Serial.println(F(" mm"));
    return;
  }

  float dCenter = readSensorMm(0, 400.0);
  float dLeft   = readSensorMm(1, 120.0);
  float dRight  = readSensorMm(3, 120.0);

  // PATHWAY A: Target straight ahead
  if (dCenter > 0.0) {
    Serial.println(F("[PATHWAY A] Target detected in front of Center Sensor!"));
    onceLinedUpDriveAndGrab(dCenter);
    return;
  }

  // PATHWAY B1: Object on Right Sensor = Sweep Left at 45 deg & reverse arc
  else if (dRight > 0.0 && dCenter == 0.0) {
    Serial.println(F("[PATHWAY B1] Object on Right! Steering left 45 deg & reversing arc..."));
    servo.write(servostarting - ASSIST_STEER_DEGREE);
    delay(150);

    digitalWrite(ENABLE_PIN, LOW);
    digitalWrite(DIR_PIN, DIR_REVERSE);
    delay(10);

    int reverseStepCount = 0;
    const int MAX_REVERSE_STEPS = 340; // 45cm reverse arc limit
    bool targetAcquired = false;
    float foundCenterDist = 0.0;

    while (reverseStepCount < MAX_REVERSE_STEPS) {
      if (checkpowerOverride()) return;

      pulseStep();
      delayMicroseconds(FORKLIFT_SLOW_SPEED);
      reverseStepCount++;

      // Force initial arc travel (step 110) before testing center sensor lock
      if (reverseStepCount >= 110 && reverseStepCount % 8 == 0) {
        float cCheck = quickReadSensorMm(0, 400.0);
        if (cCheck > 0.0) {
          Serial.println(F("[TARGET ACQUIRED] Center sensor aligned during arc!"));
          targetAcquired = true;
          foundCenterDist = cCheck;
          break; // Exit arc as soon as center locks on
        }
      }
    }

    straightenServo(true);
    digitalWrite(ENABLE_PIN, HIGH);

    // Re verify center sensor after wheels straighten
    delay(120);
    float finalCenterCheck = readSensorMm(0, 400.0);
    if (finalCenterCheck > 0.0) {
      foundCenterDist = finalCenterCheck;
      targetAcquired = true;
    } else if (!targetAcquired && dRight > 0.0) { // this can cause false positives
      foundCenterDist = dRight + 40.0;
      targetAcquired = true;
    }

    if (targetAcquired && foundCenterDist > 0.0) {
      onceLinedUpDriveAndGrab(foundCenterDist);
    }
    return;
  }

  // PATHWAY B2: Object on Left Sensor = Sweep Right at 45 deg & reverse arc
  else if (dLeft > 0.0 && dCenter == 0.0) {
    Serial.println(F("[PATHWAY B2] Object on Left! Steering right 45 deg & reversing arc..."));
    servo.write(servostarting + ASSIST_STEER_DEGREE);
    delay(150);

    digitalWrite(ENABLE_PIN, LOW);
    digitalWrite(DIR_PIN, DIR_REVERSE);
    delay(10);

    int reverseStepCount = 0;
    const int MAX_REVERSE_STEPS = 320; // ~42cm reverse arc limit
    bool targetAcquired = false;
    float foundCenterDist = 0.0;

    while (reverseStepCount < MAX_REVERSE_STEPS) {
      if (checkpowerOverride()) return;

      pulseStep();
      delayMicroseconds(FORKLIFT_SLOW_SPEED);
      reverseStepCount++;

      // Force initial arc travel (step 110) before testing center sensor lock
      if (reverseStepCount >= 110 && reverseStepCount % 8 == 0) {
        float cCheck = quickReadSensorMm(0, 400.0);
        if (cCheck > 0.0) {
          Serial.println(F("[TARGET ACQUIRED] Center sensor aligned during arc!"));
          targetAcquired = true;
          foundCenterDist = cCheck;
          break; // Exit arc as soon as center locks on
        }
      }
    }

    straightenServo(false);
    digitalWrite(ENABLE_PIN, HIGH);

    // Re verify center sensor after wheels straighten
    delay(120);
    float finalCenterCheck = readSensorMm(0, 400.0);
    if (finalCenterCheck > 0.0) {
      foundCenterDist = finalCenterCheck;
      targetAcquired = true;
    } else if (!targetAcquired && dLeft > 0.0) { // this can cause false positives
      foundCenterDist = dLeft + 40.0;
      targetAcquired = true;
    }

    if (targetAcquired && foundCenterDist > 0.0) {
      onceLinedUpDriveAndGrab(foundCenterDist);
    }
    return;
  }
}


// 5. SETUP FUNCTION

void setup() {
  Serial.begin(115200);

  IrReceiver.begin(IR_RECEIVE_PIN, DISABLE_LED_FEEDBACK);
  servo.attach(7);
  servo.write(servostarting);

  pinMode(RC_LED_PIN, OUTPUT);
  digitalWrite(RC_LED_PIN, LOW);

  pinMode(IR_BLINK_PIN, OUTPUT);
  digitalWrite(IR_BLINK_PIN, LOW);

  pinMode(CENTER_LED_PIN, OUTPUT);
  pinMode(SIDE_LED_PIN, OUTPUT);
  digitalWrite(CENTER_LED_PIN, LOW);
  digitalWrite(SIDE_LED_PIN, LOW);

  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  pinMode(STEP_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);
  pinMode(ENABLE_PIN, OUTPUT);

  digitalWrite(ENABLE_PIN, HIGH);  // Driver sleep mode on startup
  digitalWrite(STEP_PIN, LOW);
  digitalWrite(DIR_PIN, LOW);

  pinMode(TRIG_PIN, OUTPUT);
  digitalWrite(TRIG_PIN, LOW);

  for (int i = 0; i < 4; i++) {
    pinMode(echoPins[i], INPUT);
  }

  Serial.println(F("   FORKLIFT FIRMWARE ONLINE    "));
}


// 6. MAIN LOOP FUNCTION

void loop() {

  uint32_t now = millis();

  // 0. Non-Blocking Led & Servo Timers
  if (irBlinkStartTime > 0 && (now - irBlinkStartTime >= 50)) {
    digitalWrite(IR_BLINK_PIN, LOW);
    irBlinkStartTime = 0;
  }

  if (autoSteerResetTime > 0 && now >= autoSteerResetTime) {
    servo.write(servostarting);
    autoSteerResetTime = 0;
  }

  // 1. RC Auto-Avoidance Engine
  rcmodecheck();

  // 2. Small Stepper Manual Movement Engine
  runSmallStepperManual();

  // 3. Background Ultrasonic Sensor Scanner
  checkUltrasonicSensors();

  // 4. IR Remote Receiver & Parser
  if (IrReceiver.decode()) {
    uint32_t code = IrReceiver.decodedIRData.decodedRawData;

    if (IrReceiver.decodedIRData.protocol != UNKNOWN && code != 0 && code != 0xFFFFFFFF) {

      if (code != lastCode || (now - lastIRTime) > DEBOUNCE_MS) {
        lastCode = code;
        lastIRTime = now;

        digitalWrite(IR_BLINK_PIN, HIGH);
        irBlinkStartTime = now;

        Serial.print(F("\n[IR INPUT] Code: 0x"));
        Serial.println(code, HEX);

        if (code == POWER_CODE) {
          systemPower = !systemPower;
          if (!systemPower) {
            stepState = 0;
            digitalWrite(ENABLE_PIN, HIGH);
            digitalWrite(IN1, LOW);
            digitalWrite(IN2, LOW);
            digitalWrite(IN3, LOW);
            digitalWrite(IN4, LOW);
            digitalWrite(RC_LED_PIN, LOW);
            digitalWrite(IR_BLINK_PIN, LOW);
            digitalWrite(CENTER_LED_PIN, LOW);
            digitalWrite(SIDE_LED_PIN, LOW);
            Serial.println(F(">>> SYSTEM POWERED OFF (SLEEP MODE) <<<"));
          } else {
            Serial.println(F(">>> SYSTEM POWERED ON <<<"));
          }
        }

        if (systemPower) {

          if (code == VOL_PLUS_CODE) {
            if (speedIndex > 0) speedIndex--;
            stepIntervalMicros = speedLevels[speedIndex];
            Serial.print(F("[SPEED UP] NEMA 17 Interval: "));
            Serial.print(stepIntervalMicros);
            Serial.println(F(" us"));
          } else if (code == VOL_MINUS_CODE) {
            if (speedIndex < 4) speedIndex++;
            stepIntervalMicros = speedLevels[speedIndex];
            Serial.print(F("[SLOW DOWN] NEMA 17 Interval: "));
            Serial.print(stepIntervalMicros);
            Serial.println(F(" us"));
          } else if (code == EQ_CODE) {
            smallStepTarget += THIRTY_DEG_STEPS;
            Serial.println(F("[SMALL STEPPER] Moving +30 deg (CW)"));
          } else if (code == ST_REPT_CODE) {
            smallStepTarget -= THIRTY_DEG_STEPS;
            Serial.println(F("[SMALL STEPPER] Moving -30 deg (CCW)"));
          } else if (code == FUNC_STOP_CODE) {
            executeForkliftAssist();
          } else if (code == UP_CODE) {
            stepState = (stepState == 1) ? 0 : 1;
            Serial.println((stepState == 1) ? F("[DRIVE] FORWARD Enabled") : F("[DRIVE] STOPPED"));
          } else if (code == DOWN_CODE) {
            stepState = (stepState == -1) ? 0 : -1;
            Serial.println((stepState == -1) ? F("[DRIVE] REVERSE Enabled") : F("[DRIVE] STOPPED"));
          } else if (code == LEFT_CODE) {
            if (servoAtLeft) {
              servo.write(servostarting);
              servoAtLeft = false;
            } else {
              servo.write(servostarting - servodegree);
              servoAtLeft = true;
              servoAtRight = false;
            }
          } else if (code == RIGHT_CODE) {
            if (servoAtRight) {
              servo.write(servostarting);
              servoAtRight = false;
            } else {
              servo.write(servostarting + servodegree);
              servoAtRight = true;
              servoAtLeft = false;
            }
          } else if (code == RC_MODE) {
            rcmode = !rcmode;
            digitalWrite(RC_LED_PIN, rcmode ? HIGH : LOW);
            Serial.print(F("[RC MODE] "));
            Serial.println(rcmode ? F("ENABLED (LED ON)") : F("DISABLED (LED OFF)"));
          }
        }
      }
    }

    IrReceiver.resume();
  }

  // 5. Main Stepper Drive Engine
  if (systemPower && stepState != 0) {
    digitalWrite(ENABLE_PIN, LOW);

    if (stepState == 1) {
      digitalWrite(DIR_PIN, DIR_FORWARD); // Sets LOW = drives physically FORWARD
    } else if (stepState == -1) {
      digitalWrite(DIR_PIN, DIR_REVERSE); // Sets HIGH = drives physically BACKWARD
    }

    uint32_t currentMicros = micros();
    if (currentMicros - lastStepTime >= stepIntervalMicros) {
      lastStepTime = currentMicros;
      pulseStep();
    }

  } else {
    digitalWrite(STEP_PIN, LOW);
    digitalWrite(ENABLE_PIN, HIGH);
  }
}
