#include <Wire.h>
#include <ESP32Servo.h>

// ============================================================
// PID AUTOMATIC SECURITY BARRIER
// ESP32 + HC-SR04 + AS5600 + SG90 servo

// ---------------- PIN CONFIGURATION ----------------
#define TRIG_PIN    5
#define ECHO_PIN    18
#define SERVO_PIN   13
#define LED_PIN     2
#define BUZZER_PIN  4

#define SDA_PIN     21
#define SCL_PIN     22

// ---------------- AS5600 ----------------
#define AS5600_ADDR 0x36
#define RAW_ANGLE_H 0x0C

// ---------------- DISTANCE THRESHOLDS ----------------
float farLimit = 30.0;   // Above this: barrier target = 150 degrees
float midLimit = 20.0;   // Between thresholds: target = 120 degrees
const float DIST_HYST = 0.7;

// ---------------- BARRIER TARGET ANGLES ----------------
const float SERVO_REF0  = 80.0;
const float SERVO_REF40 = 120.0;
const float SERVO_REF70 = 150.0;

const float SERVO_MIN_ANGLE = 80.0;
const float SERVO_MAX_ANGLE = 150.0;

// ---------------- PID GAINS ----------------
// These are initial tuning values, not experimentally validated.
// PID output is treated as a commanded servo speed (degrees/sec).
float Kp = 3.0;
float Ki = 0.20;
float Kd = 0.08;

const float INTEGRAL_LIMIT = 50.0;
const unsigned long CONTROL_INTERVAL_MS = 50;

// ---------------- CONTROL VARIABLES ----------------
Servo gateServo;

float targetAngle = SERVO_REF0;
float servoCommand = SERVO_REF0;

float measuredAngle = -1.0;
float distanceCm = -1.0;
float error = 0.0;

float integral = 0.0;
float previousMeasurement = -1.0;

float pTerm = 0.0;
float iTerm = 0.0;
float dTerm = 0.0;
float pidOutput = 0.0;

bool autoMode = true;

unsigned long lastControlTime = 0;
unsigned long lastPrintTime = 0;
unsigned long lastDistanceTime = 0;

const unsigned long PRINT_INTERVAL_MS = 200;
const unsigned long DISTANCE_INTERVAL_MS = 60;

// ---------------- AS5600 ANGLE READING ----------------
float readAS5600Once() {
  Wire.beginTransmission(AS5600_ADDR);
  Wire.write(RAW_ANGLE_H);

  if (Wire.endTransmission(false) != 0) {
    return -1.0;
  }

  if (Wire.requestFrom(AS5600_ADDR, 2) != 2) {
    return -1.0;
  }

  uint16_t rawAngle = ((uint16_t)Wire.read() << 8) | Wire.read();
  rawAngle &= 0x0FFF;

  return rawAngle * 360.0 / 4096.0;
}

// Median filter to reduce noise in the angle measurement.
float readAS5600() {
  const int samples = 5;
  float values[samples];
  int count = 0;

  for (int i = 0; i < samples; i++) {
    float angle = readAS5600Once();

    if (angle >= 0.0) {
      values[count++] = angle;
    }

    delay(2);
  }

  if (count == 0) {
    return -1.0;
  }

  for (int i = 0; i < count - 1; i++) {
    for (int j = i + 1; j < count; j++) {
      if (values[j] < values[i]) {
        float temp = values[i];
        values[i] = values[j];
        values[j] = temp;
      }
    }
  }

  return values[count / 2];
}

// ---------------- ULTRASONIC DISTANCE ----------------
float readDistanceCm() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(3);

  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  unsigned long duration = pulseIn(ECHO_PIN, HIGH, 30000);

  if (duration == 0) {
    return -1.0;
  }

  return duration * 0.0343 / 2.0;
}

// ---------------- DISTANCE TO TARGET ANGLE ----------------
void updateTargetFromDistance(float distance) {
  if (distance < 0.0) {
    return;
  }

  // Hysteresis prevents rapid switching near the thresholds.
  if (targetAngle == SERVO_REF70) {
    if (distance <= midLimit - DIST_HYST) {
      targetAngle = SERVO_REF0;
    } else if (distance <= farLimit - DIST_HYST) {
      targetAngle = SERVO_REF40;
    }
  }
  else if (targetAngle == SERVO_REF40) {
    if (distance > farLimit + DIST_HYST) {
      targetAngle = SERVO_REF70;
    } else if (distance < midLimit - DIST_HYST) {
      targetAngle = SERVO_REF0;
    }
  }
  else { // Current target is SERVO_REF0
    if (distance > farLimit + DIST_HYST) {
      targetAngle = SERVO_REF70;
    } else if (distance > midLimit + DIST_HYST) {
      targetAngle = SERVO_REF40;
    }
  }
}

// ---------------- PID ANGLE CONTROLLER ----------------
void runPIDController() {
  unsigned long now = millis();

  if (now - lastControlTime < CONTROL_INTERVAL_MS) {
    return;
  }

  float dt = (now - lastControlTime) / 1000.0;
  lastControlTime = now;

  if (dt <= 0.0) {
    return;
  }

  measuredAngle = readAS5600();

  // If the angle sensor fails, hold the last servo command.
  // Do not calculate PID from an invalid measurement.
  if (measuredAngle < 0.0) {
    return;
  }

  error = targetAngle - measuredAngle;

  // Derivative on measurement avoids derivative kick when
  // the target angle changes suddenly.
  float measurementRate = 0.0;

  if (previousMeasurement >= 0.0) {
    measurementRate = (measuredAngle - previousMeasurement) / dt;
  }

  previousMeasurement = measuredAngle;

  // Proportional and derivative terms.
  pTerm = Kp * error;
  dTerm = -Kd * measurementRate;

  // Conditional integration helps prevent integral windup.
  float proposedIntegral = integral + error * dt;
  proposedIntegral = constrain(
    proposedIntegral, -INTEGRAL_LIMIT, INTEGRAL_LIMIT
  );

  float proposedITerm = Ki * proposedIntegral;
  float proposedOutput = pTerm + proposedITerm + dTerm;

  bool pushingFurtherIntoUpperLimit =
    (servoCommand >= SERVO_MAX_ANGLE && proposedOutput > 0.0);

  bool pushingFurtherIntoLowerLimit =
    (servoCommand <= SERVO_MIN_ANGLE && proposedOutput < 0.0);

  if (!pushingFurtherIntoUpperLimit &&
      !pushingFurtherIntoLowerLimit) {
    integral = proposedIntegral;
  }

  iTerm = Ki * integral;
  pidOutput = pTerm + iTerm + dTerm;

  // PID output is a speed command in degrees/second.
  // Integrating it updates the servo position command.
  servoCommand += pidOutput * dt;
  servoCommand = constrain(
    servoCommand, SERVO_MIN_ANGLE, SERVO_MAX_ANGLE
  );

  gateServo.write((int)round(servoCommand));
}

// ---------------- WARNING LED AND BUZZER ----------------
void updateWarning(float distance) {
  if (distance >= 0.0 && distance <= farLimit) {
    int intensity = map(
      constrain((int)(distance * 10.0), 0, (int)(farLimit * 10.0)),
      (int)(farLimit * 10.0), 0, 0, 255
    );

    analogWrite(LED_PIN, intensity);

    int buzzerFrequency = map(
      constrain((int)distance, 0, (int)farLimit),
      (int)farLimit, 0, 500, 2200
    );

    tone(BUZZER_PIN, buzzerFrequency);
  } else {
    analogWrite(LED_PIN, 0);
    noTone(BUZZER_PIN);
  }
}

// ---------------- SERIAL COMMANDS ----------------
void printHelp() {
  Serial.println("Commands:");
  Serial.println("PID,kp,ki,kd");
  Serial.println("SET,far,mid");
  Serial.println("MODE,AUTO");
  Serial.println("MODE,MANUAL");
  Serial.println("REF,0");
  Serial.println("REF,40");
  Serial.println("REF,70");
  Serial.println("HELP");
}

void handleSerial() {
  if (!Serial.available()) {
    return;
  }

  String command = Serial.readStringUntil('\n');
  command.trim();

  if (command.equalsIgnoreCase("HELP")) {
    printHelp();
  }
  else if (command.equalsIgnoreCase("MODE,AUTO")) {
    autoMode = true;
    Serial.println("ACK,MODE,AUTO");
  }
  else if (command.equalsIgnoreCase("MODE,MANUAL")) {
    autoMode = false;
    Serial.println("ACK,MODE,MANUAL");
  }
  else if (command.equalsIgnoreCase("REF,0")) {
    targetAngle = SERVO_REF0;
    Serial.println("ACK,REF,0");
  }
  else if (command.equalsIgnoreCase("REF,40")) {
    targetAngle = SERVO_REF40;
    Serial.println("ACK,REF,40");
  }
  else if (command.equalsIgnoreCase("REF,70")) {
    targetAngle = SERVO_REF70;
    Serial.println("ACK,REF,70");
  }
  else if (command.startsWith("PID,")) {
    float kp, ki, kd;

    if (sscanf(command.c_str(), "PID,%f,%f,%f", &kp, &ki, &kd) == 3 &&
        kp >= 0.0 && ki >= 0.0 && kd >= 0.0) {
      Kp = kp;
      Ki = ki;
      Kd = kd;

      integral = 0.0;
      previousMeasurement = -1.0;

      Serial.printf("ACK,PID,%.4f,%.4f,%.4f\n", Kp, Ki, Kd);
    } else {
      Serial.println("ERR,PID_FORMAT_OR_NEGATIVE_GAIN");
    }
  }
  else if (command.startsWith("SET,")) {
    float farValue, midValue;

    if (sscanf(command.c_str(), "SET,%f,%f", &farValue, &midValue) == 2 &&
        farValue > midValue && midValue > 0.0) {
      farLimit = farValue;
      midLimit = midValue;

      Serial.printf("ACK,SET,%.1f,%.1f\n", farLimit, midLimit);
    } else {
      Serial.println("ERR,SET_FORMAT");
    }
  }
  else {
    Serial.println("ERR,UNKNOWN_COMMAND");
  }
}

// ---------------- SETUP ----------------
void setup() {
  Serial.begin(115200);
  Serial.setTimeout(50);

  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  pinMode(LED_PIN, OUTPUT);
  pinMode(BUZZER_PIN, OUTPUT);

  Wire.begin(SDA_PIN, SCL_PIN);

  gateServo.setPeriodHertz(50);
  gateServo.attach(SERVO_PIN, 500, 2400);

  servoCommand = SERVO_REF0;
  targetAngle = SERVO_REF0;
  gateServo.write((int)SERVO_REF0);

  delay(500);

  lastControlTime = millis();

  Serial.println("PID Barrier Controller Ready");
  Serial.println("Distance bands: >30cm = 150deg, 20-30cm = 120deg, <=20cm = 80deg");
  Serial.println("AS5600 feedback uses direct angle readings; no calibration ranges.");
  printHelp();
}

// ---------------- MAIN LOOP ----------------
void loop() {
  handleSerial();

  unsigned long now = millis();

  // Read distance at a controlled interval.
  if (now - lastDistanceTime >= DISTANCE_INTERVAL_MS) {
    lastDistanceTime = now;

    float newDistance = readDistanceCm();

    if (newDistance >= 2.0 && newDistance <= 400.0) {
      distanceCm = newDistance;

      if (autoMode) {
        updateTargetFromDistance(distanceCm);
      }

      updateWarning(distanceCm);
    }
  }

  runPIDController();

  // CSV-style serial data for monitoring or GUI plotting.
  if (now - lastPrintTime >= PRINT_INTERVAL_MS) {
    lastPrintTime = now;

    Serial.printf(
      "DATA,%lu,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%s\n",
      now,
      distanceCm,
      targetAngle,
      measuredAngle,
      error,
      pTerm,
      iTerm,
      dTerm,
      servoCommand,
      autoMode ? "AUTO" : "MANUAL"
    );
  }
}
