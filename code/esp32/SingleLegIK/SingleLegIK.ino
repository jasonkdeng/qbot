#include <ESP32Servo.h>
#include <math.h>

Servo hipServo;
Servo thighServo;
Servo kneeServo;

// =============================
// ESP32 pins
// =============================
const int HIP_PIN   = 18;
const int THIGH_PIN = 19;
const int KNEE_PIN  = 21;

// =============================
// Leg geometry (cm)
// =============================
const float L1 = 7.08;   // thigh: hip pitch axis -> knee axis
const float L2 = 11.0;   // shin: knee axis -> foot

// =============================
// Servo calibration
// =============================
const float HIP_NEUTRAL   = 97.0;
const float THIGH_NEUTRAL = 86.0;
const float KNEE_NEUTRAL  = 94.0;

// Based on your physical servo mounting
const int THIGH_DIRECTION = -1;
const int KNEE_DIRECTION  = +1;

// Start conservatively.
// Replace these with your actual calibrated safe limits.
const float THIGH_SERVO_MIN = -60;
const float THIGH_SERVO_MAX = 260;

const float KNEE_SERVO_MIN = -60;
const float KNEE_SERVO_MAX = 260;


// ============================================================
// Convert radians -> degrees
// ============================================================
float radToDeg(float radians) {
  return radians * 180.0 / PI;
}

void traceOval(
    float centerX,
    float centerZ,
    float radiusX,
    float radiusZ,
    unsigned long durationMs
) {
  const int updatePeriodMs = 20;   // 50 Hz updates

  int steps = max(
      1,
      (int)(durationMs / updatePeriodMs)
  );

  for (int i = 0; i <= steps; i++) {

    float t = (float)i / (float)steps;
    float theta = 2.0 * PI * t;

    float x =
        centerX +
        radiusX * cos(theta);

    float z =
        centerZ +
        radiusZ * sin(theta);

    setFootPosition(x, z);

    // Give the servo/controller time before next command
    delay(updatePeriodMs);
  }
}

// ============================================================
// Send mathematical joint angles to servos
//
// Convention:
//
// thighAngle:
//   0° = thigh straight down
//   positive = thigh moves forward
//
// kneeAngle:
//   0° = leg completely straight
//   positive = knee bends forward
//
// ============================================================
void setJointAngles(float thighAngle, float kneeAngle) {

  float thighServoAngle =
      THIGH_NEUTRAL +
      THIGH_DIRECTION * thighAngle;

  float kneeServoAngle =
      KNEE_NEUTRAL +
      KNEE_DIRECTION * kneeAngle;

  Serial.print("IK thigh: ");
  Serial.print(thighAngle);
  Serial.print(" | Servo thigh: ");
  Serial.print(thighServoAngle);

  Serial.print(" | IK knee: ");
  Serial.print(kneeAngle);
  Serial.print(" | Servo knee: ");
  Serial.println(kneeServoAngle);

  if (thighServoAngle < THIGH_SERVO_MIN ||
      thighServoAngle > THIGH_SERVO_MAX ||
      kneeServoAngle < KNEE_SERVO_MIN ||
      kneeServoAngle > KNEE_SERVO_MAX) {

    Serial.println("Servo command outside limits");
    return;
  }

  int thighPulse =
      500 + (thighServoAngle / 180.0) * 2000.0;

  int kneePulse =
      500 + (kneeServoAngle / 180.0) * 2000.0;

  thighServo.writeMicroseconds(thighPulse);
  kneeServo.writeMicroseconds(kneePulse);
}


// ============================================================
// 2D Inverse Kinematics
//
// Coordinate system:
//
//           +x forward
//           -->
//
//       hip O
//           |
//           |
//           |
//           v +z downward
//
// So:
// x > 0 = foot forward
// z > 0 = foot below hip
//
// ============================================================
bool inverseKinematics(float x, float z,
                       float &thighAngle,
                       float &kneeAngle) {

  float distanceSquared = x * x + z * z;
  float distance = sqrt(distanceSquared);

  // -------------------------
  // Reachability check
  // -------------------------

  float maxReach = L1 + L2;
  float minReach = fabs(L2 - L1);

  if (distance > maxReach || distance < minReach) {
    Serial.println("ERROR: Target is outside reachable workspace.");
    return false;
  }

  // -------------------------
  // Knee angle
  //
  // Law of cosines
  // -------------------------

  float cosKnee =
      (L1 * L1 + L2 * L2 - distanceSquared)
      / (2.0 * L1 * L2);

  // Protect acos against floating-point error
  cosKnee = constrain(cosKnee, -1.0, 1.0);

  float internalKnee = acos(cosKnee);

  // Our knee convention:
  //
  // 0° = straight leg
  //
  // Internal triangle angle is 180° when straight,
  // so convert it.
  kneeAngle = 180.0 - radToDeg(internalKnee);

  // -------------------------
  // Thigh angle
  // -------------------------

  float angleToTarget = atan2(x, z);

  float cosHip =
      (L1 * L1 + distanceSquared - L2 * L2)
      / (2.0 * L1 * distance);

  cosHip = constrain(cosHip, -1.0, 1.0);

  float hipTriangleAngle = acos(cosHip);

  // Choose the knee-bent configuration
  float thighRad =
      angleToTarget - hipTriangleAngle;

  thighAngle = radToDeg(thighRad);

  return true;
}


// ============================================================
// Command foot position
// ============================================================
void setFootPosition(float x, float z) {

  float thighAngle;
  float kneeAngle;

  if (!inverseKinematics(
        x,
        z,
        thighAngle,
        kneeAngle)) {

    return;
  }

  setJointAngles(thighAngle, kneeAngle);
}


// ============================================================
// Setup
// ============================================================
void setup() {
  Serial.begin(115200);
  delay(1500);

  Serial.println("===== NEW THIGH TEST VERSION =====");

  hipServo.setPeriodHertz(50);
  thighServo.setPeriodHertz(50);
  kneeServo.setPeriodHertz(50);

  hipServo.attach(HIP_PIN, 500, 2500);
  thighServo.attach(THIGH_PIN, 500, 2500);
  kneeServo.attach(KNEE_PIN, 500, 2500);

  hipServo.write(HIP_NEUTRAL);
  thighServo.write(THIGH_NEUTRAL);
  kneeServo.write(KNEE_NEUTRAL);
}

struct LegWalkState {
  unsigned long startTime;
  int step;
};

LegWalkState walkState = {
  0,
  0
};

void updateWalk() {
  unsigned long now = millis();

  switch (walkState.step) {

    case 0:
      setFootPosition(-4, 14);
      walkState.startTime = now;
      walkState.step = 1;
      break;

    case 1:
      if (now - walkState.startTime >= 500) {
        setFootPosition(4, 14);

        walkState.startTime = now;
        walkState.step = 2;
      }
      break;

    case 2:
      if (now - walkState.startTime >= 100) {
        setFootPosition(3, 13);

        walkState.startTime = now;
        walkState.step = 3;
      }
      break;

    case 3:
      if (now - walkState.startTime >= 100) {
        setFootPosition(0, 12);

        walkState.startTime = now;
        walkState.step = 4;
      }
      break;

    case 4:
      if (now - walkState.startTime >= 100) {
        setFootPosition(-3, 13);

        walkState.startTime = now;
        walkState.step = 5;
      }
      break;

    case 5:
      if (now - walkState.startTime >= 100) {
        setFootPosition(-4, 14);

        walkState.startTime = now;
        walkState.step = 1;
      }
      break;
  }
}

void loop() {
  updateWalk();

  // thighServo.write(86);
  // delay(1000);

  // thighServo.write(100);
  // delay(1000);

  // thighServo.write(120);
  // delay(1000);

  // thighServo.write(100);
  // delay(1000);
  // Serial.println("hi");

  // Other code can run here at the same time.
}
