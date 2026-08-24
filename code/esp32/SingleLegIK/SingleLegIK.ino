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
const float THIGH_SERVO_MIN = 20;
const float THIGH_SERVO_MAX = 160;

const float KNEE_SERVO_MIN = 20;
const float KNEE_SERVO_MAX = 160;


// ============================================================
// Convert radians -> degrees
// ============================================================
float radToDeg(float radians) {
  return radians * 180.0 / PI;
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
      THIGH_NEUTRAL
      + THIGH_DIRECTION * thighAngle;

  float kneeServoAngle =
      KNEE_NEUTRAL
      + KNEE_DIRECTION * kneeAngle;

  thighServoAngle = constrain(
      thighServoAngle,
      THIGH_SERVO_MIN,
      THIGH_SERVO_MAX
  );

  kneeServoAngle = constrain(
      kneeServoAngle,
      KNEE_SERVO_MIN,
      KNEE_SERVO_MAX
  );

  thighServo.write(thighServoAngle);
  kneeServo.write(kneeServoAngle);

  Serial.println();
  Serial.println("=== Joint Command ===");

  Serial.print("Thigh joint: ");
  Serial.print(thighAngle);
  Serial.print(" deg -> servo ");
  Serial.println(thighServoAngle);

  Serial.print("Knee joint: ");
  Serial.print(kneeAngle);
  Serial.print(" deg -> servo ");
  Serial.println(kneeServoAngle);
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

  Serial.println();
  Serial.println("============================");

  Serial.print("Target X: ");
  Serial.print(x);
  Serial.println(" cm");

  Serial.print("Target Z: ");
  Serial.print(z);
  Serial.println(" cm");

  if (!inverseKinematics(
        x,
        z,
        thighAngle,
        kneeAngle)) {

    return;
  }

  Serial.print("IK thigh: ");
  Serial.println(thighAngle);

  Serial.print("IK knee: ");
  Serial.println(kneeAngle);

  setJointAngles(thighAngle, kneeAngle);
}


// ============================================================
// Setup
// ============================================================
void setup() {

  Serial.begin(115200);
  delay(1500);

  hipServo.setPeriodHertz(50);
  thighServo.setPeriodHertz(50);
  kneeServo.setPeriodHertz(50);

  hipServo.attach(HIP_PIN, 500, 2500);
  thighServo.attach(THIGH_PIN, 500, 2500);
  kneeServo.attach(KNEE_PIN, 500, 2500);

  // Keep hip centered for now
  hipServo.write(HIP_NEUTRAL);

  // Start in calibrated neutral
  thighServo.write(THIGH_NEUTRAL);
  kneeServo.write(KNEE_NEUTRAL);

  Serial.println("Quadruped leg IK ready.");
  Serial.println();
  Serial.println("Coordinate system:");
  Serial.println("  +X = forward");
  Serial.println("  +Z = downward");
  Serial.println();
  Serial.println("Example:");
  Serial.println("  setFootPosition(2, 15);");
}


// ============================================================
// Loop
// ============================================================
void loop() {

  // Example test.
  //
  // Uncomment ONE target at a time while testing.

  setFootPosition(0, 16.0);

  delay(2000);

  setFootPosition(16.0, 0);

  delay(2000);

  setFootPosition(8.0,8.0);
  delay (2000);
  
  setFootPosition(-8.0,8.0);
  delay (2000);
}