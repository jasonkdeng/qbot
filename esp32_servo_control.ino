/*
 * ESP32 Servo Control
 * 
 * Testing servo motor control
 * Uses ESP32Servo library: https://github.com/madhephaestus/ESP32Servo
 */

#include <ESP32Servo.h>

// Create servo objects
Servo servo1;
Servo servo2;

// Define GPIO pins for servos
const int SERVO1_PIN = 18;
const int SERVO2_PIN = 19;

// Servo position variables
int servo1_pos = 90;  // Initial position (degrees)
int servo2_pos = 90;

void setup() {
  Serial.begin(115200);
  
  // Allow allocation of all timers for servos
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);
  
  // Attach servos to pins
  // Standard servo: 500-2400 microseconds pulse width
  servo1.attach(SERVO1_PIN, 500, 2400);
  servo2.attach(SERVO2_PIN, 500, 2400);
  
  // Set initial positions
  servo1.write(servo1_pos);
  servo2.write(servo2_pos);
  
  Serial.println("ESP32 Servo Control Ready");
  Serial.println("Send commands via Serial:");
  Serial.println("  1<angle> - Move servo 1 (e.g., '190' for 90 degrees)");
  Serial.println("  2<angle> - Move servo 2 (e.g., '2180' for 180 degrees)");
}

void loop() {
  // Example 1: Sweep servo back and forth
  // sweepServo(servo1, 0, 180, 15);
  
  // Example 2: Control via Serial commands
  handleSerialCommands();
  
  // Example 3: Simple periodic movement
  // periodicMovement();
  
  delay(10);
}

// Function: Sweep servo smoothly between two angles
void sweepServo(Servo &servo, int minAngle, int maxAngle, int delayMs) {
  // Sweep from min to max
  for (int pos = minAngle; pos <= maxAngle; pos++) {
    servo.write(pos);
    delay(delayMs);
  }
  
  // Sweep back from max to min
  for (int pos = maxAngle; pos >= minAngle; pos--) {
    servo.write(pos);
    delay(delayMs);
  }
}

// Function: Handle serial commands for manual control
void handleSerialCommands() {
  if (Serial.available() > 0) {
    String command = Serial.readStringUntil('\n');
    command.trim();
    
    if (command.length() > 1) {
      char servoNum = command.charAt(0);
      int angle = command.substring(1).toInt();
      
      // Constrain angle to valid range
      angle = constrain(angle, 0, 180);
      
      if (servoNum == '1') {
        servo1.write(angle);
        servo1_pos = angle;
        Serial.printf("Servo 1 moved to %d degrees\n", angle);
      } 
      else if (servoNum == '2') {
        servo2.write(angle);
        servo2_pos = angle;
        Serial.printf("Servo 2 moved to %d degrees\n", angle);
      }
      else {
        Serial.println("Invalid servo number. Use 1 or 2");
      }
    }
  }
}

// Function: Move servos in a pattern
void periodicMovement() {
  static unsigned long lastMove = 0;
  static bool direction = true;
  
  if (millis() - lastMove > 2000) {  // Move every 2 seconds
    if (direction) {
      servo1.write(180);
      servo2.write(0);
    } else {
      servo1.write(0);
      servo2.write(180);
    }
    direction = !direction;
    lastMove = millis();
  }
}

// Function: Move servo smoothly to target position
void moveServoSmooth(Servo &servo, int currentPos, int targetPos, int stepDelay) {
  if (currentPos < targetPos) {
    for (int pos = currentPos; pos <= targetPos; pos++) {
      servo.write(pos);
      delay(stepDelay);
    }
  } else {
    for (int pos = currentPos; pos >= targetPos; pos--) {
      servo.write(pos);
      delay(stepDelay);
    }
  }
}

// Function: Detach servo to save power (stops PWM signal)
void detachServos() {
  servo1.detach();
  servo2.detach();
  Serial.println("Servos detached");
}

// Function: Reattach servos
void reattachServos() {
  servo1.attach(SERVO1_PIN, 500, 2400);
  servo2.attach(SERVO2_PIN, 500, 2400);
  Serial.println("Servos reattached");
}
