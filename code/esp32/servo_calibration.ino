#include <ESP32Servo.h>

Servo hipServo;
Servo thighServo;
Servo kneeServo;

const int HIP_PIN   = 18;
const int THIGH_PIN = 19;
const int KNEE_PIN  = 21;

bool hipAttached   = false;
bool thighAttached = false;
bool kneeAttached  = false;

void setup() {
  Serial.begin(115200);

  Serial.println("MG996R calibration tool");
  Serial.println("Commands:");
  Serial.println("ah / at / ak   -> attach hip/thigh/knee");
  Serial.println("dh / dt / dk   -> detach hip/thigh/knee");
  Serial.println("h 90           -> hip to 90 deg");
  Serial.println("t 90           -> thigh to 90 deg");
  Serial.println("k 90           -> knee to 90 deg");
}

void loop() {
  if (!Serial.available()) return;

  String command = Serial.readStringUntil('\n');
  command.trim();

  if (command == "ah") {
    if (!hipAttached) {
      hipServo.setPeriodHertz(50);
      hipServo.attach(HIP_PIN, 500, 2500);
      hipAttached = true;
      Serial.println("Hip attached");
    }
  }

  else if (command == "at") {
    if (!thighAttached) {
      thighServo.setPeriodHertz(50);
      thighServo.attach(THIGH_PIN, 500, 2500);
      thighAttached = true;
      Serial.println("Thigh attached");
    }
  }

  else if (command == "ak") {
    if (!kneeAttached) {
      kneeServo.setPeriodHertz(50);
      kneeServo.attach(KNEE_PIN, 500, 2500);
      kneeAttached = true;
      Serial.println("Knee attached");
    }
  }

  else if (command == "dh") {
    hipServo.detach();
    hipAttached = false;
    Serial.println("Hip detached");
  }

  else if (command == "dt") {
    thighServo.detach();
    thighAttached = false;
    Serial.println("Thigh detached");
  }

  else if (command == "dk") {
    kneeServo.detach();
    kneeAttached = false;
    Serial.println("Knee detached");
  }

  else if (command.startsWith("h ")) {
    if (!hipAttached) {
      Serial.println("Attach hip first with: ah");
      return;
    }

    int angle = constrain(command.substring(2).toInt(), 0, 180);
    hipServo.write(angle);

    Serial.print("Hip -> ");
    Serial.println(angle);
  }

  else if (command.startsWith("t ")) {
    if (!thighAttached) {
      Serial.println("Attach thigh first with: at");
      return;
    }

    int angle = constrain(command.substring(2).toInt(), 0, 180);
    thighServo.write(angle);

    Serial.print("Thigh -> ");
    Serial.println(angle);
  }

  else if (command.startsWith("k ")) {
    if (!kneeAttached) {
      Serial.println("Attach knee first with: ak");
      return;
    }

    int angle = constrain(command.substring(2).toInt(), 0, 180);
    kneeServo.write(angle);

    Serial.print("Knee -> ");
    Serial.println(angle);
  }
}
