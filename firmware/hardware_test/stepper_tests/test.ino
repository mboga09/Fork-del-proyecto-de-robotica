#include <Arduino.h>
#include <Servo.h>

// ---------------------------------------------------------
// Wiring actual
// ---------------------------------------------------------

#ifndef Z_SERVO_PIN
#define Z_SERVO_PIN 3
#endif

#ifndef SERVO_2_PIN
#define SERVO_2_PIN 6
#endif

#ifndef SERVO_3_PIN
#define SERVO_3_PIN 5
#endif

// No usamos TOOL_SERVO_PIN en esta prueba porque está repetido con SERVO_2_PIN.

// ---------------------------------------------------------
// Calibración Z - MG996R continuo
// ---------------------------------------------------------

#define Z_STOP_US 1500
#define Z_UP_US   1700
#define Z_DOWN_US 1300

// Duración corta para pruebas
#define Z_TEST_TIME_MS 300

// ---------------------------------------------------------
// Servos
// ---------------------------------------------------------

Servo zServo;
Servo servo2;
Servo servo3;

// Posiciones actuales
int servo2Angle = 45;
int servo3Angle = 90;

// ---------------------------------------------------------
// Helpers
// ---------------------------------------------------------

void stopZ() {
  zServo.writeMicroseconds(Z_STOP_US);
  Serial.println("Z STOP");
}

void moveZUp(unsigned long durationMs) {
  Serial.println("Z UP");
  zServo.writeMicroseconds(Z_UP_US);
  delay(durationMs);
  stopZ();
}

void moveZDown(unsigned long durationMs) {
  Serial.println("Z DOWN");
  zServo.writeMicroseconds(Z_DOWN_US);
  delay(durationMs);
  stopZ();
}

void moveServo2(int angle) {
  angle = constrain(angle, 0, 180);
  servo2Angle = angle;
  servo2.write(servo2Angle);

  Serial.print("SERVO 2 -> ");
  Serial.print(servo2Angle);
  Serial.println(" deg");
}

void moveServo3(int angle) {
  angle = constrain(angle, 0, 180);
  servo3Angle = angle;
  servo3.write(servo3Angle);

  Serial.print("SERVO 3 -> ");
  Serial.print(servo3Angle);
  Serial.println(" deg");
}

void printHelp() {
  Serial.println();
  Serial.println("=== Motor Test Commands ===");
  Serial.println("n  -> neutral positions");
  Serial.println("u  -> Z up short pulse");
  Serial.println("d  -> Z down short pulse");
  Serial.println("s  -> Z stop");
  Serial.println("2a -> Servo 2 to 0 deg");
  Serial.println("2b -> Servo 2 to 45 deg");
  Serial.println("2c -> Servo 2 to 90 deg");
  Serial.println("2d -> Servo 2 to 135 deg");
  Serial.println("2e -> Servo 2 to 180 deg");
  Serial.println("3a -> Servo 3 to 0 deg");
  Serial.println("3b -> Servo 3 to 60 deg");
  Serial.println("3c -> Servo 3 to 90 deg");
  Serial.println("3d -> Servo 3 to 120 deg");
  Serial.println("3e -> Servo 3 to 180 deg");
  Serial.println("demo -> automatic demo");
  Serial.println("h  -> help");
  Serial.println("===========================");
  Serial.println();
}

void neutralPosition() {
  Serial.println("Moving to neutral position...");

  stopZ();
  moveServo2(45);
  moveServo3(90);

  delay(500);

  Serial.println("Neutral ready.");
}

void demoMotion() {
  Serial.println("Starting demo...");

  neutralPosition();
  delay(1000);

  Serial.println("Testing Servo 2...");
  moveServo2(0);
  delay(700);
  moveServo2(45);
  delay(700);
  moveServo2(90);
  delay(700);
  moveServo2(135);
  delay(700);
  moveServo2(45);
  delay(1000);

  Serial.println("Testing Servo 3...");
  moveServo3(60);
  delay(700);
  moveServo3(90);
  delay(700);
  moveServo3(120);
  delay(700);
  moveServo3(90);
  delay(1000);

  Serial.println("Testing Z...");
  moveZUp(Z_TEST_TIME_MS);
  delay(1000);
  moveZDown(Z_TEST_TIME_MS);
  delay(1000);

  neutralPosition();

  Serial.println("Demo finished.");
}

// ---------------------------------------------------------
// Arduino setup / loop
// ---------------------------------------------------------

void setup() {
  Serial.begin(115200);

  zServo.attach(Z_SERVO_PIN);
  servo2.attach(SERVO_2_PIN);
  servo3.attach(SERVO_3_PIN);

  delay(500);

  neutralPosition();
  printHelp();
}

void loop() {
  if (!Serial.available()) {
    return;
  }

  String cmd = Serial.readStringUntil('\n');
  cmd.trim();

  if (cmd.length() == 0) {
    return;
  }

  if (cmd == "h") {
    printHelp();
  }

  else if (cmd == "n") {
    neutralPosition();
  }

  else if (cmd == "u") {
    moveZUp(Z_TEST_TIME_MS);
  }

  else if (cmd == "d") {
    moveZDown(Z_TEST_TIME_MS);
  }

  else if (cmd == "s") {
    stopZ();
  }

  else if (cmd == "2a") {
    moveServo2(0);
  }

  else if (cmd == "2b") {
    moveServo2(45);
  }

  else if (cmd == "2c") {
    moveServo2(90);
  }

  else if (cmd == "2d") {
    moveServo2(135);
  }

  else if (cmd == "2e") {
    moveServo2(180);
  }

  else if (cmd == "3a") {
    moveServo3(0);
  }

  else if (cmd == "3b") {
    moveServo3(60);
  }

  else if (cmd == "3c") {
    moveServo3(90);
  }

  else if (cmd == "3d") {
    moveServo3(120);
  }

  else if (cmd == "3e") {
    moveServo3(180);
  }

  else if (cmd == "demo") {
    demoMotion();
  }

  else {
    Serial.print("Unknown command: ");
    Serial.println(cmd);
    printHelp();
  }
}