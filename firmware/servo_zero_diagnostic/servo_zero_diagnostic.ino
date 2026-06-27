#include <Arduino.h>
#include <ArduinoJson.h>

#if defined(ESP32)
#include <ESP32Servo.h>
#else
#include <Servo.h>
#endif

// -----------------------------------------------------------------------------
// Standalone servo diagnostic firmware
// -----------------------------------------------------------------------------
// Purpose:
//   Temporarily load this sketch to the ESP32/Arduino to find the physical zero
//   position of the two robot arm servos without requiring HOME/ARM.
//
// Important:
//   - This sketch intentionally bypasses the robot state machine.
//   - It does not move the Z axis.
//   - Verify the pin mapping below before uploading.
//   - Use small angle changes first to avoid mechanical collisions.
//
// Serial protocol examples, one JSON object per line:
//   {"cmd":"PING"}
//   {"cmd":"MOVE_SERVOS","s2_deg":45,"s3_deg":90}
//   {"cmd":"MOVE_SERVOS","s2_deg":0,"s3_deg":0}
//   {"cmd":"ZERO"}
//   {"cmd":"CENTER"}
// -----------------------------------------------------------------------------

// Change this to match the baud rate used by your serial monitor.
static const unsigned long SERIAL_BAUD = 115200;

// Update these pins to match the ESP32-WROOM wiring.
// The previous actuator_controller.cpp defaults were SERVO_2_PIN=6 and
// SERVO_3_PIN=5, but GPIO6 is usually reserved for flash on ESP32-WROOM.
// For ESP32-WROOM, choose real available GPIOs wired to the servo signals.
#ifndef SERVO_2_PIN
#define SERVO_2_PIN 19
#endif

#ifndef SERVO_3_PIN
#define SERVO_3_PIN 21
#endif

// Safe starting positions. These match the last reported firmware status.
static int s2Deg = 45;
static int s3Deg = 90;

static Servo servo2;
static Servo servo3;

static bool validAngle(int angle) {
  return angle >= 0 && angle <= 180;
}

static void sendJsonStatus(const char* status, const char* message) {
  StaticJsonDocument<256> doc;

  doc["type"] = "status";
  doc["status"] = status;
  doc["state"] = "DIAGNOSTIC";
  doc["armed"] = true;
  doc["busy"] = false;
  doc["s2_deg"] = s2Deg;
  doc["s3_deg"] = s3Deg;
  doc["message"] = message;

  serializeJson(doc, Serial);
  Serial.println();
}

static void sendJsonAck(const char* cmd, const char* message) {
  StaticJsonDocument<256> doc;

  doc["type"] = "ack";
  doc["cmd"] = cmd;
  doc["ok"] = true;
  doc["s2_deg"] = s2Deg;
  doc["s3_deg"] = s3Deg;
  doc["message"] = message;

  serializeJson(doc, Serial);
  Serial.println();
}

static void sendJsonError(const char* cmd, const char* message) {
  StaticJsonDocument<256> doc;

  doc["type"] = "error";
  doc["cmd"] = cmd;
  doc["ok"] = false;
  doc["s2_deg"] = s2Deg;
  doc["s3_deg"] = s3Deg;
  doc["message"] = message;

  serializeJson(doc, Serial);
  Serial.println();
}

static void applyServoPositions() {
  servo2.write(s2Deg);
  servo3.write(s3Deg);
  delay(150);
}

static void handleMoveServos(JsonDocument& doc) {
  if (!doc.containsKey("s2_deg") && !doc.containsKey("s3_deg")) {
    sendJsonError("MOVE_SERVOS", "Expected s2_deg and/or s3_deg");
    return;
  }

  int requestedS2 = doc["s2_deg"] | s2Deg;
  int requestedS3 = doc["s3_deg"] | s3Deg;

  if (!validAngle(requestedS2) || !validAngle(requestedS3)) {
    sendJsonError("MOVE_SERVOS", "Angle out of range. Use 0 to 180 degrees.");
    return;
  }

  s2Deg = requestedS2;
  s3Deg = requestedS3;

  sendJsonStatus("MOVING", "Applying diagnostic servo position");
  applyServoPositions();
  sendJsonAck("MOVE_SERVOS", "Servo position applied");
  sendJsonStatus("IDLE", "Diagnostic movement completed");
}

static void processJsonLine(String line) {
  line.trim();

  if (line.length() == 0) {
    return;
  }

  StaticJsonDocument<256> doc;
  DeserializationError error = deserializeJson(doc, line);

  if (error) {
    sendJsonError("UNKNOWN", error.c_str());
    return;
  }

  const char* cmd = doc["cmd"] | "";

  if (strcmp(cmd, "PING") == 0) {
    sendJsonAck("PING", "Servo diagnostic firmware alive");
    sendJsonStatus("IDLE", "Ready");
  }
  else if (strcmp(cmd, "MOVE_SERVOS") == 0) {
    handleMoveServos(doc);
  }
  else if (strcmp(cmd, "ZERO") == 0) {
    s2Deg = 0;
    s3Deg = 0;
    sendJsonStatus("MOVING", "Moving S2/S3 to 0 degrees");
    applyServoPositions();
    sendJsonAck("ZERO", "S2 and S3 moved to 0 degrees");
    sendJsonStatus("IDLE", "Zero command completed");
  }
  else if (strcmp(cmd, "CENTER") == 0) {
    s2Deg = 90;
    s3Deg = 90;
    sendJsonStatus("MOVING", "Moving S2/S3 to 90 degrees");
    applyServoPositions();
    sendJsonAck("CENTER", "S2 and S3 moved to 90 degrees");
    sendJsonStatus("IDLE", "Center command completed");
  }
  else {
    sendJsonError(cmd, "Unknown diagnostic command");
  }
}

void setup() {
  Serial.begin(SERIAL_BAUD);
  Serial.setTimeout(100);

#if defined(ESP32)
  // Standard 50 Hz servo signal. Min/max pulse values are typical for MG995/MG996.
  servo2.setPeriodHertz(50);
  servo3.setPeriodHertz(50);
  servo2.attach(SERVO_2_PIN, 500, 2500);
  servo3.attach(SERVO_3_PIN, 500, 2500);
#else
  servo2.attach(SERVO_2_PIN);
  servo3.attach(SERVO_3_PIN);
#endif

  applyServoPositions();

  delay(500);
  sendJsonStatus("READY", "Standalone servo zero diagnostic firmware ready");
}

void loop() {
  if (!Serial.available()) {
    return;
  }

  String line = Serial.readStringUntil('\n');
  processJsonLine(line);
}
