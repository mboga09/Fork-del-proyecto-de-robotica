#include <Arduino.h>
#include <ArduinoJson.h>

#include "src/actuator/actuator_controller.h"

// ---------------------------------------------------------
// Estado simple para HMI
// ---------------------------------------------------------

static bool robotHomed = true;
static const char* robotState = "IDLE";

static const float HOME_S2_DEG = 0.0f;
static const float HOME_S3_DEG = 180.0f;

static int currentS2Deg = 0;
static int currentS3Deg = 180;
static int currentToolDeg = 0;

static const unsigned long SERIAL_BAUD = 115200;

// ---------------------------------------------------------
// Envio JSON directo
// ---------------------------------------------------------

void addCommonStatusFields(JsonDocument& doc) {
  doc["state"] = robotState;
  doc["homed"] = robotHomed;
  doc["armed"] = true;
  doc["busy"] = strcmp(robotState, "MOVING") == 0;
  doc["s2_deg"] = currentS2Deg;
  doc["s3_deg"] = currentS3Deg;
  doc["tool_deg"] = currentToolDeg;
}

void directSendStatus(const char* status, const char* message) {
  StaticJsonDocument<256> doc;

  doc["type"] = "status";
  doc["status"] = status;
  addCommonStatusFields(doc);
  doc["message"] = message;

  serializeJson(doc, Serial);
  Serial.println();
}

void directSendAck(const char* cmd, const char* message) {
  StaticJsonDocument<256> doc;

  doc["type"] = "ack";
  doc["cmd"] = cmd;
  doc["ok"] = true;
  addCommonStatusFields(doc);
  doc["message"] = message;

  serializeJson(doc, Serial);
  Serial.println();
}

void directSendError(const char* cmd, const char* message) {
  StaticJsonDocument<256> doc;

  doc["type"] = "error";
  doc["cmd"] = cmd;
  doc["ok"] = false;
  addCommonStatusFields(doc);
  doc["message"] = message;

  serializeJson(doc, Serial);
  Serial.println();
}

bool validServoAngle(float angleDeg) {
  return angleDeg >= 0.0f && angleDeg <= 180.0f;
}

void applyArmServoTargets(float s2Deg, float s3Deg) {
  currentS2Deg = (int)constrain(s2Deg, 0.0f, 180.0f);
  currentS3Deg = (int)constrain(s3Deg, 0.0f, 180.0f);

  // z_dir=0 and z_time_s=0 keep the Z axis stopped while updating S2/S3.
  moveActuators(0, 0.0f, currentS2Deg, currentS3Deg);
}

// ---------------------------------------------------------
// Handlers directos
// ---------------------------------------------------------

void handleMoveActDirect(StaticJsonDocument<256>& doc) {
  if (!doc.containsKey("z_dir") ||
      !doc.containsKey("z_time_s") ||
      !doc.containsKey("s2_deg") ||
      !doc.containsKey("s3_deg")) {
    directSendError("MOVE_ACT", "Missing actuator fields");
    return;
  }

  int zDir = doc["z_dir"];
  float zTimeS = doc["z_time_s"];
  float s2Deg = doc["s2_deg"];
  float s3Deg = doc["s3_deg"];

  if (zDir < -1 || zDir > 1) {
    directSendError("MOVE_ACT", "Invalid z_dir");
    return;
  }

  if (zTimeS < 0.0f) {
    directSendError("MOVE_ACT", "Invalid z_time_s");
    return;
  }

  if (!validServoAngle(s2Deg) || !validServoAngle(s3Deg)) {
    directSendError("MOVE_ACT", "Servo angle out of range");
    return;
  }

  currentS2Deg = (int)s2Deg;
  currentS3Deg = (int)s3Deg;

  directSendAck("MOVE_ACT", "MOVE_ACT received");

  robotState = "MOVING";
  directSendStatus("MOVING", "Actuator movement started");

  moveActuators(zDir, zTimeS, currentS2Deg, currentS3Deg);

  robotState = "IDLE";
  directSendStatus("IDLE", "Actuator movement completed");
}

void handleMoveServosDirect(StaticJsonDocument<256>& doc) {
  if (!doc.containsKey("s2_deg") && !doc.containsKey("s3_deg")) {
    directSendError("MOVE_SERVOS", "Expected s2_deg and/or s3_deg");
    return;
  }

  float requestedS2Deg = doc["s2_deg"] | currentS2Deg;
  float requestedS3Deg = doc["s3_deg"] | currentS3Deg;

  if (!validServoAngle(requestedS2Deg) || !validServoAngle(requestedS3Deg)) {
    directSendError("MOVE_SERVOS", "Servo angle out of range");
    return;
  }

  directSendAck("MOVE_SERVOS", "MOVE_SERVOS received");

  robotState = "MOVING";
  directSendStatus("MOVING", "Applying manual servo command");

  applyArmServoTargets(requestedS2Deg, requestedS3Deg);

  robotState = "IDLE";
  directSendStatus("IDLE", "Manual servo command completed");
}

void handleHomeDirect() {
  directSendAck("HOME", "HOME received");

  robotState = "MOVING";
  directSendStatus("MOVING", "Moving arm to HOME pose");

  applyArmServoTargets(HOME_S2_DEG, HOME_S3_DEG);

  robotHomed = true;
  robotState = "IDLE";
  directSendStatus("HOMED", "Robot referenced at HOME pose");
}

void handleZeroDirect() {
  directSendAck("ZERO", "ZERO received");

  robotState = "MOVING";
  directSendStatus("MOVING", "Moving S2/S3 to 0 degrees");

  applyArmServoTargets(0.0f, 0.0f);

  robotState = "IDLE";
  directSendStatus("IDLE", "ZERO completed");
}

void handleCenterDirect() {
  directSendAck("CENTER", "CENTER received");

  robotState = "MOVING";
  directSendStatus("MOVING", "Moving S2/S3 to 90 degrees");

  applyArmServoTargets(90.0f, 90.0f);

  robotState = "IDLE";
  directSendStatus("IDLE", "CENTER completed");
}

void handleToolAspirateDirect() {
  directSendAck("TOOL_ASPIRATE", "Tool aspirate received");

  robotState = "MOVING";
  directSendStatus("MOVING", "Tool aspirating");

  toolAspirate();

  robotState = "IDLE";
  directSendStatus("IDLE", "Tool aspirate completed");
}

void handleToolDispenseDirect() {
  directSendAck("TOOL_DISPENSE", "Tool dispense received");

  robotState = "MOVING";
  directSendStatus("MOVING", "Tool dispensing");

  toolDispense();

  robotState = "IDLE";
  directSendStatus("IDLE", "Tool dispense completed");
}

void processDirectJsonLine(String line) {
  line.trim();

  if (line.length() == 0) {
    return;
  }

  StaticJsonDocument<256> rawDoc;
  rawDoc["type"] = "status";
  rawDoc["status"] = "DEBUG_RAW";
  addCommonStatusFields(rawDoc);
  rawDoc["message"] = line;
  serializeJson(rawDoc, Serial);
  Serial.println();

  StaticJsonDocument<256> doc;
  DeserializationError error = deserializeJson(doc, line);

  if (error) {
    directSendError("UNKNOWN", error.c_str());
    return;
  }

  const char* cmd = doc["cmd"] | "";

  StaticJsonDocument<128> debugDoc;
  debugDoc["type"] = "status";
  debugDoc["status"] = "DEBUG_CMD";
  addCommonStatusFields(debugDoc);
  debugDoc["message"] = cmd;
  serializeJson(debugDoc, Serial);
  Serial.println();

  if (strcmp(cmd, "PING") == 0) {
    directSendAck("PING", "Controller alive");
  }

  else if (strcmp(cmd, "HOME") == 0) {
    handleHomeDirect();
  }

  else if (strcmp(cmd, "STOP") == 0) {
    stopZAxis();
    robotState = "STOPPED";
    directSendAck("STOP", "STOP received");
    directSendStatus("STOPPED", "Robot stopped");
  }

  else if (strcmp(cmd, "ESTOP") == 0) {
    stopZAxis();
    robotState = "ESTOPPED";
    directSendAck("ESTOP", "ESTOP received");
    directSendStatus("ESTOPPED", "E-stop active");
  }

  else if (strcmp(cmd, "MOVE_ACT") == 0) {
    handleMoveActDirect(doc);
  }

  else if (strcmp(cmd, "MOVE_SERVOS") == 0) {
    handleMoveServosDirect(doc);
  }

  else if (strcmp(cmd, "ZERO") == 0) {
    handleZeroDirect();
  }

  else if (strcmp(cmd, "CENTER") == 0) {
    handleCenterDirect();
  }

  else if (strcmp(cmd, "TOOL_ASPIRATE") == 0) {
    handleToolAspirateDirect();
  }

  else if (strcmp(cmd, "TOOL_DISPENSE") == 0) {
    handleToolDispenseDirect();
  }

  else {
    directSendError(cmd, "Unknown command");
  }
}

// ---------------------------------------------------------
// Arduino setup / loop
// ---------------------------------------------------------

void setup() {
  Serial.begin(SERIAL_BAUD);
  Serial.setTimeout(100);

  initializeActuators();

  delay(500);

  robotHomed = true;
  robotState = "IDLE";
  currentS2Deg = (int)HOME_S2_DEG;
  currentS3Deg = (int)HOME_S3_DEG;
  currentToolDeg = 0;

  directSendStatus("HOMED", "DIRECT FIRMWARE READY");
}

void loop() {
  if (!Serial.available()) {
    return;
  }

  String line = Serial.readStringUntil('\n');
  processDirectJsonLine(line);
}
