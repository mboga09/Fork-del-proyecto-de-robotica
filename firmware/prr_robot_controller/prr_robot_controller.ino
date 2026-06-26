#include <Arduino.h>
#include <ArduinoJson.h>

#include "src/actuator/actuator_controller.h"

// ---------------------------------------------------------
// Estado simple para HMI
// ---------------------------------------------------------

static bool robotHomed = true;
static const char* robotState = "IDLE";

// ---------------------------------------------------------
// Envío JSON directo
// ---------------------------------------------------------

void directSendStatus(const char* status, const char* message) {
  StaticJsonDocument<256> doc;

  doc["type"] = "status";
  doc["status"] = status;
  doc["state"] = robotState;
  doc["homed"] = robotHomed;
  doc["message"] = message;

  serializeJson(doc, Serial);
  Serial.println();
}

void directSendAck(const char* cmd, const char* message) {
  StaticJsonDocument<256> doc;

  doc["type"] = "ack";
  doc["cmd"] = cmd;
  doc["ok"] = true;
  doc["message"] = message;

  serializeJson(doc, Serial);
  Serial.println();
}

void directSendError(const char* cmd, const char* message) {
  StaticJsonDocument<256> doc;

  doc["type"] = "error";
  doc["cmd"] = cmd;
  doc["ok"] = false;
  doc["message"] = message;

  serializeJson(doc, Serial);
  Serial.println();
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

  if (s2Deg < 0.0f || s2Deg > 180.0f ||
      s3Deg < 0.0f || s3Deg > 180.0f) {
    directSendError("MOVE_ACT", "Servo angle out of range");
    return;
  }

  directSendAck("MOVE_ACT", "MOVE_ACT received");

  robotState = "MOVING";
  directSendStatus("MOVING", "Actuator movement started");

  moveActuators(zDir, zTimeS, s2Deg, s3Deg);

  robotState = "IDLE";
  directSendStatus("IDLE", "Actuator movement completed");
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
  rawDoc["state"] = robotState;
  rawDoc["homed"] = robotHomed;
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
  debugDoc["state"] = robotState;
  debugDoc["homed"] = robotHomed;
  debugDoc["message"] = cmd;
  serializeJson(debugDoc, Serial);
  Serial.println();

  if (strcmp(cmd, "PING") == 0) {
    directSendAck("PING", "Controller alive");
  }

  else if (strcmp(cmd, "HOME") == 0) {
    robotHomed = true;
    robotState = "IDLE";
    directSendAck("HOME", "HOME received");
    directSendStatus("HOMED", "Robot referenced");
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
    directSendStatus("ESTOPPED", "Emergency stop active");
  }

  else if (strcmp(cmd, "MOVE_ACT") == 0) {
    handleMoveActDirect(doc);
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
  Serial.begin(9600);
  Serial.setTimeout(100);

  initializeActuators();

  delay(500);

  robotHomed = true;
  robotState = "IDLE";

  directSendStatus("HOMED", "DIRECT FIRMWARE READY");
}

void loop() {
  if (!Serial.available()) {
    return;
  }

  String line = Serial.readStringUntil('\n');
  processDirectJsonLine(line);
}
