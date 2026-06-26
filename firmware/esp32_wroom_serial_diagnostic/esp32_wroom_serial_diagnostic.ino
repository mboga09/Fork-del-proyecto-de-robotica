#include <Arduino.h>
#include <ArduinoJson.h>

// ---------------------------------------------------------
// ESP32 WROOM serial diagnostic firmware
// ---------------------------------------------------------
// Objetivo:
//   - Probar comunicación JSON desde la HMI.
//   - No mover ningún actuador real.
//   - Simular estados MOVING/HOMING/IDLE para verificar que Python no mande
//     el siguiente comando antes de que el ESP32 reporte finalización.
//   - Permitir que STOP/ESTOP interrumpan cualquier acción dummy.
//
// Protocolo:
//   115200 baud
//   UTF-8
//   Un objeto JSON por línea
// ---------------------------------------------------------

static const unsigned long BAUD_RATE = 115200;

static const unsigned long DUMMY_HOME_MS = 700;
static const unsigned long DUMMY_MOVE_MS = 150;
static const unsigned long DUMMY_TOOL_MS = 300;

static bool robotHomed = false;
static const char* robotState = "IDLE";

static bool operationActive = false;
static unsigned long operationEndsAtMs = 0;
static String activeOperation = "";

// ---------------------------------------------------------
// JSON helpers
// ---------------------------------------------------------

void sendJson(JsonDocument& doc) {
  serializeJson(doc, Serial);
  Serial.println();
}

void sendStatus(const char* status, const char* message) {
  StaticJsonDocument<256> doc;
  doc["type"] = "status";
  doc["status"] = status;
  doc["state"] = robotState;
  doc["homed"] = robotHomed;
  doc["message"] = message;
  sendJson(doc);
}

void sendAck(const char* cmd, const char* message) {
  StaticJsonDocument<256> doc;
  doc["type"] = "ack";
  doc["cmd"] = cmd;
  doc["ok"] = true;
  doc["state"] = robotState;
  doc["homed"] = robotHomed;
  doc["message"] = message;
  sendJson(doc);
}

void sendError(const char* cmd, const char* message) {
  StaticJsonDocument<256> doc;
  doc["type"] = "error";
  doc["cmd"] = cmd;
  doc["ok"] = false;
  doc["state"] = robotState;
  doc["homed"] = robotHomed;
  doc["message"] = message;
  sendJson(doc);
}

void sendDebugRaw(const String& line) {
  StaticJsonDocument<384> doc;
  doc["type"] = "status";
  doc["status"] = "DEBUG_RAW";
  doc["state"] = robotState;
  doc["homed"] = robotHomed;
  doc["message"] = line;
  sendJson(doc);
}

// ---------------------------------------------------------
// Dummy operation scheduler
// ---------------------------------------------------------

bool isBusy() {
  return operationActive;
}

void startDummyOperation(const char* operationName, unsigned long durationMs, const char* movingMessage) {
  operationActive = true;
  activeOperation = operationName;
  operationEndsAtMs = millis() + durationMs;
  robotState = "MOVING";
  sendStatus("MOVING", movingMessage);
}

void finishDummyOperation() {
  if (!operationActive) {
    return;
  }

  String finishedOperation = activeOperation;

  operationActive = false;
  activeOperation = "";
  robotState = "IDLE";

  if (finishedOperation == "HOME") {
    robotHomed = true;
    sendStatus("HOMED", "Dummy HOME completed");
    return;
  }

  sendStatus("IDLE", "Dummy operation completed");
}

void updateDummyOperation() {
  if (!operationActive) {
    return;
  }

  long remainingMs = (long)(operationEndsAtMs - millis());
  if (remainingMs <= 0) {
    finishDummyOperation();
  }
}

void cancelDummyOperation(const char* finalState, const char* message) {
  operationActive = false;
  activeOperation = "";
  robotState = finalState;
  sendStatus(finalState, message);
}

// ---------------------------------------------------------
// Command handlers
// ---------------------------------------------------------

void handlePing() {
  sendAck("PING", "ESP32 WROOM dummy alive");
}

void handleHome() {
  if (isBusy()) {
    sendError("HOME", "Busy: dummy operation already active");
    return;
  }

  robotHomed = false;
  sendAck("HOME", "Dummy HOME received");
  startDummyOperation("HOME", DUMMY_HOME_MS, "Dummy homing started");
}

void handleMoveAct(StaticJsonDocument<384>& doc) {
  if (isBusy()) {
    sendError("MOVE_ACT", "Busy: dummy operation already active");
    return;
  }

  if (!robotHomed) {
    sendError("MOVE_ACT", "Robot must be homed before MOVE_ACT");
    return;
  }

  if (!doc.containsKey("z_dir") ||
      !doc.containsKey("z_time_s") ||
      !doc.containsKey("s2_deg") ||
      !doc.containsKey("s3_deg")) {
    sendError("MOVE_ACT", "Missing actuator fields");
    return;
  }

  int zDir = doc["z_dir"];
  float zTimeS = doc["z_time_s"];
  float s2Deg = doc["s2_deg"];
  float s3Deg = doc["s3_deg"];

  if (zDir < -1 || zDir > 1) {
    sendError("MOVE_ACT", "Invalid z_dir");
    return;
  }

  if (zTimeS < 0.0f) {
    sendError("MOVE_ACT", "Invalid z_time_s");
    return;
  }

  if (s2Deg < 0.0f || s2Deg > 180.0f ||
      s3Deg < 0.0f || s3Deg > 180.0f) {
    sendError("MOVE_ACT", "Servo angle out of range");
    return;
  }

  StaticJsonDocument<256> ack;
  ack["type"] = "ack";
  ack["cmd"] = "MOVE_ACT";
  ack["ok"] = true;
  ack["state"] = robotState;
  ack["homed"] = robotHomed;
  ack["message"] = "Dummy MOVE_ACT accepted";
  ack["z_dir"] = zDir;
  ack["z_time_s"] = zTimeS;
  ack["s2_deg"] = s2Deg;
  ack["s3_deg"] = s3Deg;
  sendJson(ack);

  startDummyOperation("MOVE_ACT", DUMMY_MOVE_MS, "Dummy actuator movement started");
}

void handleToolAspirate() {
  if (isBusy()) {
    sendError("TOOL_ASPIRATE", "Busy: dummy operation already active");
    return;
  }

  if (!robotHomed) {
    sendError("TOOL_ASPIRATE", "Robot must be homed before tool command");
    return;
  }

  sendAck("TOOL_ASPIRATE", "Dummy aspirate accepted");
  startDummyOperation("TOOL_ASPIRATE", DUMMY_TOOL_MS, "Dummy tool aspirating");
}

void handleToolDispense() {
  if (isBusy()) {
    sendError("TOOL_DISPENSE", "Busy: dummy operation already active");
    return;
  }

  if (!robotHomed) {
    sendError("TOOL_DISPENSE", "Robot must be homed before tool command");
    return;
  }

  sendAck("TOOL_DISPENSE", "Dummy dispense accepted");
  startDummyOperation("TOOL_DISPENSE", DUMMY_TOOL_MS, "Dummy tool dispensing");
}

void handleStop() {
  sendAck("STOP", "STOP received; cancelling dummy operation");
  cancelDummyOperation("STOPPED", "Dummy operation stopped");
}

void handleEstop() {
  robotHomed = false;
  sendAck("ESTOP", "ESTOP received; cancelling dummy operation");
  cancelDummyOperation("ESTOPPED", "Dummy emergency stop active");
}

// ---------------------------------------------------------
// Parser
// ---------------------------------------------------------

void processJsonLine(String line) {
  line.trim();

  if (line.length() == 0) {
    return;
  }

  sendDebugRaw(line);

  StaticJsonDocument<384> doc;
  DeserializationError error = deserializeJson(doc, line);

  if (error) {
    sendError("UNKNOWN", error.c_str());
    return;
  }

  const char* cmd = doc["cmd"] | "";

  if (strcmp(cmd, "PING") == 0) {
    handlePing();
  } else if (strcmp(cmd, "HOME") == 0) {
    handleHome();
  } else if (strcmp(cmd, "MOVE_ACT") == 0) {
    handleMoveAct(doc);
  } else if (strcmp(cmd, "TOOL_ASPIRATE") == 0) {
    handleToolAspirate();
  } else if (strcmp(cmd, "TOOL_DISPENSE") == 0) {
    handleToolDispense();
  } else if (strcmp(cmd, "STOP") == 0) {
    handleStop();
  } else if (strcmp(cmd, "ESTOP") == 0) {
    handleEstop();
  } else {
    sendError(cmd, "Unknown command");
  }
}

// ---------------------------------------------------------
// Arduino setup / loop
// ---------------------------------------------------------

void setup() {
  Serial.begin(BAUD_RATE);
  Serial.setTimeout(50);

  delay(500);

  robotHomed = false;
  robotState = "IDLE";

  sendStatus("READY", "ESP32 WROOM dummy serial diagnostic ready @ 115200");
}

void loop() {
  while (Serial.available()) {
    String line = Serial.readStringUntil('\n');
    processJsonLine(line);
  }

  updateDummyOperation();
}
