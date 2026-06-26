#include "command_parser.h"

#include <Arduino.h>
#include <ArduinoJson.h>
#include <string.h>

#include "../../controller_state_machine/robot_state_machine/robot_state_machine.h"
#include "../serial_protocol/serial_protocol.h"
#include "../../actuator/actuator_controller.h"


// -----------------------------------------------------------------------------
// Forward declarations
// -----------------------------------------------------------------------------
static void handlePing();
static void handleHome();
static void handleStop();
static void handleEmergencyStop();
static void handleMoveDummy();
static void handleMoveAct(StaticJsonDocument<256>& doc);
static void handleToolAspirate();
static void handleToolDispense();


// -----------------------------------------------------------------------------
// Command handlers
// -----------------------------------------------------------------------------

static void handlePing() {
  sendAck("PING", true, "Controller alive");
}

static void handleHome() {
  if (!canStartHoming()) {
    sendCommandError("HOME", "Homing can only start from IDLE or STOPPED");
    return;
  }

  setIsHomed(true);

  sendAck("HOME", true, "HOME command received");

  enterState(STATE_HOMING);
  sendStatus("HOMING", "Homing started");

  // Para pruebas: home inmediato
  setIsHomed(true);
  enterState(STATE_IDLE);
  sendStatus("HOMED", "Homing completed. Robot is now idle and referenced.");
}
static void handleStop() {
  if (getCurrentState() == STATE_ESTOPPED) {
    sendCommandError("STOP", "Cannot stop because robot is already in ESTOPPED state");
    return;
  }

  sendAck("STOP", true, "STOP command received");

  stopZAxis();

  enterState(STATE_STOPPED);
  sendStatus("STOPPED", "Robot stopped by command");
}

static void handleEmergencyStop() {
  setIsHomed(false);

  sendAck("ESTOP", true, "Emergency stop command received");

  stopZAxis();

  enterState(STATE_ESTOPPED);
  sendStatus("ESTOPPED", "Emergency stop active");
}

static void handleMoveDummy() {
  if (!canStartMotion()) {
    sendCommandError("MOVE_DUMMY", "Motion requires IDLE state and completed homing");
    return;
  }

  sendAck("MOVE_DUMMY", true, "Dummy move command received");

  enterState(STATE_MOVING);
  sendStatus("MOVING", "Dummy movement started");

  enterState(STATE_IDLE);
  sendStatus("IDLE", "Dummy movement completed");
}

static void handleMoveAct(StaticJsonDocument<256>& doc) {
  sendStatus("DEBUG_MOVE_ACT", "Entered MOVE_ACT handler");

  //if (!canStartMotion()) {
    //sendCommandError("MOVE_ACT", "Motion requires IDLE state and completed homing");
    //return;
  //}

  if (!doc.containsKey("z_dir") ||
      !doc.containsKey("z_time_s") ||
      !doc.containsKey("s2_deg") ||
      !doc.containsKey("s3_deg")) {
    sendCommandError("MOVE_ACT", "Missing actuator fields");
    return;
  }

  int zDir = doc["z_dir"];
  float zTimeS = doc["z_time_s"];
  float s2Deg = doc["s2_deg"];
  float s3Deg = doc["s3_deg"];

  if (zDir < -1 || zDir > 1) {
    sendCommandError("MOVE_ACT", "Invalid z_dir");
    return;
  }

  if (zTimeS < 0.0f) {
    sendCommandError("MOVE_ACT", "Invalid z_time_s");
    return;
  }

  if (s2Deg < 0.0f || s2Deg > 180.0f ||
      s3Deg < 0.0f || s3Deg > 180.0f) {
    sendCommandError("MOVE_ACT", "Servo angle out of range");
    return;
  }

  sendAck("MOVE_ACT", true, "MOVE_ACT received");

  enterState(STATE_MOVING);
  sendStatus("MOVING", "Actuator movement started");

  moveActuators(zDir, zTimeS, s2Deg, s3Deg);

  enterState(STATE_IDLE);
  sendStatus("IDLE", "Actuator movement completed");
}

static void handleToolAspirate() {
  if (!canStartMotion()) {
    sendCommandError("TOOL_ASPIRATE", "Tool action requires IDLE state and completed homing");
    return;
  }

  sendAck("TOOL_ASPIRATE", true, "Tool aspirate command received");

  enterState(STATE_MOVING);
  sendStatus("MOVING", "Tool aspirating");

  toolAspirate();

  enterState(STATE_IDLE);
  sendStatus("IDLE", "Tool aspirate completed");
}

static void handleToolDispense() {
  if (!canStartMotion()) {
    sendCommandError("TOOL_DISPENSE", "Tool action requires IDLE state and completed homing");
    return;
  }

  sendAck("TOOL_DISPENSE", true, "Tool dispense command received");

  enterState(STATE_MOVING);
  sendStatus("MOVING", "Tool dispensing");

  toolDispense();

  enterState(STATE_IDLE);
  sendStatus("IDLE", "Tool dispense completed");
}


// -----------------------------------------------------------------------------
// Main parser entry point
// -----------------------------------------------------------------------------

void processJsonLine(String line) {
  line.trim();

  if (line.length() == 0) {
    return;
  }

  StaticJsonDocument<256> doc;
  DeserializationError error = deserializeJson(doc, line);

  if (error) {
    sendError("Invalid JSON");
    return;
  }

  if (!doc.containsKey("cmd")) {
    sendError("Missing cmd field");
    return;
  }

  const char* cmd = doc["cmd"];

  if (cmd == nullptr) {
    sendError("Invalid cmd field");
    return;
  }

  sendStatus("DEBUG_CMD", cmd);

  if (strcmp(cmd, "ESTOP") == 0) {
    handleEmergencyStop();
    return;
  }

  if (strcmp(cmd, "STOP") == 0) {
    handleStop();
    return;
  }

  if (strcmp(cmd, "PING") == 0) {
    handlePing();
  }
  else if (strcmp(cmd, "HOME") == 0) {
    handleHome();
  }
 
  else if (strcmp(cmd, "MOVE_DUMMY") == 0) {
    handleMoveDummy();
  }
  else if (strcmp(cmd, "MOVE_ACT") == 0) {
    handleMoveAct(doc);
  }
  else if (strcmp(cmd, "TOOL_ASPIRATE") == 0) {
    handleToolAspirate();
  }
  else if (strcmp(cmd, "TOOL_DISPENSE") == 0) {
    handleToolDispense();
  }
  else {
    sendCommandError(cmd, "Unknown command");
  }
}