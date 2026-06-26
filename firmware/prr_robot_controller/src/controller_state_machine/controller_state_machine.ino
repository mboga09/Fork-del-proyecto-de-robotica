#include "command_parser.h"
#include "robot_state_machine.h"
#include "serial_protocol.h"
#include "state_actions.h"

void setup() {
  initializeSerialProtocol();
  initializeStateMachine();

  sendStatus("IDLE", "ESP32-C6 state machine firmware ready");
}

void loop() {
  String line;

  if (readSerialLine(line)) {
    processJsonLine(line);
  }

  updateStateActions();
}