#include "state_actions.h"

#include <Arduino.h>

#include "../../config.h"
#include "../controller_state_machine/robot_state_machine/robot_state_machine.h"
#include "../communication/serial_protocol/serial_protocol.h"

static void updateHoming() {
  if (getCurrentState() != STATE_HOMING) {
    return;
  }

  // Dummy condition for now.
  // Later, replace this with real homing logic:
  // if (limitSwitchTriggered()) { ... }
  if (millis() - getStateEnteredAtMs() >= DUMMY_HOMING_DURATION_MS) {
    setIsHomed(true);

    enterState(STATE_IDLE);
    sendStatus("HOMED", "Homing completed. Robot is now idle and referenced.");
  }
}

static void updateMoving() {
  if (getCurrentState() != STATE_MOVING) {
    return;
  }

  // Placeholder for future motion control.
  // Later this function will check whether target position was reached.
}

static void updateRoute() {
  if (getCurrentState() != STATE_ROUTE_RUNNING) {
    return;
  }

  // Placeholder for future route execution.
}

static void updateAction() {
  if (getCurrentState() != STATE_ACTION_RUNNING) {
    return;
  }

  // Placeholder for future pipette or tool actions.
}

void updateStateActions() {
  updateHoming();
  updateMoving();
  updateRoute();
  updateAction();
}