#include "robot_state_machine.h"

#include <Arduino.h>

static RobotState currentState = STATE_BOOTING;
static bool isHomed = false;
static unsigned long stateEnteredAtMs = 0;

void initializeStateMachine() {
  currentState = STATE_IDLE;
  isHomed = false;
  stateEnteredAtMs = millis();
}

RobotState getCurrentState() {
  return currentState;
}

const char* getCurrentStateString() {
  return stateToString(currentState);
}

const char* stateToString(RobotState state) {
  switch (state) {
    case STATE_BOOTING:
      return "BOOTING";
    case STATE_IDLE:
      return "IDLE";
    case STATE_HOMING:
      return "HOMING";
    case STATE_MOVING:
      return "MOVING";
    case STATE_ROUTE_RUNNING:
      return "ROUTE_RUNNING";
    case STATE_ACTION_RUNNING:
      return "ACTION_RUNNING";
    case STATE_STOPPED:
      return "STOPPED";
    case STATE_ERROR:
      return "ERROR";
    case STATE_ESTOPPED:
      return "ESTOPPED";
    default:
      return "UNKNOWN";
  }
}

bool getIsHomed() {
  return isHomed;
}

void setIsHomed(bool homed) {
  isHomed = homed;
}

unsigned long getStateEnteredAtMs() {
  return stateEnteredAtMs;
}

bool isBusy() {
  return currentState == STATE_HOMING ||
         currentState == STATE_MOVING ||
         currentState == STATE_ROUTE_RUNNING ||
         currentState == STATE_ACTION_RUNNING;
}

bool canStartHoming() {
  return currentState == STATE_IDLE ||
         currentState == STATE_STOPPED;
}

bool canStartMotion() {
  return currentState == STATE_IDLE && isHomed;
}

void enterState(RobotState newState) {
  currentState = newState;
  stateEnteredAtMs = millis();
}