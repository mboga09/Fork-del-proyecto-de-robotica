#pragma once

#include <Arduino.h>

enum RobotState {
  STATE_BOOTING,
  STATE_IDLE,
  STATE_HOMING,
  STATE_MOVING,
  STATE_ROUTE_RUNNING,
  STATE_ACTION_RUNNING,
  STATE_STOPPED,
  STATE_ERROR,
  STATE_ESTOPPED
};

void initializeStateMachine();

RobotState getCurrentState();
const char* getCurrentStateString();
const char* stateToString(RobotState state);

bool getIsHomed();
void setIsHomed(bool homed);

unsigned long getStateEnteredAtMs();

bool isBusy();
bool canStartHoming();
bool canStartMotion();

void enterState(RobotState newState);