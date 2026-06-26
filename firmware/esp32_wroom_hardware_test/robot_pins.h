#pragma once

// ESP32 WROOM pin configuration near final hardware layout.
// Arduino pin names D18/D19/D21/D22 map to GPIO18/GPIO19/GPIO21/GPIO22.

static const int PIN_Q1_Z_SCREW = 18;
static const int PIN_Q2_ARM_1 = 19;
static const int PIN_Q3_ARM_2 = 21;
static const int PIN_TOOL_SUCTION = 22;

// GPIO34 is input-only and has no internal pull-up/pull-down on ESP32.
// Use an external pull-up or pull-down resistor according to wiring.
static const int PIN_Q1_ESTOP_LIMIT = 34;
