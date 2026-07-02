#include <Arduino.h>
#include <ArduinoJson.h>

static const int PIN_Q1_Z_SCREW = 18;
static const int PIN_Q2_ARM_1 = 19;
static const int PIN_Q3_ARM_2 = 21;
static const int PIN_TOOL_SERVO = 22;
static const int PIN_Q1_HOME_SENSOR = 34;

static const uint32_t BAUD_RATE = 115200;
static const uint32_t SERVO_FREQ_HZ = 50;
static const uint8_t SERVO_RES_BITS = 16;
static const uint32_t SERVO_PERIOD_US = 20000;
static const uint32_t SERVO_DUTY_MAX = (1UL << SERVO_RES_BITS) - 1;

static const uint8_t CH_Q1 = 0;
static const uint8_t CH_Q2 = 1;
static const uint8_t CH_Q3 = 2;
static const uint8_t CH_TOOL = 3;

static const int Q1_STOP_US = 1500;

// Extended symmetric pulses around stop for the continuous Z servo.
// These are intentionally beyond the common 1000-2000 us servo range.
// The usToDuty() helper clamps pulses to the ESP32 output range [500, 2500].
// Gravity/friction compensation stays in Python via directional Z speeds.
static const int Q1_FORWARD_US = 2500;
static const int Q1_REVERSE_US = 500;
static const int HOME_SENSOR_ACTIVE_LEVEL = HIGH;

static const uint32_t INITIAL_HOME_TIMEOUT_MS = 45000;
static const uint32_t ROUTE_HOME_HOLD_MS = 1000;

static const float SERVO_MIN_DEG = 0.0f;
static const float SERVO_MAX_DEG = 180.0f;
static const float MAX_Z_TIME_S = 120.0f;
static const float ROTARY_STEP_DEG = 0.5f;
static const uint32_t MOTION_PERIOD_MS = 25;

// Tool calibration for the new continuous-rotation servo:
//   Aspirate: servo.write(70) for 2800 ms, then stop at 90.
//   Dispense: servo.write(110) for 2800 ms, then stop at 90.
// The 3000 ms stop pauses from the calibration sketch are not part of the
// autonomous action; the firmware stops the servo immediately after each run.
static const float TOOL_STOP_DEG = 90.0f;
static const float TOOL_HOME_DEG = TOOL_STOP_DEG;
static const float TOOL_ASPIRATE_DEG = 70.0f;
static const float TOOL_DISPENSE_DEG = 110.0f;
static const uint32_t TOOL_RUN_MS = 2800;
static const uint32_t TOOL_STOP_SETTLE_MS = 100;

static const float HOME_S2_DEG = 0.0f;
static const float HOME_S3_DEG = 180.0f;

static QueueHandle_t commandQueue;
static QueueHandle_t logQueue;

static volatile bool robotArmed = false;
static volatile bool motionBusy = false;
static volatile bool stopRequested = false;
static volatile bool estopRequested = false;

static float currentS2 = HOME_S2_DEG;
static float currentS3 = HOME_S3_DEG;
static float currentTool = TOOL_HOME_DEG;

struct RobotCommand {
  char cmd[24];
  char name[48];
  int zDir;
  float zTimeS;
  float s2Deg;
  float s3Deg;
  float toolDeg;
};

struct LogLine {
  char line[384];
};

uint32_t usToDuty(int pulseUs) {
  pulseUs = constrain(pulseUs, 500, 2500);
  return (uint32_t)((uint64_t)pulseUs * SERVO_DUTY_MAX / SERVO_PERIOD_US);
}

int angleToUs(float angleDeg) {
  angleDeg = constrain(angleDeg, SERVO_MIN_DEG, SERVO_MAX_DEG);
  return (int)(500.0f + (angleDeg / 180.0f) * 2000.0f);
}

void writeServoUs(uint8_t channel, int pulseUs) {
  ledcWrite(channel, usToDuty(pulseUs));
}

void writeServoAngle(uint8_t channel, float angleDeg) {
  writeServoUs(channel, angleToUs(angleDeg));
}

bool q1HomeSensorActive() {
  return digitalRead(PIN_Q1_HOME_SENSOR) == HOME_SENSOR_ACTIVE_LEVEL;
}

void stopQ1() {
  writeServoUs(CH_Q1, Q1_STOP_US);
}

void setQ1(int direction) {
  if (direction > 0) {
    writeServoUs(CH_Q1, Q1_FORWARD_US);
  } else if (direction < 0) {
    writeServoUs(CH_Q1, Q1_REVERSE_US);
  } else {
    stopQ1();
  }
}

void stopTool() {
  currentTool = TOOL_STOP_DEG;
  writeServoAngle(CH_TOOL, currentTool);
}

void queueRaw(const char* json) {
  if (logQueue == nullptr) return;
  LogLine line;
  strncpy(line.line, json, sizeof(line.line) - 1);
  line.line[sizeof(line.line) - 1] = '\0';
  xQueueSend(logQueue, &line, 0);
}

void queueStatus(const char* status, const char* state, const char* message) {
  StaticJsonDocument<384> doc;
  doc["type"] = "status";
  doc["status"] = status;
  doc["state"] = state;
  doc["armed"] = robotArmed;
  doc["busy"] = motionBusy;
  doc["q1_home_sensor_active"] = q1HomeSensorActive();
  doc["s2_deg"] = currentS2;
  doc["s3_deg"] = currentS3;
  doc["tool_deg"] = currentTool;
  doc["message"] = message;
  char out[384];
  serializeJson(doc, out, sizeof(out));
  queueRaw(out);
}

void sendDoc(JsonDocument& doc) {
  serializeJson(doc, Serial);
  Serial.println();
}

void sendAck(const char* cmd, const char* message) {
  StaticJsonDocument<256> doc;
  doc["type"] = "ack";
  doc["cmd"] = cmd;
  doc["ok"] = true;
  doc["armed"] = robotArmed;
  doc["busy"] = motionBusy;
  doc["message"] = message;
  sendDoc(doc);
}

void sendError(const char* cmd, const char* message) {
  StaticJsonDocument<256> doc;
  doc["type"] = "error";
  doc["cmd"] = cmd;
  doc["ok"] = false;
  doc["armed"] = robotArmed;
  doc["busy"] = motionBusy;
  doc["message"] = message;
  sendDoc(doc);
}

void sendDebugRaw(const String& line) {
  StaticJsonDocument<384> doc;
  doc["type"] = "status";
  doc["status"] = "DEBUG_RAW";
  doc["state"] = motionBusy ? "MOVING" : "IDLE";
  doc["armed"] = robotArmed;
  doc["busy"] = motionBusy;
  doc["message"] = line;
  sendDoc(doc);
}

void safeOutputs() {
  stopQ1();
  writeServoAngle(CH_Q2, currentS2);
  writeServoAngle(CH_Q3, currentS3);
  stopTool();
}

void enterEStop() {
  motionBusy = false;
  robotArmed = false;
  stopRequested = true;
  estopRequested = true;
  safeOutputs();
  queueStatus("ESTOPPED", "ESTOPPED", "ESTOP command received; outputs stopped");
}

void IRAM_ATTR onHomeSensorChange() {
}

bool validateAngle(float angle) {
  return angle >= SERVO_MIN_DEG && angle <= SERVO_MAX_DEG;
}

float stepToward(float current, float target, float stepDeg) {
  if (fabs(current - target) <= stepDeg) return target;
  return current + (target > current ? stepDeg : -stepDeg);
}

void publishPosition(const char* name) {
  StaticJsonDocument<384> doc;
  doc["type"] = "status";
  doc["status"] = "POSITION";
  doc["state"] = "IDLE";
  doc["armed"] = robotArmed;
  doc["busy"] = motionBusy;
  doc["name"] = name;
  doc["s2_deg"] = currentS2;
  doc["s3_deg"] = currentS3;
  doc["tool_deg"] = currentTool;
  doc["q1_home_sensor_active"] = q1HomeSensorActive();
  char out[384];
  serializeJson(doc, out, sizeof(out));
  queueRaw(out);
}

void executeInitialHome() {
  if (estopRequested) {
    enterEStop();
    return;
  }

  motionBusy = true;
  robotArmed = false;
  stopRequested = false;

  // Initial homing is Z-only. Do not rewrite S2, S3 or tool outputs here.
  // Route Home is responsible for moving the arm to HOME after Z is calibrated.
  stopQ1();
  stopTool();
  queueStatus("HOMING", "HOMING", "Initial Z-only homing started");

  uint32_t startMs = millis();
  if (!q1HomeSensorActive()) {
    setQ1(-1);
  }

  while (!q1HomeSensorActive()) {
    if (stopRequested || estopRequested) {
      stopQ1();
      stopTool();
      motionBusy = false;
      queueStatus(estopRequested ? "ESTOPPED" : "STOPPED", estopRequested ? "ESTOPPED" : "STOPPED", "Initial Z homing interrupted");
      return;
    }

    if (millis() - startMs >= INITIAL_HOME_TIMEOUT_MS) {
      stopQ1();
      stopTool();
      motionBusy = false;
      queueStatus("ERROR", "IDLE", "Initial Z homing timeout before limit switch");
      return;
    }

    vTaskDelay(pdMS_TO_TICKS(10));
  }

  stopQ1();
  stopTool();
  motionBusy = false;
  robotArmed = true;
  publishPosition("INITIAL_Z_HOME");
  queueStatus("Z_HOMED", "IDLE", "Initial Z homing complete; only d1 calibrated at limit switch");
}

void executeMove(const RobotCommand& command) {
  if (!robotArmed) {
    queueStatus("ERROR", "IDLE", "MOVE_ACT rejected: Initial Z Homing required");
    return;
  }
  if (estopRequested) {
    enterEStop();
    return;
  }
  if (!validateAngle(command.s2Deg) || !validateAngle(command.s3Deg)) {
    queueStatus("ERROR", "IDLE", "MOVE_ACT rejected: servo angle out of range");
    return;
  }
  if (command.zDir < -1 || command.zDir > 1 || command.zTimeS < 0.0f || command.zTimeS > MAX_Z_TIME_S) {
    queueStatus("ERROR", "IDLE", "MOVE_ACT rejected: unsafe z_time_s");
    return;
  }

  motionBusy = true;
  stopRequested = false;

  int effectiveZDir = command.zDir;
  if (q1HomeSensorActive() && effectiveZDir < 0) {
    effectiveZDir = 0;
    stopQ1();
    queueStatus("Z_LIMIT", "MOVING", "Q1 limit switch active; negative Z overwritten to 0");
  }

  queueStatus("MOVING", "MOVING", command.name);

  // Safety rule for workspace clearance: every MOVE_ACT aligns J2/J3 first
  // with Z stopped. Only after the planar arm reaches the target does Q1 move.
  stopQ1();
  TickType_t lastWake = xTaskGetTickCount();
  while (true) {
    if (stopRequested || estopRequested) {
      stopQ1();
      motionBusy = false;
      queueStatus(estopRequested ? "ESTOPPED" : "STOPPED", estopRequested ? "ESTOPPED" : "STOPPED", "Motion interrupted while aligning J2/J3");
      return;
    }

    currentS2 = stepToward(currentS2, command.s2Deg, ROTARY_STEP_DEG);
    currentS3 = stepToward(currentS3, command.s3Deg, ROTARY_STEP_DEG);
    writeServoAngle(CH_Q2, currentS2);
    writeServoAngle(CH_Q3, currentS3);

    bool rotaryDone = fabs(currentS2 - command.s2Deg) < 0.01f && fabs(currentS3 - command.s3Deg) < 0.01f;
    if (rotaryDone) break;
    vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(MOTION_PERIOD_MS));
  }

  uint32_t startMs = millis();
  uint32_t zDurationMs = (uint32_t)(command.zTimeS * 1000.0f);
  bool zActive = effectiveZDir != 0 && zDurationMs > 0;
  setQ1(zActive ? effectiveZDir : 0);

  while (zActive) {
    if (stopRequested || estopRequested) {
      stopQ1();
      motionBusy = false;
      queueStatus(estopRequested ? "ESTOPPED" : "STOPPED", estopRequested ? "ESTOPPED" : "STOPPED", "Motion interrupted during Z move");
      return;
    }

    writeServoAngle(CH_Q2, currentS2);
    writeServoAngle(CH_Q3, currentS3);

    if (effectiveZDir < 0 && q1HomeSensorActive()) {
      zActive = false;
      effectiveZDir = 0;
      stopQ1();
      queueStatus("Z_LIMIT", "MOVING", "Q1 limit switch reached; Z overwritten to 0");
    }

    if (zActive && millis() - startMs >= zDurationMs) {
      zActive = false;
      stopQ1();
    }

    if (zActive) {
      vTaskDelay(pdMS_TO_TICKS(10));
    }
  }

  stopQ1();
  motionBusy = false;
  publishPosition(command.name);
  queueStatus("IDLE", "IDLE", "Motion completed");
}

void executeToolStop(const char* name) {
  if (!robotArmed) {
    queueStatus("ERROR", "IDLE", "Tool rejected: Initial Z Homing required");
    return;
  }

  motionBusy = true;
  stopRequested = false;
  queueStatus("MOVING", "MOVING", name);

  stopTool();
  vTaskDelay(pdMS_TO_TICKS(TOOL_STOP_SETTLE_MS));

  motionBusy = false;
  publishPosition(name);
  queueStatus("IDLE", "IDLE", "Tool stopped");
}

void executeTool(const char* name, float runDeg) {
  if (!robotArmed) {
    queueStatus("ERROR", "IDLE", "Tool rejected: Initial Z Homing required");
    return;
  }
  if (!validateAngle(runDeg)) {
    queueStatus("ERROR", "IDLE", "Tool rejected: speed command out of range");
    return;
  }

  motionBusy = true;
  stopRequested = false;
  queueStatus("MOVING", "MOVING", name);

  currentTool = runDeg;
  writeServoAngle(CH_TOOL, currentTool);

  uint32_t startMs = millis();
  while (millis() - startMs < TOOL_RUN_MS) {
    if (stopRequested || estopRequested) {
      stopTool();
      motionBusy = false;
      queueStatus(estopRequested ? "ESTOPPED" : "STOPPED", estopRequested ? "ESTOPPED" : "STOPPED", "Tool interrupted");
      return;
    }

    vTaskDelay(pdMS_TO_TICKS(10));
  }

  stopTool();
  vTaskDelay(pdMS_TO_TICKS(TOOL_STOP_SETTLE_MS));
  motionBusy = false;
  publishPosition(name);
  queueStatus("IDLE", "IDLE", "Tool continuous-rotation action completed");
}

void handleRouteHome() {
  estopRequested = false;
  stopRequested = false;
  robotArmed = true;
  motionBusy = true;

  currentS2 = HOME_S2_DEG;
  currentS3 = HOME_S3_DEG;
  stopTool();
  safeOutputs();

  queueStatus("MOVING", "MOVING", "Route home hold against mechanical stop");

  uint32_t startMs = millis();
  while (millis() - startMs < ROUTE_HOME_HOLD_MS) {
    if (stopRequested || estopRequested) {
      if (estopRequested) {
        enterEStop();
        return;
      }
      motionBusy = false;
      safeOutputs();
      queueStatus("STOPPED", "STOPPED", "Route home hold interrupted");
      return;
    }

    writeServoAngle(CH_Q2, HOME_S2_DEG);
    writeServoAngle(CH_Q3, HOME_S3_DEG);
    stopTool();
    vTaskDelay(pdMS_TO_TICKS(20));
  }

  motionBusy = false;
  publishPosition("ROUTE_HOME");
  queueStatus("HOMED", "IDLE", "Route home held 1s for mechanical stop correction; Z position unchanged");
}

void motionTask(void* parameter) {
  RobotCommand command;
  for (;;) {
    if (xQueueReceive(commandQueue, &command, pdMS_TO_TICKS(20)) == pdTRUE) {
      if (strcmp(command.cmd, "PING") == 0) {
        queueStatus("READY", "IDLE", "ESP32 HMI controller alive");
      } else if (strcmp(command.cmd, "HOME_Z") == 0 || strcmp(command.cmd, "INITIAL_HOME") == 0) {
        executeInitialHome();
      } else if (strcmp(command.cmd, "HOME") == 0 || strcmp(command.cmd, "ARM_TEST") == 0) {
        handleRouteHome();
      } else if (strcmp(command.cmd, "MOVE_ACT") == 0) {
        executeMove(command);
      } else if (strcmp(command.cmd, "TOOL_ASPIRATE") == 0) {
        executeTool("TOOL_ASPIRATE", TOOL_ASPIRATE_DEG);
      } else if (strcmp(command.cmd, "TOOL_DISPENSE") == 0) {
        executeTool("TOOL_DISPENSE", TOOL_DISPENSE_DEG);
      } else if (strcmp(command.cmd, "TOOL_HOME") == 0) {
        executeToolStop("TOOL_HOME");
      } else if (strcmp(command.cmd, "STOP") == 0) {
        stopRequested = true;
        stopQ1();
        stopTool();
        motionBusy = false;
        queueStatus("STOPPED", "STOPPED", "STOP received");
      } else if (strcmp(command.cmd, "ESTOP") == 0) {
        enterEStop();
      } else {
        queueStatus("ERROR", "IDLE", "Unknown command");
      }
    }
  }
}

void parseLine(String line) {
  line.trim();
  if (line.length() == 0) return;
  sendDebugRaw(line);

  StaticJsonDocument<512> doc;
  DeserializationError err = deserializeJson(doc, line);
  if (err) {
    sendError("UNKNOWN", err.c_str());
    return;
  }

  RobotCommand c = {};
  const char* cmd = doc["cmd"] | "";
  const char* name = doc["name"] | cmd;
  strncpy(c.cmd, cmd, sizeof(c.cmd) - 1);
  strncpy(c.name, name, sizeof(c.name) - 1);
  c.zDir = doc["z_dir"] | 0;
  c.zTimeS = doc["z_time_s"] | 0.0f;
  c.s2Deg = doc["s2_deg"] | currentS2;
  c.s3Deg = doc["s3_deg"] | currentS3;
  c.toolDeg = doc["tool_deg"] | currentTool;

  if (strlen(c.cmd) == 0) {
    sendError("UNKNOWN", "Missing cmd");
    return;
  }
  if (strcmp(c.cmd, "STOP") == 0) stopRequested = true;
  if (strcmp(c.cmd, "ESTOP") == 0) estopRequested = true;

  if (xQueueSend(commandQueue, &c, pdMS_TO_TICKS(20)) != pdTRUE) {
    sendError(c.cmd, "Command queue full");
    return;
  }
  sendAck(c.cmd, "Command queued");
}

void serialTask(void* parameter) {
  String line;
  line.reserve(512);
  LogLine logLine;
  for (;;) {
    while (Serial.available()) {
      char ch = (char)Serial.read();
      if (ch == '\n') {
        parseLine(line);
        line = "";
      } else if (ch != '\r') {
        line += ch;
        if (line.length() > 511) {
          line = "";
          sendError("UNKNOWN", "Serial line too long");
        }
      }
    }
    while (xQueueReceive(logQueue, &logLine, 0) == pdTRUE) {
      Serial.println(logLine.line);
    }
    vTaskDelay(pdMS_TO_TICKS(5));
  }
}

void setup() {
  Serial.begin(BAUD_RATE);
  Serial.setTimeout(10);
  delay(500);

  pinMode(PIN_Q1_HOME_SENSOR, INPUT);
  ledcSetup(CH_Q1, SERVO_FREQ_HZ, SERVO_RES_BITS);
  ledcSetup(CH_Q2, SERVO_FREQ_HZ, SERVO_RES_BITS);
  ledcSetup(CH_Q3, SERVO_FREQ_HZ, SERVO_RES_BITS);
  ledcSetup(CH_TOOL, SERVO_FREQ_HZ, SERVO_RES_BITS);
  ledcAttachPin(PIN_Q1_Z_SCREW, CH_Q1);
  ledcAttachPin(PIN_Q2_ARM_1, CH_Q2);
  ledcAttachPin(PIN_Q3_ARM_2, CH_Q3);
  ledcAttachPin(PIN_TOOL_SERVO, CH_TOOL);
  safeOutputs();

  commandQueue = xQueueCreate(12, sizeof(RobotCommand));
  logQueue = xQueueCreate(24, sizeof(LogLine));
  attachInterrupt(digitalPinToInterrupt(PIN_Q1_HOME_SENSOR), onHomeSensorChange, CHANGE);
  xTaskCreatePinnedToCore(serialTask, "SerialTask", 8192, nullptr, 2, nullptr, 0);
  xTaskCreatePinnedToCore(motionTask, "MotionTask", 8192, nullptr, 3, nullptr, 1);
  queueStatus("READY", "IDLE", "ESP32 HMI controller ready @ 115200");
}

void loop() {
  vTaskDelay(pdMS_TO_TICKS(1000));
}
