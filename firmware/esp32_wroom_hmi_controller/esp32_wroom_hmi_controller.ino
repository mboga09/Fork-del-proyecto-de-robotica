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
static const int Q1_FORWARD_US = 1700;
static const int Q1_REVERSE_US = 1300;
static const int HOME_SENSOR_ACTIVE_LEVEL = HIGH;

static const float SERVO_MIN_DEG = 0.0f;
static const float SERVO_MAX_DEG = 180.0f;
static const float MAX_Z_TIME_S = 120.0f;
static const float ROTARY_STEP_DEG = 0.5f;
static const uint32_t MOTION_PERIOD_MS = 25;
static const float TOOL_STEP_DEG = 1.0f;
static const uint32_t TOOL_PERIOD_MS = 15;
static const uint32_t TOOL_HOLD_MS = 1000;

static const float TOOL_HOME_DEG = 0.0f;
static const float TOOL_ASPIRATE_DEG = 180.0f;
static const float TOOL_DISPENSE_DEG = 0.0f;

static QueueHandle_t commandQueue;
static QueueHandle_t logQueue;

static volatile bool robotArmed = false;
static volatile bool motionBusy = false;
static volatile bool stopRequested = false;
static volatile bool estopRequested = false;

static float currentS2 = 45.0f;
static float currentS3 = 90.0f;
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
  writeServoAngle(CH_TOOL, currentTool);
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

void executeMove(const RobotCommand& command) {
  if (!robotArmed) {
    queueStatus("ERROR", "IDLE", "MOVE_ACT rejected: HOME required");
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
  if (q1HomeSensorActive() && command.zDir < 0) {
    stopQ1();
    queueStatus("IDLE", "IDLE", "Q1 home sensor active; negative Z command ignored");
    return;
  }

  motionBusy = true;
  stopRequested = false;
  queueStatus("MOVING", "MOVING", command.name);

  uint32_t startMs = millis();
  uint32_t zDurationMs = (uint32_t)(command.zTimeS * 1000.0f);
  bool zActive = command.zDir != 0 && zDurationMs > 0;
  setQ1(zActive ? command.zDir : 0);

  TickType_t lastWake = xTaskGetTickCount();
  while (true) {
    if (stopRequested || estopRequested) {
      stopQ1();
      motionBusy = false;
      queueStatus(estopRequested ? "ESTOPPED" : "STOPPED", estopRequested ? "ESTOPPED" : "STOPPED", "Motion interrupted");
      return;
    }

    currentS2 = stepToward(currentS2, command.s2Deg, ROTARY_STEP_DEG);
    currentS3 = stepToward(currentS3, command.s3Deg, ROTARY_STEP_DEG);
    writeServoAngle(CH_Q2, currentS2);
    writeServoAngle(CH_Q3, currentS3);

    if (zActive && command.zDir < 0 && q1HomeSensorActive()) {
      zActive = false;
      stopQ1();
      motionBusy = false;
      publishPosition(command.name);
      queueStatus("IDLE", "IDLE", "Q1 home sensor reached; Z stopped");
      return;
    }

    if (zActive && millis() - startMs >= zDurationMs) {
      zActive = false;
      stopQ1();
    }

    bool done = fabs(currentS2 - command.s2Deg) < 0.01f && fabs(currentS3 - command.s3Deg) < 0.01f && !zActive;
    if (done) break;
    vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(MOTION_PERIOD_MS));
  }

  stopQ1();
  motionBusy = false;
  publishPosition(command.name);
  queueStatus("IDLE", "IDLE", "Motion completed");
}

void executeTool(const char* name, float targetDeg) {
  if (!robotArmed) {
    queueStatus("ERROR", "IDLE", "Tool rejected: HOME required");
    return;
  }
  if (!validateAngle(targetDeg)) {
    queueStatus("ERROR", "IDLE", "Tool rejected: angle out of range");
    return;
  }
  motionBusy = true;
  stopRequested = false;
  queueStatus("MOVING", "MOVING", name);

  TickType_t lastWake = xTaskGetTickCount();
  while (true) {
    if (stopRequested || estopRequested) {
      motionBusy = false;
      queueStatus(estopRequested ? "ESTOPPED" : "STOPPED", estopRequested ? "ESTOPPED" : "STOPPED", "Tool interrupted");
      return;
    }
    currentTool = stepToward(currentTool, targetDeg, TOOL_STEP_DEG);
    writeServoAngle(CH_TOOL, currentTool);
    if (fabs(currentTool - targetDeg) < 0.01f) break;
    vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(TOOL_PERIOD_MS));
  }

  vTaskDelay(pdMS_TO_TICKS(TOOL_HOLD_MS));
  motionBusy = false;
  publishPosition(name);
  queueStatus("IDLE", "IDLE", "Tool completed");
}

void handleHome() {
  estopRequested = false;
  stopRequested = false;
  robotArmed = true;
  motionBusy = false;
  safeOutputs();

  if (q1HomeSensorActive()) {
    queueStatus("HOMED", "IDLE", "Q1 home sensor active; position set to home");
  } else {
    queueStatus("HOMED", "IDLE", "Controller armed; Q1 home sensor is not active");
  }
}

void motionTask(void* parameter) {
  RobotCommand command;
  for (;;) {
    if (xQueueReceive(commandQueue, &command, pdMS_TO_TICKS(20)) == pdTRUE) {
      if (strcmp(command.cmd, "PING") == 0) {
        queueStatus("READY", "IDLE", "ESP32 HMI controller alive");
      } else if (strcmp(command.cmd, "HOME") == 0 || strcmp(command.cmd, "ARM_TEST") == 0) {
        handleHome();
      } else if (strcmp(command.cmd, "MOVE_ACT") == 0) {
        executeMove(command);
      } else if (strcmp(command.cmd, "TOOL_ASPIRATE") == 0) {
        executeTool("TOOL_ASPIRATE", TOOL_ASPIRATE_DEG);
      } else if (strcmp(command.cmd, "TOOL_DISPENSE") == 0) {
        executeTool("TOOL_DISPENSE", TOOL_DISPENSE_DEG);
      } else if (strcmp(command.cmd, "TOOL_HOME") == 0) {
        executeTool("TOOL_HOME", TOOL_HOME_DEG);
      } else if (strcmp(command.cmd, "STOP") == 0) {
        stopRequested = true;
        stopQ1();
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
