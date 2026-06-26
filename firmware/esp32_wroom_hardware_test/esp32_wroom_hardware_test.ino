#include <Arduino.h>
#include <ArduinoJson.h>

#include "../common/robot_pins.h"

// ---------------------------------------------------------
// ESP32 WROOM hardware test firmware
// ---------------------------------------------------------
// Objetivos:
//   - Probar cableado y direccionalidad sin depender de la HMI.
//   - Mantener el eje Z en modo seguro: por defecto no se mueve.
//   - Mover q2/q3 lento para verificar offsets y sentido.
//   - Mover q1 solo con tiempos cortos si se solicita explicitamente.
//   - Usar dos tareas FreeRTOS fijadas a core:
//       Core 0: SerialTask, parser JSON, logging.
//       Core 1: MotionTask, PWM y estado del controlador.
//
// Protocolo:
//   115200 baud
//   Un objeto JSON por linea
// ---------------------------------------------------------

static const uint32_t BAUD_RATE = 115200;

// PWM 50 Hz para servos.
static const uint32_t SERVO_FREQ_HZ = 50;
static const uint8_t SERVO_RES_BITS = 16;
static const uint32_t SERVO_PERIOD_US = 20000;
static const uint32_t SERVO_DUTY_MAX = (1UL << SERVO_RES_BITS) - 1;

static const uint8_t CH_Q1 = 0;
static const uint8_t CH_Q2 = 1;
static const uint8_t CH_Q3 = 2;

// Ajustes iniciales seguros. Modificables en runtime con CONFIG.
static volatile float q2TrimDeg = 0.0f;
static volatile float q3TrimDeg = 0.0f;
static volatile int q1StopUs = 1500;
static volatile int q1ForwardUs = 1700;
static volatile int q1ReverseUs = 1300;

// Limites de seguridad del test.
static const float SERVO_MIN_DEG = 0.0f;
static const float SERVO_MAX_DEG = 180.0f;
static const float MAX_Z_TEST_TIME_S = 2.0f;

// Movimiento rotativo lento.
static const float ROTARY_STEP_DEG = 0.5f;
static const uint32_t MOTION_PERIOD_MS = 25;

// GPIO34 requiere resistencia externa. Ajustar si el switch es activo alto.
static const int ESTOP_ACTIVE_LEVEL = LOW;

static volatile bool robotArmed = false;
static volatile bool motionBusy = false;
static volatile bool stopRequested = false;
static volatile bool estopRequested = false;

static float currentServo2Deg = 45.0f;
static float currentServo3Deg = 90.0f;

struct RobotCommand {
  char cmd[24];
  char name[40];
  int zDir;
  float zTimeS;
  float s2Deg;
  float s3Deg;
  float q2Trim;
  float q3Trim;
  int q1Stop;
  int q1Forward;
  int q1Reverse;
};

static QueueHandle_t commandQueue;
static QueueHandle_t logQueue;

struct LogLine {
  char line[384];
};

// ---------------------------------------------------------
// Logging helpers
// ---------------------------------------------------------

void queueRawJson(const char* json) {
  if (logQueue == nullptr) {
    return;
  }

  LogLine msg;
  strncpy(msg.line, json, sizeof(msg.line) - 1);
  msg.line[sizeof(msg.line) - 1] = '\0';
  xQueueSend(logQueue, &msg, 0);
}

void queueStatus(const char* status, const char* state, const char* message) {
  StaticJsonDocument<384> doc;
  doc["type"] = "status";
  doc["status"] = status;
  doc["state"] = state;
  doc["armed"] = robotArmed;
  doc["busy"] = motionBusy;
  doc["message"] = message;

  char out[384];
  serializeJson(doc, out, sizeof(out));
  queueRawJson(out);
}

void sendJsonNow(JsonDocument& doc) {
  serializeJson(doc, Serial);
  Serial.println();
}

void sendAckNow(const char* cmd, const char* message) {
  StaticJsonDocument<384> doc;
  doc["type"] = "ack";
  doc["cmd"] = cmd;
  doc["ok"] = true;
  doc["armed"] = robotArmed;
  doc["busy"] = motionBusy;
  doc["message"] = message;
  sendJsonNow(doc);
}

void sendErrorNow(const char* cmd, const char* message) {
  StaticJsonDocument<384> doc;
  doc["type"] = "error";
  doc["cmd"] = cmd;
  doc["ok"] = false;
  doc["armed"] = robotArmed;
  doc["busy"] = motionBusy;
  doc["message"] = message;
  sendJsonNow(doc);
}

void sendDebugRawNow(const String& line) {
  StaticJsonDocument<384> doc;
  doc["type"] = "status";
  doc["status"] = "DEBUG_RAW";
  doc["state"] = motionBusy ? "MOVING" : "IDLE";
  doc["armed"] = robotArmed;
  doc["busy"] = motionBusy;
  doc["message"] = line;
  sendJsonNow(doc);
}

// ---------------------------------------------------------
// PWM helpers
// ---------------------------------------------------------

uint32_t usToDuty(int pulseUs) {
  pulseUs = constrain(pulseUs, 500, 2500);
  return (uint32_t)((uint64_t)pulseUs * SERVO_DUTY_MAX / SERVO_PERIOD_US);
}

int angleToUs(float angleDeg) {
  angleDeg = constrain(angleDeg, 0.0f, 180.0f);
  return (int)(500.0f + (angleDeg / 180.0f) * 2000.0f);
}

void writeServoUs(uint8_t channel, int pulseUs) {
  ledcWrite(channel, usToDuty(pulseUs));
}

void writeServoAngle(uint8_t channel, float angleDeg) {
  writeServoUs(channel, angleToUs(angleDeg));
}

void stopQ1() {
  writeServoUs(CH_Q1, q1StopUs);
}

void setQ1Direction(int zDir) {
  if (zDir > 0) {
    writeServoUs(CH_Q1, q1ForwardUs);
  } else if (zDir < 0) {
    writeServoUs(CH_Q1, q1ReverseUs);
  } else {
    stopQ1();
  }
}

bool estopIsActive() {
  return digitalRead(PIN_Q1_ESTOP_LIMIT) == ESTOP_ACTIVE_LEVEL;
}

void IRAM_ATTR onEstopChange() {
  if (digitalRead(PIN_Q1_ESTOP_LIMIT) == ESTOP_ACTIVE_LEVEL) {
    estopRequested = true;
    stopRequested = true;
  }
}

void applySafeOutputs() {
  stopQ1();
  writeServoAngle(CH_Q2, currentServo2Deg);
  writeServoAngle(CH_Q3, currentServo3Deg);
  digitalWrite(PIN_TOOL_SUCTION, LOW);
}

// ---------------------------------------------------------
// Motion execution
// ---------------------------------------------------------

float stepTowards(float current, float target, float step) {
  if (fabs(target - current) <= step) {
    return target;
  }
  return current + (target > current ? step : -step);
}

void publishPosition(const char* name) {
  StaticJsonDocument<384> doc;
  doc["type"] = "status";
  doc["status"] = "POSITION";
  doc["state"] = motionBusy ? "MOVING" : "IDLE";
  doc["armed"] = robotArmed;
  doc["busy"] = motionBusy;
  doc["name"] = name;
  doc["s2_deg"] = currentServo2Deg;
  doc["s3_deg"] = currentServo3Deg;
  doc["q2_trim_deg"] = q2TrimDeg;
  doc["q3_trim_deg"] = q3TrimDeg;

  char out[384];
  serializeJson(doc, out, sizeof(out));
  queueRawJson(out);
}

void enterEstop() {
  motionBusy = false;
  robotArmed = false;
  stopRequested = true;
  applySafeOutputs();
  queueStatus("ESTOPPED", "ESTOPPED", "Q1 emergency limit active; outputs set safe");
}

void executeMoveAct(const RobotCommand& command) {
  if (!robotArmed) {
    queueStatus("ERROR", "IDLE", "MOVE_ACT rejected: ARM_TEST or HOME required");
    return;
  }

  if (estopIsActive() || estopRequested) {
    enterEstop();
    return;
  }

  float targetS2 = command.s2Deg + q2TrimDeg;
  float targetS3 = command.s3Deg + q3TrimDeg;

  if (targetS2 < SERVO_MIN_DEG || targetS2 > SERVO_MAX_DEG ||
      targetS3 < SERVO_MIN_DEG || targetS3 > SERVO_MAX_DEG) {
    queueStatus("ERROR", "IDLE", "MOVE_ACT rejected: servo target plus trim out of range");
    return;
  }

  if (command.zDir < -1 || command.zDir > 1 || command.zTimeS < 0.0f || command.zTimeS > MAX_Z_TEST_TIME_S) {
    queueStatus("ERROR", "IDLE", "MOVE_ACT rejected: unsafe Z request for hardware test");
    return;
  }

  motionBusy = true;
  stopRequested = false;
  queueStatus("MOVING", "MOVING", command.name);

  const uint32_t startedMs = millis();
  const uint32_t zDurationMs = (uint32_t)(command.zTimeS * 1000.0f);
  bool zActive = (command.zDir != 0 && zDurationMs > 0);

  if (zActive) {
    setQ1Direction(command.zDir);
  } else {
    stopQ1();
  }

  TickType_t lastWake = xTaskGetTickCount();
  while (true) {
    if (stopRequested) {
      stopQ1();
      motionBusy = false;
      queueStatus(estopRequested ? "ESTOPPED" : "STOPPED", estopRequested ? "ESTOPPED" : "STOPPED", "Motion interrupted");
      return;
    }

    if (estopIsActive()) {
      estopRequested = true;
      enterEstop();
      return;
    }

    currentServo2Deg = stepTowards(currentServo2Deg, targetS2, ROTARY_STEP_DEG);
    currentServo3Deg = stepTowards(currentServo3Deg, targetS3, ROTARY_STEP_DEG);

    writeServoAngle(CH_Q2, currentServo2Deg);
    writeServoAngle(CH_Q3, currentServo3Deg);

    if (zActive && (millis() - startedMs >= zDurationMs)) {
      zActive = false;
      stopQ1();
    }

    bool rotDone = fabs(currentServo2Deg - targetS2) < 0.01f && fabs(currentServo3Deg - targetS3) < 0.01f;
    if (rotDone && !zActive) {
      break;
    }

    vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(MOTION_PERIOD_MS));
  }

  stopQ1();
  motionBusy = false;
  publishPosition(command.name);
  queueStatus("IDLE", "IDLE", "Motion completed");
}

void executeToolPulse(const char* name) {
  if (!robotArmed) {
    queueStatus("ERROR", "IDLE", "Tool rejected: ARM_TEST or HOME required");
    return;
  }

  motionBusy = true;
  queueStatus("MOVING", "MOVING", name);
  digitalWrite(PIN_TOOL_SUCTION, HIGH);
  vTaskDelay(pdMS_TO_TICKS(300));
  digitalWrite(PIN_TOOL_SUCTION, LOW);
  motionBusy = false;
  queueStatus("IDLE", "IDLE", "Tool pulse completed");
}

void applyConfig(const RobotCommand& command) {
  q2TrimDeg = command.q2Trim;
  q3TrimDeg = command.q3Trim;
  q1StopUs = command.q1Stop;
  q1ForwardUs = command.q1Forward;
  q1ReverseUs = command.q1Reverse;
  stopQ1();
  queueStatus("CONFIG", "IDLE", "Runtime trims updated");
}

void motionTask(void* parameter) {
  RobotCommand command;

  for (;;) {
    if (estopIsActive() && !estopRequested) {
      estopRequested = true;
      enterEstop();
    }

    if (xQueueReceive(commandQueue, &command, pdMS_TO_TICKS(20)) == pdTRUE) {
      if (strcmp(command.cmd, "PING") == 0) {
        queueStatus("READY", "IDLE", "ESP32 WROOM hardware test alive");
      } else if (strcmp(command.cmd, "ARM_TEST") == 0 || strcmp(command.cmd, "HOME") == 0) {
        estopRequested = false;
        stopRequested = false;
        robotArmed = true;
        motionBusy = false;
        applySafeOutputs();
        queueStatus("HOMED", "IDLE", "Hardware test armed without Z homing motion");
      } else if (strcmp(command.cmd, "MOVE_ACT") == 0) {
        executeMoveAct(command);
      } else if (strcmp(command.cmd, "TOOL_ASPIRATE") == 0 || strcmp(command.cmd, "TOOL_DISPENSE") == 0) {
        executeToolPulse(command.cmd);
      } else if (strcmp(command.cmd, "CONFIG") == 0) {
        applyConfig(command);
      } else if (strcmp(command.cmd, "STOP") == 0) {
        stopRequested = true;
        stopQ1();
        motionBusy = false;
        queueStatus("STOPPED", "STOPPED", "STOP received");
      } else if (strcmp(command.cmd, "ESTOP") == 0) {
        estopRequested = true;
        enterEstop();
      }
    }
  }
}

// ---------------------------------------------------------
// Serial parsing
// ---------------------------------------------------------

bool queueCommand(const RobotCommand& command) {
  return xQueueSend(commandQueue, &command, pdMS_TO_TICKS(20)) == pdTRUE;
}

void loadString(char* dst, size_t dstSize, const char* value) {
  if (value == nullptr) {
    dst[0] = '\0';
    return;
  }
  strncpy(dst, value, dstSize - 1);
  dst[dstSize - 1] = '\0';
}

void handleLine(String line) {
  line.trim();
  if (line.length() == 0) {
    return;
  }

  sendDebugRawNow(line);

  StaticJsonDocument<512> doc;
  DeserializationError error = deserializeJson(doc, line);
  if (error) {
    sendErrorNow("UNKNOWN", error.c_str());
    return;
  }

  RobotCommand command = {};
  const char* cmd = doc["cmd"] | "";
  const char* name = doc["name"] | cmd;
  loadString(command.cmd, sizeof(command.cmd), cmd);
  loadString(command.name, sizeof(command.name), name);

  command.zDir = doc["z_dir"] | 0;
  command.zTimeS = doc["z_time_s"] | 0.0f;
  command.s2Deg = doc["s2_deg"] | currentServo2Deg;
  command.s3Deg = doc["s3_deg"] | currentServo3Deg;

  command.q2Trim = doc["q2_trim_deg"] | q2TrimDeg;
  command.q3Trim = doc["q3_trim_deg"] | q3TrimDeg;
  command.q1Stop = doc["q1_stop_us"] | q1StopUs;
  command.q1Forward = doc["q1_forward_us"] | q1ForwardUs;
  command.q1Reverse = doc["q1_reverse_us"] | q1ReverseUs;

  if (strlen(command.cmd) == 0) {
    sendErrorNow("UNKNOWN", "Missing cmd");
    return;
  }

  if (strcmp(command.cmd, "MOVE_ACT") == 0) {
    if (command.s2Deg < SERVO_MIN_DEG || command.s2Deg > SERVO_MAX_DEG ||
        command.s3Deg < SERVO_MIN_DEG || command.s3Deg > SERVO_MAX_DEG) {
      sendErrorNow(command.cmd, "Servo command out of range before trim");
      return;
    }
  }

  if (!queueCommand(command)) {
    sendErrorNow(command.cmd, "Command queue full");
    return;
  }

  sendAckNow(command.cmd, "Command queued");
}

void flushLogs() {
  LogLine msg;
  while (xQueueReceive(logQueue, &msg, 0) == pdTRUE) {
    Serial.println(msg.line);
  }
}

void serialTask(void* parameter) {
  String line;
  line.reserve(512);

  for (;;) {
    while (Serial.available()) {
      char c = (char)Serial.read();
      if (c == '\n') {
        handleLine(line);
        line = "";
      } else if (c != '\r') {
        line += c;
        if (line.length() > 511) {
          line = "";
          sendErrorNow("UNKNOWN", "Serial line too long");
        }
      }
    }

    flushLogs();
    vTaskDelay(pdMS_TO_TICKS(5));
  }
}

// ---------------------------------------------------------
// Arduino setup / loop
// ---------------------------------------------------------

void setup() {
  Serial.begin(BAUD_RATE);
  Serial.setTimeout(10);
  delay(500);

  pinMode(PIN_TOOL_SUCTION, OUTPUT);
  digitalWrite(PIN_TOOL_SUCTION, LOW);
  pinMode(PIN_Q1_ESTOP_LIMIT, INPUT);

  ledcSetup(CH_Q1, SERVO_FREQ_HZ, SERVO_RES_BITS);
  ledcSetup(CH_Q2, SERVO_FREQ_HZ, SERVO_RES_BITS);
  ledcSetup(CH_Q3, SERVO_FREQ_HZ, SERVO_RES_BITS);

  ledcAttachPin(PIN_Q1_Z_SCREW, CH_Q1);
  ledcAttachPin(PIN_Q2_ARM_1, CH_Q2);
  ledcAttachPin(PIN_Q3_ARM_2, CH_Q3);

  applySafeOutputs();

  commandQueue = xQueueCreate(12, sizeof(RobotCommand));
  logQueue = xQueueCreate(24, sizeof(LogLine));

  attachInterrupt(digitalPinToInterrupt(PIN_Q1_ESTOP_LIMIT), onEstopChange, CHANGE);

  xTaskCreatePinnedToCore(serialTask, "SerialTask", 8192, nullptr, 2, nullptr, 0);
  xTaskCreatePinnedToCore(motionTask, "MotionTask", 8192, nullptr, 3, nullptr, 1);

  queueStatus("READY", "IDLE", "ESP32 WROOM hardware test ready @ 115200");
}

void loop() {
  vTaskDelay(pdMS_TO_TICKS(1000));
}
