/*
  Dummy Serial JSON Protocol
  PRR Robotic Pipetting Project

  Board: ESP32-C6
  Purpose:
    - Receive simple JSON commands through Serial.
    - Send dummy responses to validate communication with the Python HMI.
    - No motors are controlled in this version.

  Protocol:
    - Baud rate: 115200
    - Encoding: UTF-8
    - One JSON object per line
    - Line ending: \n

  Supported commands:
    {"cmd":"PING"}
    {"cmd":"HOME"}
    {"cmd":"STOP"}
*/

#include <ArduinoJson.h>

const unsigned long BAUD_RATE = 115200;
const unsigned long DUMMY_HOMING_DELAY_MS = 2000;

String inputLine = "";

void sendAck(const char* cmd, bool ok, const char* message) {
  StaticJsonDocument<256> doc;

  doc["type"] = "ack";
  doc["cmd"] = cmd;
  doc["ok"] = ok;
  doc["message"] = message;

  serializeJson(doc, Serial);
  Serial.println();
}

void sendStatus(const char* status, const char* message) {
  StaticJsonDocument<256> doc;

  doc["type"] = "status";
  doc["status"] = status;
  doc["message"] = message;

  serializeJson(doc, Serial);
  Serial.println();
}

void sendError(const char* message) {
  StaticJsonDocument<256> doc;

  doc["type"] = "error";
  doc["ok"] = false;
  doc["message"] = message;

  serializeJson(doc, Serial);
  Serial.println();
}

void sendCommandError(const char* cmd, const char* message) {
  StaticJsonDocument<256> doc;

  doc["type"] = "error";
  doc["cmd"] = cmd;
  doc["ok"] = false;
  doc["message"] = message;

  serializeJson(doc, Serial);
  Serial.println();
}

void handlePing() {
  sendAck("PING", true, "ESP32-C6 alive");
}

void handleHome() {
  sendAck("HOME", true, "HOME command received");
  sendStatus("HOMING", "Dummy homing started");

  delay(DUMMY_HOMING_DELAY_MS);

  sendStatus("HOMED", "Dummy homing completed");
}

void handleStop() {
  sendAck("STOP", true, "STOP command received");
  sendStatus("STOPPED", "Dummy stop completed");
}

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

  if (strcmp(cmd, "PING") == 0) {
    handlePing();
  } else if (strcmp(cmd, "HOME") == 0) {
    handleHome();
  } else if (strcmp(cmd, "STOP") == 0) {
    handleStop();
  } else {
    sendCommandError(cmd, "Unknown command");
  }
}

void setup() {
  Serial.begin(BAUD_RATE);

  delay(1000);

  sendStatus("READY", "ESP32-C6 dummy firmware ready");
  sendStatus("IDLE", "Waiting for commands");
}

void loop() {
  while (Serial.available() > 0) {
    char incomingChar = Serial.read();

    if (incomingChar == '\n') {
      processJsonLine(inputLine);
      inputLine = "";
    } else {
      inputLine += incomingChar;
    }
  }
}