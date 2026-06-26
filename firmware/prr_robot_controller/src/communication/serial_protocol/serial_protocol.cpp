#include "serial_protocol.h"

#include <Arduino.h>
#include <ArduinoJson.h>

#include "../../../config.h"
#include "../../controller_state_machine/robot_state_machine/robot_state_machine.h"


static String inputLine = "";

void initializeSerialProtocol() {
  Serial.begin(BAUD_RATE);

  unsigned long startTime = millis();

  while (!Serial && millis() - startTime < 5000) {
    delay(10);
  }

  delay(500);
}

bool readSerialLine(String& lineOut) {
  while (Serial.available() > 0) {
    char incomingChar = Serial.read();

    if (incomingChar == '\n') {
      lineOut = inputLine;
      inputLine = "";
      return true;
    }

    inputLine += incomingChar;
  }

  return false;
}

void sendStatus(const char* status, const char* message) {
  StaticJsonDocument<256> doc;

  doc["type"] = "status";
  doc["status"] = status;
  doc["state"] = getCurrentStateString();
  doc["homed"] = getIsHomed();
  doc["message"] = message;

  serializeJson(doc, Serial);
  Serial.println();
}

void sendAck(const char* cmd, bool ok, const char* message) {
  StaticJsonDocument<256> doc;

  doc["type"] = "ack";
  doc["cmd"] = cmd;
  doc["ok"] = ok;
  doc["state"] = getCurrentStateString();
  doc["homed"] = getIsHomed();
  doc["message"] = message;

  serializeJson(doc, Serial);
  Serial.println();
}

void sendCommandError(const char* cmd, const char* message) {
  StaticJsonDocument<256> doc;

  doc["type"] = "error";
  doc["cmd"] = cmd;
  doc["ok"] = false;
  doc["state"] = getCurrentStateString();
  doc["homed"] = getIsHomed();
  doc["message"] = message;

  serializeJson(doc, Serial);
  Serial.println();
}

void sendError(const char* message) {
  StaticJsonDocument<256> doc;

  doc["type"] = "error";
  doc["ok"] = false;
  doc["state"] = getCurrentStateString();
  doc["homed"] = getIsHomed();
  doc["message"] = message;

  serializeJson(doc, Serial);
  Serial.println();
}