#pragma once

#include <Arduino.h>

void initializeSerialProtocol();

bool readSerialLine(String& lineOut);

void sendStatus(const char* status, const char* message);
void sendAck(const char* cmd, bool ok, const char* message);
void sendCommandError(const char* cmd, const char* message);
void sendError(const char* message);