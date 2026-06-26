# Serial JSON Protocol

This document defines the serial communication protocol between the Python HMI/control layer and the ESP32-C6 firmware for the PRR robotic pipetting project.

The goal of this first protocol version is to validate communication only:

```text
HMI button press
→ Python control layer event
→ Serial JSON command
→ ESP32-C6 dummy response
→ HMI status update

ESP32-C6 requires USB CDC On Boot enabled in Arduino IDE for Serial Monitor/HMI communication.