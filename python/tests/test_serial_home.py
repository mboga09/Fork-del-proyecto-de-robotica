"""
Manual serial communication test for the ESP32-C6 dummy firmware.

Purpose:
    Test the JSON serial protocol without the HMI.

Expected behavior:
    1. Connect to ESP32-C6.
    2. Send PING.
    3. Send HOME.
    4. Print firmware responses:
        - READY
        - IDLE
        - PING ack
        - HOME ack
        - HOMING
        - HOMED
"""

import time

from control.serial_controller import SerialController


PORT = "COM4"  # Change this to your ESP32-C6 port


def handle_message(message: dict) -> None:
    print(f"[FIRMWARE] {message}")


def handle_error(error: str) -> None:
    print(f"[ERROR] {error}")


def main() -> None:
    controller = SerialController(port=PORT)

    controller.on_message = handle_message
    controller.on_error = handle_error

    print(f"Connecting to {PORT}...")
    controller.connect()

    time.sleep(2)

    print("Sending PING...")
    controller.ping()

    time.sleep(1)

    print("Sending HOME...")
    controller.home()

    time.sleep(4)

    print("Disconnecting...")
    controller.disconnect()


if __name__ == "__main__":
    main()