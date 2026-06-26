"""
Serial controller for communication between the Python HMI/control layer
and the ESP32-C6 firmware.

This module sends JSON commands over serial and reads JSON responses
from the firmware.

Protocol:
    - Baud rate: 115200
    - Encoding: UTF-8
    - One JSON object per line
    - Line ending: \\n
"""

from __future__ import annotations

import json
import threading
from typing import Callable, Optional

import serial
from serial import SerialException

from control.serial_protocol import (
    BAUD_RATE,
    ENCODING,
    LINE_ENDING,
    READ_TIMEOUT_S,
    Command,
    make_home_command,
    make_ping_command,
    make_stop_command,
)


FirmwareMessageCallback = Callable[[dict], None]
ErrorCallback = Callable[[str], None]


class SerialController:
    """
    Handles serial communication with the ESP32-C6 firmware.

    This class does not depend on PySide6 directly. That keeps it reusable
    and easier to test outside the HMI.
    """

    def __init__(
        self,
        port: str,
        baud_rate: int = BAUD_RATE,
        read_timeout_s: float = READ_TIMEOUT_S,
    ) -> None:
        self.port = port
        self.baud_rate = baud_rate
        self.read_timeout_s = read_timeout_s

        self._serial: Optional[serial.Serial] = None
        self._reader_thread: Optional[threading.Thread] = None
        self._running = False

        self.on_message: Optional[FirmwareMessageCallback] = None
        self.on_error: Optional[ErrorCallback] = None

    # ------------------------------------------------------------
    # Connection management
    # ------------------------------------------------------------

    @property
    def is_connected(self) -> bool:
        return self._serial is not None and self._serial.is_open

    def connect(self) -> None:
        """
        Open the serial connection and start the reader thread.
        """
        if self.is_connected:
            return

        try:
            self._serial = serial.Serial(
                port=self.port,
                baudrate=self.baud_rate,
                timeout=self.read_timeout_s,
            )
        except SerialException as exc:
            self._emit_error(f"Could not open serial port {self.port}: {exc}")
            return

        self._running = True
        self._reader_thread = threading.Thread(
            target=self._read_loop,
            daemon=True,
        )
        self._reader_thread.start()

    def disconnect(self) -> None:
        """
        Stop the reader thread and close the serial connection.
        """
        self._running = False

        if self._reader_thread and self._reader_thread.is_alive():
            self._reader_thread.join(timeout=1.0)

        if self._serial and self._serial.is_open:
            self._serial.close()

        self._serial = None
        self._reader_thread = None

    # ------------------------------------------------------------
    # Command sending
    # ------------------------------------------------------------

    def send_command(self, command: dict) -> None:
        """
        Send a command dictionary as one JSON line.
        """
        if not self.is_connected or self._serial is None:
            self._emit_error("Cannot send command: serial port is not connected")
            return

        try:
            json_line = json.dumps(command) + LINE_ENDING
            self._serial.write(json_line.encode(ENCODING))
            self._serial.flush()
        except SerialException as exc:
            self._emit_error(f"Serial write error: {exc}")

    def ping(self) -> None:
        self.send_command(make_ping_command())

    def home(self) -> None:
        self.send_command(make_home_command())

    def stop(self) -> None:
        self.send_command(make_stop_command())

    # ------------------------------------------------------------
    # Reading loop
    # ------------------------------------------------------------

    def _read_loop(self) -> None:
        """
        Continuously read JSON lines from the firmware.
        """
        while self._running:
            if not self._serial or not self._serial.is_open:
                continue

            try:
                raw_line = self._serial.readline()
            except SerialException as exc:
                self._emit_error(f"Serial read error: {exc}")
                continue

            if not raw_line:
                continue

            try:
                line = raw_line.decode(ENCODING).strip()
            except UnicodeDecodeError:
                self._emit_error("Received non UTF-8 data from firmware")
                continue

            if not line:
                continue

            try:
                message = json.loads(line)
            except json.JSONDecodeError:
                self._emit_error(f"Received invalid JSON from firmware: {line}")
                continue

            self._emit_message(message)

    # ------------------------------------------------------------
    # Callback helpers
    # ------------------------------------------------------------

    def _emit_message(self, message: dict) -> None:
        if self.on_message is not None:
            self.on_message(message)

    def _emit_error(self, message: str) -> None:
        if self.on_error is not None:
            self.on_error(message)