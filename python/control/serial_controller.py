"""
Serial controller for communication between the Python HMI/control layer
and the ESP32 firmware.

This module sends JSON commands over serial and reads JSON responses
from the firmware.

Protocol:
    - Baud rate: 115200
    - Encoding: UTF-8
    - One JSON object per line
    - Line ending: \n
Diagnostic behavior:
    - Every transmitted JSON line is printed as TX SERIAL.
    - Every received JSON line is printed as RX SERIAL.
    - Callers can wait for ACK and terminal status messages before sending
      the next command.
"""

from __future__ import annotations

import json
import threading
import time
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
MessagePredicate = Callable[[dict], bool]


class SerialController:
    """
    Handles serial communication with the ESP32 firmware.

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

        self._message_condition = threading.Condition()
        self._message_history: list[dict] = []
        self._wait_cursor = 0

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

        with self._message_condition:
            self._message_history.clear()
            self._wait_cursor = 0

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

        with self._message_condition:
            self._message_condition.notify_all()

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
            json_line = json.dumps(command)
            print(f"TX SERIAL: {json_line}", flush=True)
            self._serial.write((json_line + LINE_ENDING).encode(ENCODING))
            self._serial.flush()
        except SerialException as exc:
            self._emit_error(f"Serial write error: {exc}")

    def mark_wait_cursor_at_end(self) -> None:
        """
        Move the wait cursor to the newest received message.

        The UI still logs all messages, but the next wait_for_ack/status call
        starts from messages received after this point.
        """
        with self._message_condition:
            self._wait_cursor = len(self._message_history)

    def wait_for_message(
        self,
        predicate: MessagePredicate,
        timeout_s: float = 10.0,
        consume: bool = True,
    ) -> Optional[dict]:
        """
        Wait until a received JSON message matches predicate.

        The wait cursor prevents one command from consuming stale messages
        produced by a previous command. If consume=True, the cursor advances
        past the matched message.
        """
        deadline = time.monotonic() + timeout_s

        with self._message_condition:
            start_index = self._wait_cursor

            while True:
                for index in range(start_index, len(self._message_history)):
                    message = self._message_history[index]
                    if predicate(message):
                        if consume:
                            self._wait_cursor = index + 1
                        return message

                remaining_s = deadline - time.monotonic()
                if remaining_s <= 0.0:
                    return None

                self._message_condition.wait(timeout=remaining_s)
                start_index = self._wait_cursor

    def wait_for_ack(self, cmd: str, timeout_s: float = 5.0) -> Optional[dict]:
        return self.wait_for_message(
            lambda message: (
                message.get("type") == "ack"
                and message.get("cmd") == cmd
                and bool(message.get("ok", False))
            ),
            timeout_s=timeout_s,
        )

    def wait_for_status(
        self,
        statuses: set[str] | tuple[str, ...] | list[str],
        timeout_s: float = 20.0,
    ) -> Optional[dict]:
        expected = set(statuses)
        return self.wait_for_message(
            lambda message: (
                message.get("type") == "status"
                and message.get("status") in expected
            ),
            timeout_s=timeout_s,
        )

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

            print(f"RX SERIAL: {line}", flush=True)

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
        with self._message_condition:
            self._message_history.append(message)
            self._message_condition.notify_all()

        if self.on_message is not None:
            self.on_message(message)

    def _emit_error(self, message: str) -> None:
        if self.on_error is not None:
            self.on_error(message)
