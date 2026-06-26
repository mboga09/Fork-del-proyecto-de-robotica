"""
Qt bridge for the serial controller.

This class connects the non-Qt SerialController callbacks to PySide6 signals
so the HMI can safely update widgets from the main UI thread.
"""

from __future__ import annotations

from PySide6.QtCore import QObject, Signal

from control.serial_controller import SerialController
from control.serial_protocol import get_message_text


class SerialQtBridge(QObject):
    firmware_message_received = Signal(dict)
    status_text_received = Signal(str)
    error_received = Signal(str)
    connection_changed = Signal(bool)

    def __init__(
        self,
        port: str,
        baud_rate: int,
        parent: QObject | None = None,
    ) -> None:
        super().__init__(parent)

        self.controller = SerialController(
            port=port,
            baud_rate=baud_rate,
        )

        self.controller.on_message = self._handle_firmware_message
        self.controller.on_error = self._handle_serial_error

    @property
    def is_connected(self) -> bool:
        return self.controller.is_connected

    def connect_serial(self) -> None:
        self.controller.connect()
        self.connection_changed.emit(self.controller.is_connected)

    def disconnect_serial(self) -> None:
        self.controller.disconnect()
        self.connection_changed.emit(False)

    def ping(self) -> None:
        self.controller.ping()

    def home(self) -> None:
        self.controller.home()

    def stop(self) -> None:
        self.controller.stop()

    def _handle_firmware_message(self, message: dict) -> None:
        self.firmware_message_received.emit(message)
        self.status_text_received.emit(get_message_text(message))

    def _handle_serial_error(self, error: str) -> None:
        self.error_received.emit(error)