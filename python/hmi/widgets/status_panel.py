from PySide6.QtWidgets import QGroupBox, QVBoxLayout, QTextEdit


class StatusPanel(QGroupBox):
    def __init__(self) -> None:
        super().__init__("Status Log")

        self.log_box = QTextEdit()
        self.log_box.setReadOnly(True)

        self._build_ui()

    def _build_ui(self) -> None:
        layout = QVBoxLayout()
        layout.addWidget(self.log_box)
        self.setLayout(layout)

    def log_message(self, message: str) -> None:
        self.log_box.append(f"> {message}")