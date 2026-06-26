import sys
from pathlib import Path

from PySide6.QtWidgets import QApplication

from hmi.main_window import MainWindow


def load_stylesheet(app: QApplication) -> None:
    style_path = Path(__file__).parent / "styles" / "main.qss"

    if style_path.exists():
        with open(style_path, "r", encoding="utf-8") as file:
            app.setStyleSheet(file.read())


def run_app() -> None:
    app = QApplication(sys.argv)

    load_stylesheet(app)

    window = MainWindow()
    window.show()

    sys.exit(app.exec())