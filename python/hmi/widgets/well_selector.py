from PySide6.QtCore import Signal
from PySide6.QtWidgets import QGroupBox, QGridLayout, QPushButton


class WellSelector(QGroupBox):
    well_selection_changed = Signal(list)

    def __init__(self, plate_config: dict | None = None) -> None:
        super().__init__("Well Selector")

        self.plate_config = plate_config or {}
        self.well_buttons: dict[str, QPushButton] = {}
        self.selection_order: list[str] = []

        self._build_ui()

    def _build_ui(self) -> None:
        layout = QGridLayout()

        rows = self.plate_config.get("rows", ["A", "B"])
        columns = self.plate_config.get("columns", [1, 2, 3])

        for row_index, row_name in enumerate(rows):
            for column_index, column_number in enumerate(columns):
                well_name = f"{row_name}{column_number}"

                button = QPushButton(well_name)
                button.setCheckable(True)
                button.toggled.connect(
                    lambda checked, name=well_name: self._on_well_toggled(name, checked)
                )

                self.well_buttons[well_name] = button
                layout.addWidget(button, row_index, column_index)

        self.setLayout(layout)

    def _on_well_toggled(self, well_name: str, checked: bool) -> None:
        if checked:
            if well_name not in self.selection_order:
                self.selection_order.append(well_name)
        else:
            if well_name in self.selection_order:
                self.selection_order.remove(well_name)

        self.well_selection_changed.emit(self.get_selected_wells())

    def get_selected_wells(self) -> list[str]:
        return [
            well_name
            for well_name, button in self.well_buttons.items()
            if button.isChecked()
        ]

    def get_all_wells(self) -> list[str]:
        return list(self.well_buttons.keys())

    def get_route_wells(self) -> list[str]:
        return self.selection_order.copy()

    def clear_selection(self) -> None:
        for button in self.well_buttons.values():
            button.setChecked(False)

        self.selection_order.clear()
        self.well_selection_changed.emit([])