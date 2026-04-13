"""
metrics.py — Performance metrics table widget.

Displays simulation metrics in a dark-styled QTableWidget.
"""

import numpy as np
from PySide6.QtWidgets import QTableWidget, QTableWidgetItem, QHeaderView
from PySide6.QtCore import Qt
from PySide6.QtGui import QColor

from gui.theme import Theme


class MetricsTable(QTableWidget):
    """Performance metrics table with dark styling."""

    METRIC_NAMES = [
        "Steady-State Error (°)",
        "Overshoot (%)",
        "Settling Time 2% (s)",
        "RMS Error (°)",
        "Max |Torque| (Nm)",
        "ISE (rad²·s)",
        "Peak Error (°)",
    ]

    def __init__(self):
        super().__init__()
        self.setColumnCount(3)
        self.setHorizontalHeaderLabels(["Metric", "Joint 1 (Azimuth)", "Joint 2 (Elevation)"])
        self.horizontalHeader().setSectionResizeMode(QHeaderView.ResizeMode.Stretch)
        self.verticalHeader().setVisible(False)
        self.setAlternatingRowColors(True)
        self.setStyleSheet(f"""
            QTableWidget {{
                alternate-background-color: {Theme.BG_MID};
            }}
        """)
        self._init_rows()

    def _init_rows(self):
        self.setRowCount(len(self.METRIC_NAMES))
        for i, name in enumerate(self.METRIC_NAMES):
            item = QTableWidgetItem(name)
            item.setFlags(item.flags() & ~Qt.ItemFlag.ItemIsEditable)
            item.setForeground(QColor(Theme.TEXT_PRIMARY))
            self.setItem(i, 0, item)
            for j in range(1, 3):
                val = QTableWidgetItem("—")
                val.setFlags(val.flags() & ~Qt.ItemFlag.ItemIsEditable)
                val.setTextAlignment(Qt.AlignmentFlag.AlignCenter)
                val.setForeground(QColor(Theme.TEXT_SECONDARY))
                self.setItem(i, j, val)

    def update_metrics(self, m: dict):
        """Fill table from simulation metrics dict."""
        rows = [
            (np.degrees(m.get('ss_error_j1', 0)),
             np.degrees(m.get('ss_error_j2', 0))),
            (m.get('overshoot_j1_pct', 0),
             m.get('overshoot_j2_pct', 0)),
            (m.get('settling_time_j1', 0),
             m.get('settling_time_j2', 0)),
            (np.degrees(m.get('rms_error_j1', 0)),
             np.degrees(m.get('rms_error_j2', 0))),
            (m.get('max_torque_j1', 0),
             m.get('max_torque_j2', 0)),
            (m.get('ise_j1', 0),
             m.get('ise_j2', 0)),
            (np.degrees(m.get('peak_error_j1', 0)),
             np.degrees(m.get('peak_error_j2', 0))),
        ]
        for i, (v1, v2) in enumerate(rows):
            for j, v in enumerate([v1, v2], 1):
                item = self.item(i, j)
                if item:
                    item.setText(f"{v:.4f}")
                    # Color non-zero values green
                    item.setForeground(QColor(Theme.GREEN if abs(v) > 1e-8 else Theme.TEXT_SECONDARY))
