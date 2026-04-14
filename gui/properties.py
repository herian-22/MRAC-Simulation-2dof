"""
properties.py — Context-sensitive block parameter editor panel.

Each block type has its own form with editable parameters.
"""

import numpy as np
from PySide6.QtWidgets import (
    QWidget, QVBoxLayout, QFormLayout, QLabel, QLineEdit,
    QComboBox, QPushButton, QScrollArea, QFrame, QMessageBox
)
from PySide6.QtCore import Signal

from gui.theme import Theme
from models.config import SimConfig


class PropertiesPanel(QWidget):
    """Context-sensitive block parameter editor."""

    params_changed = Signal()

    def __init__(self, config: SimConfig):
        super().__init__()
        self.config = config
        self.current_block = None
        self.fields = {}

        self.main_layout = QVBoxLayout(self)
        self.main_layout.setContentsMargins(0, 0, 0, 0)
        self.main_layout.setSpacing(8)

        # Header
        self.header_label = QLabel("Block Parameters")
        self.header_label.setObjectName("blockTitle")
        self.main_layout.addWidget(self.header_label)

        # Separator
        sep = QFrame()
        sep.setObjectName("separator")
        sep.setFrameShape(QFrame.Shape.HLine)
        self.main_layout.addWidget(sep)

        # Scroll area
        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setFrameShape(QFrame.Shape.NoFrame)
        self.form_container = QWidget()
        self.form_layout = QFormLayout(self.form_container)
        self.form_layout.setContentsMargins(4, 4, 4, 4)
        self.form_layout.setSpacing(10)
        scroll.setWidget(self.form_container)
        self.main_layout.addWidget(scroll, 1)

        # Apply button
        self.update_btn = QPushButton(" Apply Parameters")
        self.update_btn.clicked.connect(self._apply_params)
        self.main_layout.addWidget(self.update_btn)

        self.show_block("input")

    def _clear_form(self):
        while self.form_layout.count():
            child = self.form_layout.takeAt(0)
            if child.widget():
                child.widget().deleteLater()
        self.fields.clear()

    def _add_section(self, title):
        lbl = QLabel(title)
        lbl.setObjectName("sectionTitle")
        self.form_layout.addRow(lbl)

    def _add_field(self, key, label, value, tooltip=""):
        le = QLineEdit(str(value))
        le.setToolTip(tooltip)
        self.fields[key] = le
        self.form_layout.addRow(label, le)

    def _add_combo(self, key, label, items, current):
        cb = QComboBox()
        cb.addItems(items)
        idx = cb.findText(current)
        if idx >= 0:
            cb.setCurrentIndex(idx)
        self.fields[key] = cb
        self.form_layout.addRow(label, cb)

    def _add_readonly(self, label, value):
        lbl = QLabel(str(value))
        lbl.setStyleSheet(f"color: {Theme.TEXT_SECONDARY}; font-style: italic;")
        self.form_layout.addRow(label, lbl)

    def show_block(self, block_id):
        self.current_block = block_id
        self._clear_form()
        cfg = self.config

        if block_id == "input":
            self.header_label.setText(" Step Input — Setpoint")
            self._add_section("Trajectory")
            self._add_combo("traj_type", "Type:",
                          ["step", "sinusoidal", "multipoint"],
                          cfg.simulation.trajectory_type)
            self._add_section("Step Parameters")
            self._add_field("q1_target", "θ₁ Azimuth (°):",
                          f"{np.degrees(cfg.simulation.q1_target):.1f}",
                          "Target azimuth angle")
            self._add_field("q2_target", "θ₂ Elevation (°):",
                          f"{np.degrees(cfg.simulation.q2_target):.1f}",
                          "Target elevation angle")
            self._add_section("Simulation Time")
            self._add_field("t_end", "T-End (s):", f"{cfg.simulation.t_end:.1f}")
            self._add_field("dt", "dt (s):", f"{cfg.simulation.dt:.4f}")

        elif block_id == "ref":
            self.header_label.setText("〰️  Reference Model — 2nd Order")
            self._add_section("Computed Parameters")
            self._add_readonly("ωn (rad/s):", f"{cfg.reference.omega_n:.4f}")
            self._add_readonly("ζ (damping):", f"{cfg.reference.zeta:.4f}")
            self._add_section("Design Specs")
            self._add_field("overshoot", "Overshoot (%):",
                          f"{cfg.reference.overshoot * 100:.1f}")
            self._add_field("peak_time", "Peak Time (s):",
                          f"{cfg.reference.peak_time:.1f}")

        elif block_id == "mrac":
            self.header_label.setText("MRAC Controller")
            self._add_section("Computed Torque (PD Gains)")
            self._add_field("Kp1", "Kp₁ (Azimuth):", f"{cfg.controller.Kp1:.1f}")
            self._add_field("Kp2", "Kp₂ (Elevation):", f"{cfg.controller.Kp2:.1f}")
            self._add_field("Kv1", "Kv₁ (Azimuth):", f"{cfg.controller.Kv1:.1f}")
            self._add_field("Kv2", "Kv₂ (Elevation):", f"{cfg.controller.Kv2:.1f}")
            self._add_section("Adaptive Gains (MIT Rule)")
            self._add_field("gamma1", "γ₁ (Azimuth):", f"{cfg.controller.gamma1:.1f}",
                          "Range: 710 – 830")
            self._add_field("gamma2", "γ₂ (Elevation):", f"{cfg.controller.gamma2:.1f}",
                          "Range: 880 – 990")
            self._add_section("Bounds")
            self._add_field("theta_max", "θ_max:", f"{cfg.controller.theta_max:.1f}")
            self._add_field("theta_min", "θ_min:", f"{cfg.controller.theta_min:.1f}")

        elif block_id == "plant":
            self.header_label.setText("📡  Satellite Dish — 2-DoF Plant")
            self._add_section("Link 1 (Azimuth Base)")
            self._add_field("m1", "Mass m₁ (kg):", f"{cfg.physical.m1:.1f}")
            self._add_field("a1", "Parameter a₁ (m):", f"{cfg.physical.a1:.2f}")
            self._add_field("d1", "Parameter d₁ (m):", f"{cfg.physical.d1:.2f}")
            self._add_section("Link 2 (Elevation + Dish 1.6m)")
            self._add_field("m2", "Mass m₂ (kg):", f"{cfg.physical.m2:.1f}")
            self._add_field("a2", "Parameter a₂ (m):", f"{cfg.physical.a2:.2f}")
            self._add_section("Environment")
            self._add_field("g", "Gravity (m/s²):", f"{cfg.physical.g:.2f}")

        elif block_id == "mit":
            self.header_label.setText(" MIT Rule — Adaptive Law")
            self._add_section("Algorithm")
            self._add_readonly("Update law:", "dθ/dt = −γ · e · φ(t)")
            self._add_readonly("Params/joint:", "3 (θ_r, θ_q, θ_dq)")
            self._add_section("Adaptive Gains")
            self._add_field("gamma1", "γ₁ (Azimuth):", f"{cfg.controller.gamma1:.1f}")
            self._add_field("gamma2", "γ₂ (Elevation):", f"{cfg.controller.gamma2:.1f}")

        elif block_id == "scope":
            self.header_label.setText("Scope — Output Display")
            self._add_section("Information")
            self._add_readonly("Status:", "Press ▶ Run to view plots")
            self._add_readonly("Available:", "7 scope tabs")
            info = QLabel(
                "  • Joint 1 & 2 Response\n"
                "  • Tracking Error\n"
                "  • Torque Profile\n"
                "  • Adaptive Parameters\n"
                "  • Phase Portrait\n"
                "  • Performance Metrics"
            )
            info.setStyleSheet(
                f"color: {Theme.TEXT_SECONDARY}; padding: 8px; "
                f"background: {Theme.BG_MID}; border-radius: 4px;"
            )
            self.form_layout.addRow(info)

    def _apply_params(self):
        try:
            cfg = self.config
            bid = self.current_block

            if bid == "input":
                cfg.simulation.trajectory_type = self.fields['traj_type'].currentText()
                cfg.simulation.q1_target = np.radians(float(self.fields['q1_target'].text()))
                cfg.simulation.q2_target = np.radians(float(self.fields['q2_target'].text()))
                cfg.simulation.t_end = float(self.fields['t_end'].text())
                cfg.simulation.dt = float(self.fields['dt'].text())

            elif bid == "ref":
                cfg.reference.overshoot = float(self.fields['overshoot'].text()) / 100.0
                cfg.reference.peak_time = float(self.fields['peak_time'].text())

            elif bid in ("mrac", "mit"):
                if 'Kp1' in self.fields:
                    cfg.controller.Kp1 = float(self.fields['Kp1'].text())
                    cfg.controller.Kp2 = float(self.fields['Kp2'].text())
                    cfg.controller.Kv1 = float(self.fields['Kv1'].text())
                    cfg.controller.Kv2 = float(self.fields['Kv2'].text())
                cfg.controller.gamma1 = float(self.fields['gamma1'].text())
                cfg.controller.gamma2 = float(self.fields['gamma2'].text())
                if 'theta_max' in self.fields:
                    cfg.controller.theta_max = float(self.fields['theta_max'].text())
                    cfg.controller.theta_min = float(self.fields['theta_min'].text())

            elif bid == "plant":
                cfg.physical.m1 = float(self.fields['m1'].text())
                cfg.physical.a1 = float(self.fields['a1'].text())
                cfg.physical.d1 = float(self.fields['d1'].text())
                cfg.physical.m2 = float(self.fields['m2'].text())
                cfg.physical.a2 = float(self.fields['a2'].text())
                cfg.physical.g = float(self.fields['g'].text())

            self.params_changed.emit()

        except ValueError as e:
            QMessageBox.warning(self, "Invalid Input",
                              f"Enter valid numeric values.\n\nError: {e}")
