"""
app.py — Main SimulinkGUI window that assembles all components.

Layout:
    ┌──────────────────────────────────────────────┐
    │  Toolbar: [▶ Run] [📋 Export] | progress bar │
    ├──────────────────────────────────────────────┤
    │  Block Diagram Canvas (Interactive)          │
    ├──────────────┬───────────────────────────────┤
    │  Properties  │  Scope Tabs (7 views)         │
    │  Panel       │  + Metrics Table              │
    └──────────────┴───────────────────────────────┘

Graph Tab Structure:
    1. Basic System Output (4 subplots)
    2. MRAC Performance Analysis (4 subplots)
    3. Adaptive Parameters & Robustness (2 subplots)
    4. Optimization: Gain Variation γ (2 subplots) — requires batch sim
    5. Metrics Table
    6. 3D Dynamics Visualizer
"""

import sys
import os
import numpy as np

from PySide6.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout,
    QSplitter, QTabWidget, QLabel, QPushButton,
    QProgressBar, QToolBar, QFrame, QMessageBox,
    QFileDialog, QSizePolicy
)
from PySide6.QtCore import Qt
from PySide6.QtGui import QColor, QPalette, QIcon

from gui.theme import Theme, build_stylesheet
from gui.diagram import BlockDiagramView
from gui.properties import PropertiesPanel
from gui.scope import ScopePlotCanvas
from gui.metrics import MetricsTable
from gui.worker import SimulationWorker, GainVariationWorker
from gui.visualizer_pro import RobotArmVisualizer

from models.config import default_config


class SimulinkGUI(QMainWindow):
    """Main Simulink-like window for MRAC 2-DoF Satellite Dish."""

    def __init__(self):
        super().__init__()
        self.setWindowTitle("MRAC Simulation — 2-DoF Satellite Dish Receiver")
        
        icon_path = os.path.join(os.path.dirname(os.path.dirname(__file__)), "mrac_icon.ico")
        if os.path.exists(icon_path):
            self.setWindowIcon(QIcon(icon_path))
            
        self.showMaximized()
        self.sim_config = default_config()
        self.sim_result = None
        self.worker = None
        self.gain_worker = None
        self.gain_results = None  # List of (gamma, SimResult) for gain variation

        self._build_toolbar()
        self._build_ui()
        self._build_statusbar()

        self.statusBar().showMessage(
            "Ready — Click a block to edit parameters, then press ▶ Run"
        )

    # ── Toolbar ──────────────────────────────────────────────
    def _build_toolbar(self):
        toolbar = QToolBar("Main Toolbar")
        toolbar.setMovable(False)
        self.addToolBar(toolbar)

        title_lbl = QLabel("  MRAC Simulink  ")
        title_lbl.setStyleSheet(
            f"color: {Theme.BLUE}; font-size: 16px; font-weight: bold;"
        )
        toolbar.addWidget(title_lbl)
        toolbar.addSeparator()

        self.run_btn = QPushButton("  ▶  Run Simulation  ")
        self.run_btn.setObjectName("runBtn")
        self.run_btn.clicked.connect(self._run_simulation)
        toolbar.addWidget(self.run_btn)

        self.export_btn = QPushButton("  📋  Export Results  ")
        self.export_btn.setObjectName("exportBtn")
        self.export_btn.clicked.connect(self._export_results)
        self.export_btn.setEnabled(False)
        toolbar.addWidget(self.export_btn)

        toolbar.addSeparator()

        self.progress_bar = QProgressBar()
        self.progress_bar.setFixedWidth(200)
        self.progress_bar.setRange(0, 0)
        self.progress_bar.setVisible(False)
        toolbar.addWidget(self.progress_bar)

        self.sim_status_lbl = QLabel("")
        toolbar.addWidget(self.sim_status_lbl)

        spacer = QWidget()
        spacer.setSizePolicy(QSizePolicy.Policy.Expanding,
                             QSizePolicy.Policy.Preferred)
        toolbar.addWidget(spacer)

        self.config_lbl = QLabel()
        self._update_config_label()
        toolbar.addWidget(self.config_lbl)

    def _update_config_label(self):
        c = self.sim_config
        self.config_lbl.setText(
            f"γ₁={c.controller.gamma1:.0f}  γ₂={c.controller.gamma2:.0f}  |  "
            f"ωn={c.reference.omega_n:.2f}  ζ={c.reference.zeta:.2f}  |  "
            f"T={c.simulation.t_end:.0f}s"
        )

    # ── Main UI ──────────────────────────────────────────────
    def _build_ui(self):
        central = QWidget()
        self.setCentralWidget(central)
        main_layout = QVBoxLayout(central)
        main_layout.setContentsMargins(8, 4, 8, 4)
        main_layout.setSpacing(4)

        v_splitter = QSplitter(Qt.Orientation.Vertical)

        # Top: block diagram
        self.diagram_view = BlockDiagramView(self._on_block_clicked)
        self.diagram_view.setMinimumHeight(260)
        v_splitter.addWidget(self.diagram_view)

        # Bottom: properties | scope
        h_splitter = QSplitter(Qt.Orientation.Horizontal)

        # Left: properties
        props_frame = QFrame()
        props_frame.setStyleSheet(f"""
            QFrame {{
                background-color: {Theme.BG_DARK};
                border: 1px solid {Theme.BORDER};
                border-radius: 8px;
            }}
        """)
        props_inner = QVBoxLayout(props_frame)
        props_inner.setContentsMargins(12, 12, 12, 12)

        self.props_panel = PropertiesPanel(self.sim_config)
        self.props_panel.params_changed.connect(self._on_params_changed)
        props_inner.addWidget(self.props_panel)
        h_splitter.addWidget(props_frame)

        # Right: scope tabs + metrics
        right_frame = QFrame()
        right_layout = QVBoxLayout(right_frame)
        right_layout.setContentsMargins(0, 0, 0, 0)
        right_layout.setSpacing(4)

        self.scope_tabs = QTabWidget()
        self.scope_tabs.setDocumentMode(True)

        # === Tab 1: Basic System Output ===
        self.canvas_output = ScopePlotCanvas()
        # === Tab 2: MRAC Performance Analysis ===
        self.canvas_mrac = ScopePlotCanvas()
        # === Tab 3: Adaptive Parameters & Robustness ===
        self.canvas_adaptive = ScopePlotCanvas()
        # === Tab 4: Optimization: Gain Variation ===
        self.canvas_gain_var = ScopePlotCanvas()
        # === Tab 5: Metrics ===
        self.metrics_table = MetricsTable()
        # === Tab 6: 3D Visualizer ===
        self.visualizer_3d = RobotArmVisualizer()

        self.scope_tabs.addTab(self.canvas_output,   "📈 Basic System Output")
        self.scope_tabs.addTab(self.canvas_mrac,     "📊 MRAC Performance Analysis")
        self.scope_tabs.addTab(self.canvas_adaptive, "⚡ Adaptive Parameters")
        self.scope_tabs.addTab(self.canvas_gain_var, "🔄 Optimization: Gain Variation")
        self.scope_tabs.addTab(self.metrics_table,   "📋 Metrics")
        self.scope_tabs.addTab(self.visualizer_3d,   "🛰️ 3D Dynamics")

        right_layout.addWidget(self.scope_tabs)
        h_splitter.addWidget(right_frame)

        h_splitter.setSizes([320, 1050])
        v_splitter.addWidget(h_splitter)
        v_splitter.setSizes([310, 550])
        main_layout.addWidget(v_splitter)

        self._draw_empty_scopes()

    def _draw_empty_scopes(self):
        canvases = [self.canvas_output, self.canvas_mrac,
                    self.canvas_adaptive, self.canvas_gain_var]
        titles = [
            "Basic System Output — Angular Displacement & Velocity",
            "MRAC Performance Analysis — Controlled Variable vs Reference Model",
            "Adaptive Parameters & Robustness Analysis",
            "Optimization — Effect of Adaptive Gain Variation (γ)"
        ]
        for canvas, title in zip(canvases, titles):
            ax = canvas.fig.add_subplot(111)
            canvas.apply_style(ax)
            ax.set_title(title, fontsize=14)
            ax.text(0.5, 0.5, "▶  Run simulation to view results",
                    transform=ax.transAxes, ha='center', va='center',
                    fontsize=14, color=Theme.TEXT_MUTED, style='italic')
            canvas.fig.tight_layout()
            canvas.draw()

    def _build_statusbar(self):
        sb = self.statusBar()
        solver_lbl = QLabel("Solver: RK45")
        solver_lbl.setStyleSheet(
            f"color: {Theme.TEXT_MUTED}; padding: 0 12px;"
        )
        sb.addPermanentWidget(solver_lbl)

    # ── Events ───────────────────────────────────────────────
    def _on_block_clicked(self, block_id):
        self.props_panel.show_block(block_id)
        self.statusBar().showMessage(
            f"Selected: {block_id.upper()} — Edit parameters on the left"
        )

    def _on_params_changed(self):
        self._update_config_label()
        self.props_panel.show_block(self.props_panel.current_block)
        self.statusBar().showMessage("✓ Parameters updated", 3000)

    # ── Simulation ───────────────────────────────────────────
    def _run_simulation(self):
        self.props_panel._apply_params()

        self.run_btn.setEnabled(False)
        self.export_btn.setEnabled(False)
        self.progress_bar.setVisible(True)
        self.sim_status_lbl.setText("  Solving ODE...")
        self.sim_status_lbl.setStyleSheet(
            f"color: {Theme.YELLOW}; font-weight: bold;"
        )
        self.statusBar().showMessage(
            "⏳ Simulation running — solving differential equations..."
        )
        self.diagram_view.start_animation()

        # Run main simulation
        self.worker = SimulationWorker(self.sim_config)
        self.worker.finished.connect(self._on_sim_done)
        self.worker.error_occurred.connect(self._on_sim_error)
        self.worker.start()

    def _on_sim_done(self, result):
        self.sim_result = result

        # Plot tabs 1-3 immediately
        self._plot_output_dasar(result)
        self._plot_mrac_analysis(result)
        self._plot_adaptive_params(result)
        self.metrics_table.update_metrics(result.metrics)

        # Start 3D animation
        self.visualizer_3d.start_animation(result.q, self.sim_config)

        # Now launch gain variation batch simulation
        self.sim_status_lbl.setText("  Solving gain variations...")
        self.sim_status_lbl.setStyleSheet(
            f"color: {Theme.ORANGE}; font-weight: bold;"
        )
        self.statusBar().showMessage(
            "⏳ Running gain variation sweep (γ)..."
        )

        self.gain_worker = GainVariationWorker(self.sim_config)
        self.gain_worker.finished.connect(self._on_gain_done)
        self.gain_worker.error_occurred.connect(self._on_gain_error)
        self.gain_worker.start()

    def _on_gain_done(self, results):
        """Called when gain variation batch simulation completes."""
        self.gain_results = results
        self._plot_gain_variation(results)

        self.run_btn.setEnabled(True)
        self.export_btn.setEnabled(True)
        self.progress_bar.setVisible(False)
        self.diagram_view.stop_animation()

        self.sim_status_lbl.setText("  ✓ Complete")
        self.sim_status_lbl.setStyleSheet(
            f"color: {Theme.GREEN}; font-weight: bold;"
        )

        m = self.sim_result.metrics
        self.statusBar().showMessage(
            f"✓ Done T={self.sim_result.t[-1]:.1f}s | "
            f"SSE: J1={np.degrees(m['ss_error_j1']):.4f}° "
            f"J2={np.degrees(m['ss_error_j2']):.4f}° | "
            f"OS: J1={m['overshoot_j1_pct']:.1f}% "
            f"J2={m['overshoot_j2_pct']:.1f}%"
        )

        # Switch to first output tab
        self.scope_tabs.setCurrentWidget(self.canvas_output)

    def _on_gain_error(self, msg):
        """If gain variation fails, still enable controls."""
        self.run_btn.setEnabled(True)
        self.export_btn.setEnabled(True)
        self.progress_bar.setVisible(False)
        self.diagram_view.stop_animation()

        self.sim_status_lbl.setText("  ✓ Partial (gain sweep failed)")
        self.sim_status_lbl.setStyleSheet(
            f"color: {Theme.YELLOW}; font-weight: bold;"
        )
        self.statusBar().showMessage(
            f"✓ Main simulation done. Gain sweep error: {msg}"
        )

    def _on_sim_error(self, msg):
        self.run_btn.setEnabled(True)
        self.progress_bar.setVisible(False)
        self.diagram_view.stop_animation()
        self.sim_status_lbl.setText("  ✗ Error")
        self.sim_status_lbl.setStyleSheet(
            f"color: {Theme.RED}; font-weight: bold;"
        )
        self.statusBar().showMessage(f"Simulation failed: {msg}")
        QMessageBox.critical(self, "Simulation Error",
                           f"Error:\n\n{msg}")

    # ── Plotting helpers ─────────────────────────────────────
    def _annot_box(self, ax, text, loc='tl'):
        ha, va = ('left', 'top') if loc == 'tl' else ('right', 'top')
        x = 0.02 if loc == 'tl' else 0.98
        ax.text(x, 0.98, text,
                transform=ax.transAxes, ha=ha, va=va, fontsize=10,
                color=Theme.GREEN,
                bbox=dict(boxstyle='round,pad=0.4',
                         facecolor=Theme.BG_LIGHT,
                         edgecolor=Theme.BORDER, alpha=0.9))

    def _legend(self, ax, **kw):
        ax.legend(facecolor=Theme.BG_LIGHT, edgecolor=Theme.BORDER,
                 labelcolor=Theme.TEXT_PRIMARY, fontsize=9, **kw)

    # ────────────────────────────────────────────────────────
    # TAB 1: Basic System Output
    # ────────────────────────────────────────────────────────
    def _plot_output_dasar(self, r):
        """
        4 subplots (2×2):
          (a) Angular Displacement - Joint 1 → target 60°
          (b) Angular Velocity - Joint 1
          (c) Angular Displacement - Joint 2 → target 30°
          (d) Angular Velocity - Joint 2
        """
        c = self.canvas_output
        c.clear()
        t = r.t

        # (a) Angular Displacement - Joint 1
        ax1 = c.fig.add_subplot(221); c.apply_style(ax1)
        ax1.plot(t, np.degrees(r.q[:, 0]), color=Theme.BLUE, lw=1.8,
                 label='Output q₁ (°)')
        ax1.plot(t, np.degrees(r.q_desired[:, 0]), ':', color=Theme.PURPLE,
                 lw=1.2, alpha=.7, label='Reference (60°)')
        ax1.axhline(60, color=Theme.GREEN, lw=0.8, ls='--', alpha=0.5)
        ax1.set_ylabel('Degrees (°)')
        ax1.set_title('Angular Displacement — Joint 1', fontsize=11)
        self._legend(ax1, loc='lower right')

        # (b) Angular Velocity - Joint 1
        ax2 = c.fig.add_subplot(222); c.apply_style(ax2)
        ax2.plot(t, np.degrees(r.dq[:, 0]), color=Theme.RED, lw=1.5,
                 label='dq₁ (°/s)')
        ax2.axhline(0, color=Theme.BORDER, lw=.5)
        ax2.set_ylabel('Degrees/s (°/s)')
        ax2.set_title('Angular Velocity — Joint 1', fontsize=11)
        self._legend(ax2, loc='upper right')

        # (c) Angular Displacement - Joint 2
        ax3 = c.fig.add_subplot(223); c.apply_style(ax3)
        ax3.plot(t, np.degrees(r.q[:, 1]), color=Theme.ORANGE, lw=1.8,
                 label='Output q₂ (°)')
        ax3.plot(t, np.degrees(r.q_desired[:, 1]), ':', color=Theme.PURPLE,
                 lw=1.2, alpha=.7, label='Reference (30°)')
        ax3.axhline(30, color=Theme.GREEN, lw=0.8, ls='--', alpha=0.5)
        ax3.set_xlabel('Time (s)')
        ax3.set_ylabel('Degrees (°)')
        ax3.set_title('Angular Displacement — Joint 2', fontsize=11)
        self._legend(ax3, loc='lower right')

        # (d) Angular Velocity - Joint 2
        ax4 = c.fig.add_subplot(224); c.apply_style(ax4)
        ax4.plot(t, np.degrees(r.dq[:, 1]), color=Theme.CYAN, lw=1.5,
                 label='dq₂ (°/s)')
        ax4.axhline(0, color=Theme.BORDER, lw=.5)
        ax4.set_xlabel('Time (s)')
        ax4.set_ylabel('Degrees/s (°/s)')
        ax4.set_title('Angular Velocity — Joint 2', fontsize=11)
        self._legend(ax4, loc='upper right')

        c.fig.suptitle('Basic System Output Graphs', fontsize=14,
                       fontweight='bold', color=Theme.MPL_TEXT, y=1.0)
        c.fig.tight_layout()
        c.draw()

    # ────────────────────────────────────────────────────────
    # TAB 2: Controller Performance Analysis (MRAC)
    # ────────────────────────────────────────────────────────
    def _plot_mrac_analysis(self, r):
        """
        4 subplots (2×2):
          (a) Controlled Var vs Reference Model - Joint 1
          (b) Controlled Var vs Reference Model - Joint 2
          (c) Error Signal - Joint 1
          (d) Error Signal - Joint 2
        """
        c = self.canvas_mrac
        c.clear()
        t = r.t

        # (a) Controlled Variable vs Reference Model - Joint 1
        ax1 = c.fig.add_subplot(221); c.apply_style(ax1)
        ax1.plot(t, np.degrees(r.q[:, 0]), color=Theme.BLUE, lw=2,
                 label='Controlled Variable (q₁)')
        ax1.plot(t, np.degrees(r.q_ref[:, 0]), '--', color=Theme.GREEN, lw=1.5,
                 label='Reference Model (qm₁)')
        ax1.set_ylabel('Degrees (°)')
        ax1.set_title('Controlled Variable vs Reference Model — Joint 1', fontsize=10)
        self._legend(ax1, loc='lower right')

        # (b) Controlled Variable vs Reference Model - Joint 2
        ax2 = c.fig.add_subplot(222); c.apply_style(ax2)
        ax2.plot(t, np.degrees(r.q[:, 1]), color=Theme.ORANGE, lw=2,
                 label='Controlled Variable (q₂)')
        ax2.plot(t, np.degrees(r.q_ref[:, 1]), '--', color=Theme.GREEN, lw=1.5,
                 label='Reference Model (qm₂)')
        ax2.set_ylabel('Degrees (°)')
        ax2.set_title('Controlled Variable vs Reference Model — Joint 2', fontsize=10)
        self._legend(ax2, loc='lower right')

        # (c) Error Signal - Joint 1
        ax3 = c.fig.add_subplot(223); c.apply_style(ax3)
        e1 = np.degrees(r.error[:, 0])
        ax3.plot(t, e1, color=Theme.RED, lw=1.5, label='e₁ = q₁ - qm₁')
        ax3.fill_between(t, e1, 0, color=Theme.RED, alpha=0.1)
        ax3.axhline(0, color=Theme.BORDER, lw=.5)
        ax3.set_xlabel('Time (s)')
        ax3.set_ylabel('Error (°)')
        ax3.set_title('Error Signal — Joint 1', fontsize=10)
        self._legend(ax3, loc='upper right')

        # (d) Error Signal - Joint 2
        ax4 = c.fig.add_subplot(224); c.apply_style(ax4)
        e2 = np.degrees(r.error[:, 1])
        ax4.plot(t, e2, color=Theme.ORANGE, lw=1.5, label='e₂ = q₂ - qm₂')
        ax4.fill_between(t, e2, 0, color=Theme.ORANGE, alpha=0.1)
        ax4.axhline(0, color=Theme.BORDER, lw=.5)
        ax4.set_xlabel('Time (s)')
        ax4.set_ylabel('Error (°)')
        ax4.set_title('Error Signal — Joint 2', fontsize=10)
        self._legend(ax4, loc='upper right')

        c.fig.suptitle('MRAC Performance Analysis', fontsize=14,
                       fontweight='bold', color=Theme.MPL_TEXT, y=1.0)
        c.fig.tight_layout()
        c.draw()

    # ────────────────────────────────────────────────────────
    # TAB 3: Adaptive Parameters & Robustness
    # ────────────────────────────────────────────────────────
    def _plot_adaptive_params(self, r):
        """
        2 subplots (2×1):
          (a) Adaptive parameter dα₁ vs dissipative term - Joint 1
          (b) Adaptive parameter dα₂ vs dissipative term - Joint 2
        """
        c = self.canvas_adaptive
        c.clear()
        t = r.t

        # Compute dalpha/dt numerically (derivative of adaptive parameters)
        dt_t = np.diff(t)
        # alpha_adapt is (N, 2)
        dalpha_j1 = np.gradient(r.alpha_adapt[:, 0], t)
        dalpha_j2 = np.gradient(r.alpha_adapt[:, 1], t)

        # Friction/dissipative torque
        if r.friction_torque is not None:
            fric_j1 = r.friction_torque[:, 0]
            fric_j2 = r.friction_torque[:, 1]
        else:
            # Fallback: estimate from velocity
            fric_j1 = 2.5 * r.dq[:, 0]
            fric_j2 = 1.8 * r.dq[:, 1]

        # (a) Joint 1
        ax1 = c.fig.add_subplot(211); c.apply_style(ax1)
        ax1_twin = ax1.twinx()
        ln1 = ax1.plot(t, dalpha_j1, color=Theme.BLUE, lw=1.5,
                        label='dα₁/dt (Adaptation Rate)')
        ln2 = ax1_twin.plot(t, fric_j1, color=Theme.RED, lw=1.2, ls='--',
                             label='Dissipative Term (Friction τ_f₁)')
        ax1.set_ylabel('dα₁/dt', color=Theme.BLUE)
        ax1_twin.set_ylabel('Friction Torque (Nm)', color=Theme.RED)
        ax1.set_title(
            'Adaptive Parameter Behavior (dα₁) vs Dissipative Term — Joint 1',
            fontsize=10
        )
        ax1_twin.tick_params(colors=Theme.MPL_TICK)
        ax1_twin.spines['right'].set_color(Theme.BORDER)
        # Combined legend
        lns = ln1 + ln2
        labs = [l.get_label() for l in lns]
        ax1.legend(lns, labs, facecolor=Theme.BG_LIGHT, edgecolor=Theme.BORDER,
                   labelcolor=Theme.TEXT_PRIMARY, fontsize=9, loc='upper right')

        # (b) Joint 2
        ax2 = c.fig.add_subplot(212, sharex=ax1); c.apply_style(ax2)
        ax2_twin = ax2.twinx()
        ln3 = ax2.plot(t, dalpha_j2, color=Theme.ORANGE, lw=1.5,
                        label='dα₂/dt (Adaptation Rate)')
        ln4 = ax2_twin.plot(t, fric_j2, color=Theme.PURPLE, lw=1.2, ls='--',
                             label='Dissipative Term (Friction τ_f₂)')
        ax2.set_xlabel('Time (s)')
        ax2.set_ylabel('dα₂/dt', color=Theme.ORANGE)
        ax2_twin.set_ylabel('Friction Torque (Nm)', color=Theme.PURPLE)
        ax2.set_title(
            'Adaptive Parameter Behavior (dα₂) vs Dissipative Term — Joint 2',
            fontsize=10
        )
        ax2_twin.tick_params(colors=Theme.MPL_TICK)
        ax2_twin.spines['right'].set_color(Theme.BORDER)
        lns2 = ln3 + ln4
        labs2 = [l.get_label() for l in lns2]
        ax2.legend(lns2, labs2, facecolor=Theme.BG_LIGHT, edgecolor=Theme.BORDER,
                   labelcolor=Theme.TEXT_PRIMARY, fontsize=9, loc='upper right')

        c.fig.suptitle('Adaptive Parameters & Robustness Analysis', fontsize=14,
                       fontweight='bold', color=Theme.MPL_TEXT, y=1.0)
        c.fig.tight_layout()
        c.draw()

    # ────────────────────────────────────────────────────────
    # TAB 4: Gain Variation Optimization (γ)
    # ────────────────────────────────────────────────────────
    def _plot_gain_variation(self, gain_results):
        """
        2 subplots (2×1):
          (a) Effect of gain variation γ on output - Joint 1
          (b) Effect of gain variation γ on output - Joint 2
        """
        c = self.canvas_gain_var
        c.clear()

        colors = ['#58a6ff', '#3fb950', '#f0883e', '#f85149', '#bc8cff']

        # (a) Joint 1
        ax1 = c.fig.add_subplot(211); c.apply_style(ax1)
        for idx, (gamma_val, result) in enumerate(gain_results):
            color = colors[idx % len(colors)]
            ax1.plot(result.t, np.degrees(result.q[:, 0]),
                     color=color, lw=1.3, label=f'γ = {gamma_val}')
        # Plot reference
        ref_result = gain_results[0][1]
        ax1.plot(ref_result.t, np.degrees(ref_result.q_desired[:, 0]),
                 ':', color=Theme.TEXT_MUTED, lw=1.5, label='Target (60°)')
        ax1.set_ylabel('Degrees (°)')
        ax1.set_title('Effect of Adaptive Gain Variation (γ) on Output — Joint 1',
                       fontsize=11)
        self._legend(ax1, loc='lower right', ncol=3)

        # (b) Joint 2
        ax2 = c.fig.add_subplot(212, sharex=ax1); c.apply_style(ax2)
        for idx, (gamma_val, result) in enumerate(gain_results):
            color = colors[idx % len(colors)]
            ax2.plot(result.t, np.degrees(result.q[:, 1]),
                     color=color, lw=1.3, label=f'γ = {gamma_val}')
        ax2.plot(ref_result.t, np.degrees(ref_result.q_desired[:, 1]),
                 ':', color=Theme.TEXT_MUTED, lw=1.5, label='Target (30°)')
        ax2.set_xlabel('Time (s)')
        ax2.set_ylabel('Degrees (°)')
        ax2.set_title('Effect of Adaptive Gain Variation (γ) on Output — Joint 2',
                       fontsize=11)
        self._legend(ax2, loc='lower right', ncol=3)

        c.fig.suptitle('Optimization — Adaptive Gain Variation (γ)', fontsize=14,
                       fontweight='bold', color=Theme.MPL_TEXT, y=1.0)
        c.fig.tight_layout()
        c.draw()

    # ── Export ────────────────────────────────────────────────
    def _export_results(self):
        if not self.sim_result:
            QMessageBox.warning(self, "No Data", "Run a simulation first!")
            return

        folder = QFileDialog.getExistingDirectory(self, "Select Output Folder")
        if not folder:
            return

        try:
            r = self.sim_result
            for canvas, name in [
                (self.canvas_output,   'basic_system_output_graphs'),
                (self.canvas_mrac,     'mrac_performance_analysis_graphs'),
                (self.canvas_adaptive, 'adaptive_params_robustness_graphs'),
                (self.canvas_gain_var, 'gain_variation_optimization_graphs'),
            ]:
                canvas.fig.savefig(
                    os.path.join(folder, f'{name}.png'),
                    dpi=150, facecolor=Theme.MPL_BG, bbox_inches='tight'
                )

            csv_path = os.path.join(folder, 'simulation_data.csv')
            header = ('time,q1_rad,q2_rad,dq1,dq2,'
                     'qm1_rad,qm2_rad,e1_rad,e2_rad,'
                     'tau1_Nm,tau2_Nm,'
                     'alpha1,alpha2,'
                     'ee_x,ee_y,ee_z,'
                     'friction_j1,friction_j2')

            fric_data = r.friction_torque if r.friction_torque is not None else np.zeros((len(r.t), 2))
            data = np.column_stack([r.t, r.q, r.dq, r.q_ref, r.error,
                                    r.tau, r.alpha_adapt, r.end_effector,
                                    fric_data])
            np.savetxt(csv_path, data, delimiter=',', header=header,
                      comments='', fmt='%.8f')

            met_path = os.path.join(folder, 'metrics.txt')
            with open(met_path, 'w', encoding='utf-8') as f:
                f.write("MRAC 2-DoF Simulation Metrics\n" + "=" * 40 + "\n")
                for k, v in r.metrics.items():
                    f.write(f"{k}: {v:.6f}\n")

            QMessageBox.information(self, "Export Complete",
                                  f"Files exported to:\n{folder}")
            self.statusBar().showMessage(f"✓ Exported to {folder}", 5000)
        except Exception as e:
            QMessageBox.critical(self, "Export Error", str(e))


def run_app():
    """Launch the Simulink GUI application."""
    app = QApplication(sys.argv)
    app.setStyle("Fusion")
    app.setStyleSheet(build_stylesheet())

    palette = QPalette()
    palette.setColor(QPalette.ColorRole.Window, QColor(Theme.BG_DARKEST))
    palette.setColor(QPalette.ColorRole.WindowText, QColor(Theme.TEXT_PRIMARY))
    palette.setColor(QPalette.ColorRole.Base, QColor(Theme.BG_DARK))
    palette.setColor(QPalette.ColorRole.AlternateBase, QColor(Theme.BG_MID))
    palette.setColor(QPalette.ColorRole.ToolTipBase, QColor(Theme.BG_LIGHT))
    palette.setColor(QPalette.ColorRole.ToolTipText, QColor(Theme.TEXT_PRIMARY))
    palette.setColor(QPalette.ColorRole.Text, QColor(Theme.TEXT_PRIMARY))
    palette.setColor(QPalette.ColorRole.Button, QColor(Theme.BG_LIGHT))
    palette.setColor(QPalette.ColorRole.ButtonText, QColor(Theme.TEXT_PRIMARY))
    palette.setColor(QPalette.ColorRole.Highlight, QColor(Theme.BLUE_DIM))
    palette.setColor(QPalette.ColorRole.HighlightedText, QColor("white"))
    app.setPalette(palette)

    window = SimulinkGUI()
    window.show()
    sys.exit(app.exec())
