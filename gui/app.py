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
from PySide6.QtGui import QColor, QPalette

from gui.theme import Theme, build_stylesheet
from gui.diagram import BlockDiagramView
from gui.properties import PropertiesPanel
from gui.scope import ScopePlotCanvas
from gui.metrics import MetricsTable
from gui.worker import SimulationWorker

from models.config import default_config


class SimulinkGUI(QMainWindow):
    """Main Simulink-like window for MRAC 2-DoF Satellite Dish."""

    def __init__(self):
        super().__init__()
        self.setWindowTitle("MRAC Simulink — 2-DoF Satellite Dish Antenna Controller")
        self.resize(1440, 900)
        self.sim_config = default_config()
        self.sim_result = None
        self.worker = None

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

        self.canvas_j1 = ScopePlotCanvas()
        self.canvas_j2 = ScopePlotCanvas()
        self.canvas_error = ScopePlotCanvas()
        self.canvas_torque = ScopePlotCanvas()
        self.canvas_adapt = ScopePlotCanvas()
        self.canvas_phase = ScopePlotCanvas()
        self.metrics_table = MetricsTable()

        self.scope_tabs.addTab(self.canvas_j1, "📈 Joint 1 (Azimuth)")
        self.scope_tabs.addTab(self.canvas_j2, "📉 Joint 2 (Elevation)")
        self.scope_tabs.addTab(self.canvas_error, "📊 Tracking Error")
        self.scope_tabs.addTab(self.canvas_torque, "⚡ Torque")
        self.scope_tabs.addTab(self.canvas_adapt, "🔄 Adaptive Params")
        self.scope_tabs.addTab(self.canvas_phase, "🌀 Phase Portrait")
        self.scope_tabs.addTab(self.metrics_table, "📋 Metrics")

        right_layout.addWidget(self.scope_tabs)
        h_splitter.addWidget(right_frame)

        h_splitter.setSizes([320, 1050])
        v_splitter.addWidget(h_splitter)
        v_splitter.setSizes([310, 550])
        main_layout.addWidget(v_splitter)

        self._draw_empty_scopes()

    def _draw_empty_scopes(self):
        canvases = [self.canvas_j1, self.canvas_j2, self.canvas_error,
                    self.canvas_torque, self.canvas_adapt, self.canvas_phase]
        titles = [
            "Joint 1 (Azimuth) Response",
            "Joint 2 (Elevation) Response",
            "Tracking Error e(t)",
            "Control Torque τ(t)",
            "Adaptive Parameters θ(t)",
            "Phase Portrait"
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

        self.worker = SimulationWorker(self.sim_config)
        self.worker.finished.connect(self._on_sim_done)
        self.worker.error_occurred.connect(self._on_sim_error)
        self.worker.start()

    def _on_sim_done(self, result):
        self.sim_result = result
        self.run_btn.setEnabled(True)
        self.export_btn.setEnabled(True)
        self.progress_bar.setVisible(False)
        self.diagram_view.stop_animation()

        self.sim_status_lbl.setText("  ✓ Complete")
        self.sim_status_lbl.setStyleSheet(
            f"color: {Theme.GREEN}; font-weight: bold;"
        )

        m = result.metrics
        self.statusBar().showMessage(
            f"✓ Done T={result.t[-1]:.1f}s | "
            f"SSE: J1={np.degrees(m['ss_error_j1']):.4f}° "
            f"J2={np.degrees(m['ss_error_j2']):.4f}° | "
            f"OS: J1={m['overshoot_j1_pct']:.1f}% "
            f"J2={m['overshoot_j2_pct']:.1f}%"
        )

        self._plot_joint1(result)
        self._plot_joint2(result)
        self._plot_error(result)
        self._plot_torque(result)
        self._plot_adaptive(result)
        self._plot_phase(result)
        self.metrics_table.update_metrics(result.metrics)

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

    def _plot_joint1(self, r):
        c = self.canvas_j1; c.clear()
        ax = c.fig.add_subplot(111); c.apply_style(ax)
        t = r.t
        ax.plot(t, np.degrees(r.q[:, 0]), color=Theme.BLUE, lw=2, label='Actual q₁')
        ax.plot(t, np.degrees(r.q_ref[:, 0]), '--', color=Theme.GREEN, lw=1.5, label='Ref qm₁')
        ax.plot(t, np.degrees(r.q_desired[:, 0]), ':', color=Theme.PURPLE, lw=1, alpha=.7, label='Desired')
        ax.set_xlabel('Time (s)'); ax.set_ylabel('Angle (°)')
        ax.set_title('Joint 1 — Azimuth Response'); ax.set_xlim([t[0], t[-1]])
        self._legend(ax, loc='lower right')
        os_p = r.metrics.get('overshoot_j1_pct', 0)
        ts = r.metrics.get('settling_time_j1', 0)
        self._annot_box(ax, f"Overshoot: {os_p:.1f}%\nSettling: {ts:.2f}s")
        c.fig.tight_layout(); c.draw()

    def _plot_joint2(self, r):
        c = self.canvas_j2; c.clear()
        ax = c.fig.add_subplot(111); c.apply_style(ax)
        t = r.t
        ax.plot(t, np.degrees(r.q[:, 1]), color=Theme.ORANGE, lw=2, label='Actual q₂')
        ax.plot(t, np.degrees(r.q_ref[:, 1]), '--', color=Theme.GREEN, lw=1.5, label='Ref qm₂')
        ax.plot(t, np.degrees(r.q_desired[:, 1]), ':', color=Theme.PURPLE, lw=1, alpha=.7, label='Desired')
        ax.set_xlabel('Time (s)'); ax.set_ylabel('Angle (°)')
        ax.set_title('Joint 2 — Elevation Response'); ax.set_xlim([t[0], t[-1]])
        self._legend(ax, loc='lower right')
        os_p = r.metrics.get('overshoot_j2_pct', 0)
        ts = r.metrics.get('settling_time_j2', 0)
        self._annot_box(ax, f"Overshoot: {os_p:.1f}%\nSettling: {ts:.2f}s")
        c.fig.tight_layout(); c.draw()

    def _plot_error(self, r):
        c = self.canvas_error; c.clear()
        ax1 = c.fig.add_subplot(211); c.apply_style(ax1)
        ax2 = c.fig.add_subplot(212, sharex=ax1); c.apply_style(ax2)
        t = r.t
        e1 = np.degrees(r.error[:, 0]); e2 = np.degrees(r.error[:, 1])
        ax1.plot(t, e1, color=Theme.RED, lw=1.5, label='e₁(t)')
        ax1.fill_between(t, e1, 0, alpha=.15, color=Theme.RED)
        ax1.axhline(0, color=Theme.BORDER, lw=.5)
        ax1.set_ylabel('J1 Error (°)'); ax1.set_title('Tracking Error — e(t) = q − qm')
        self._legend(ax1, loc='upper right')
        self._annot_box(ax1, f"SS Err: {np.degrees(r.metrics.get('ss_error_j1',0)):.4f}°", 'tr')
        ax2.plot(t, e2, color=Theme.ORANGE, lw=1.5, label='e₂(t)')
        ax2.fill_between(t, e2, 0, alpha=.15, color=Theme.ORANGE)
        ax2.axhline(0, color=Theme.BORDER, lw=.5)
        ax2.set_xlabel('Time (s)'); ax2.set_ylabel('J2 Error (°)')
        self._legend(ax2, loc='upper right')
        self._annot_box(ax2, f"SS Err: {np.degrees(r.metrics.get('ss_error_j2',0)):.4f}°", 'tr')
        c.fig.tight_layout(); c.draw()

    def _plot_torque(self, r):
        c = self.canvas_torque; c.clear()
        ax1 = c.fig.add_subplot(211); c.apply_style(ax1)
        ax2 = c.fig.add_subplot(212, sharex=ax1); c.apply_style(ax2)
        t = r.t
        ax1.plot(t, r.tau[:, 0], color=Theme.BLUE, lw=1.2, label='τ₁')
        ax1.fill_between(t, r.tau[:, 0], 0, alpha=.1, color=Theme.BLUE)
        ax1.set_ylabel('τ₁ (Nm)'); ax1.set_title('Control Torque Profile')
        self._legend(ax1, loc='upper right')
        self._annot_box(ax1, f"Max |τ₁|: {r.metrics.get('max_torque_j1',0):.1f} Nm", 'tr')
        ax2.plot(t, r.tau[:, 1], color=Theme.ORANGE, lw=1.2, label='τ₂')
        ax2.fill_between(t, r.tau[:, 1], 0, alpha=.1, color=Theme.ORANGE)
        ax2.set_xlabel('Time (s)'); ax2.set_ylabel('τ₂ (Nm)')
        self._legend(ax2, loc='upper right')
        self._annot_box(ax2, f"Max |τ₂|: {r.metrics.get('max_torque_j2',0):.1f} Nm", 'tr')
        c.fig.tight_layout(); c.draw()

    def _plot_adaptive(self, r):
        c = self.canvas_adapt; c.clear()
        ax1 = c.fig.add_subplot(211); c.apply_style(ax1)
        ax2 = c.fig.add_subplot(212, sharex=ax1); c.apply_style(ax2)
        t = r.t; names = ['θ_r', 'θ_q', 'θ_dq']
        colors = [Theme.BLUE, Theme.ORANGE, Theme.PURPLE]
        for p in range(3):
            ax1.plot(t, r.theta_adapt[:, p], color=colors[p], lw=1.5, label=names[p])
        ax1.set_ylabel('θ'); ax1.set_title('Adaptive Parameters — Joint 1')
        self._legend(ax1, loc='upper right', ncol=3)
        for p in range(3):
            ax2.plot(t, r.theta_adapt[:, 3+p], color=colors[p], lw=1.5, label=names[p])
        ax2.set_xlabel('Time (s)'); ax2.set_ylabel('θ')
        ax2.set_title('Adaptive Parameters — Joint 2')
        self._legend(ax2, loc='upper right', ncol=3)
        c.fig.tight_layout(); c.draw()

    def _plot_phase(self, r):
        c = self.canvas_phase; c.clear()
        ax1 = c.fig.add_subplot(121); c.apply_style(ax1)
        ax2 = c.fig.add_subplot(122); c.apply_style(ax2)
        de = np.gradient(r.error, r.t, axis=0)
        for idx, (ax, tit) in enumerate(zip([ax1, ax2], ['Joint 1', 'Joint 2'])):
            ed = np.degrees(r.error[:, idx])
            ded = np.degrees(de[:, idx])
            sc = ax.scatter(ed, ded, c=r.t, cmap='cool', s=2, alpha=.6)
            ax.plot(ed[0], ded[0], 'o', color=Theme.GREEN, ms=8, label='Start', zorder=5)
            ax.plot(ed[-1], ded[-1], 's', color=Theme.RED, ms=8, label='End', zorder=5)
            ax.set_xlabel(f'e{idx+1} (°)'); ax.set_ylabel(f'ė{idx+1} (°/s)')
            ax.set_title(tit)
            self._legend(ax, loc='upper right')
            c.fig.colorbar(sc, ax=ax, label='Time (s)', shrink=.8)
        c.fig.tight_layout(); c.draw()

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
                (self.canvas_j1, 'joint1_response'),
                (self.canvas_j2, 'joint2_response'),
                (self.canvas_error, 'tracking_error'),
                (self.canvas_torque, 'torque_profile'),
                (self.canvas_adapt, 'adaptive_params'),
                (self.canvas_phase, 'phase_portrait'),
            ]:
                canvas.fig.savefig(
                    os.path.join(folder, f'{name}.png'),
                    dpi=150, facecolor=Theme.MPL_BG, bbox_inches='tight'
                )

            csv_path = os.path.join(folder, 'simulation_data.csv')
            header = ('time,q1_rad,q2_rad,dq1,dq2,'
                     'qm1_rad,qm2_rad,e1_rad,e2_rad,'
                     'tau1_Nm,tau2_Nm,'
                     'theta_r1,theta_q1,theta_dq1,'
                     'theta_r2,theta_q2,theta_dq2,'
                     'ee_x,ee_y,ee_z')
            data = np.column_stack([r.t, r.q, r.dq, r.q_ref, r.error,
                                    r.tau, r.theta_adapt, r.end_effector])
            np.savetxt(csv_path, data, delimiter=',', header=header,
                      comments='', fmt='%.8f')

            met_path = os.path.join(folder, 'metrics.txt')
            with open(met_path, 'w', encoding='utf-8') as f:
                f.write("MRAC 2-DoF Simulation Metrics\n" + "=" * 40 + "\n")
                for k, v in r.metrics.items():
                    f.write(f"{k}: {v:.6f}\n")

            QMessageBox.information(self, "Export Complete",
                                  f"8 files exported to:\n{folder}")
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
