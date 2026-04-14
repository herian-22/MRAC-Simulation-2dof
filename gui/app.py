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


def resource_path(relative_path):
    """ Get absolute path to resource, works for dev and for PyInstaller """
    try:
        # PyInstaller creates a temp folder and stores path in _MEIPASS
        base_path = sys._MEIPASS
    except Exception:
        base_path = os.path.dirname(os.path.dirname(__file__))
    return os.path.join(base_path, relative_path)

class SimulinkGUI(QMainWindow):
    """Main Simulink-like window for MRAC 2-DoF Satellite Dish."""

    def __init__(self):
        super().__init__()
        self.setWindowTitle("MRAC Simulation — 2-DoF Satellite Dish Receiver")
        
        icon_path = resource_path("mrac_icon.ico")
        if os.path.exists(icon_path):
            self.setWindowIcon(QIcon(icon_path))
            
        self.resize(1280, 720)
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

        title_lbl = QLabel("  MRAC 2-DoF Satellite Dish Receiver  ")
        title_lbl.setStyleSheet(
            f"color: {Theme.BLUE}; font-size: 16px; font-weight: bold;"
        )
        toolbar.addWidget(title_lbl)
        toolbar.addSeparator()

        self.run_btn = QPushButton("  ▶  Run Simulation  ")
        self.run_btn.setObjectName("runBtn")
        self.run_btn.clicked.connect(self._run_simulation)
        toolbar.addWidget(self.run_btn)

        self.export_btn = QPushButton(" Export Results  ")
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

        # === TOP SECTION: Diagram + Properties ===
        top_h_splitter = QSplitter(Qt.Orientation.Horizontal)
        
        # Left: block diagram
        self.diagram_view = BlockDiagramView(self._on_block_clicked)
        self.diagram_view.setMinimumHeight(250)
        top_h_splitter.addWidget(self.diagram_view)

        # Right: properties panel
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
        top_h_splitter.addWidget(props_frame)
        
        top_h_splitter.setSizes([800, 350])
        v_splitter.addWidget(top_h_splitter)

        # === BOTTOM SECTION: Main Tabs (Visualizer & Scopes) ===
        self.scope_tabs = QTabWidget()
        self.scope_tabs.setDocumentMode(True)

        # --- Tab 1: 3D Visualizer ---
        viz_frame = QFrame()
        viz_layout = QVBoxLayout(viz_frame)
        viz_layout.setContentsMargins(0, 0, 0, 0)
        self.visualizer_3d = RobotArmVisualizer()
        viz_layout.addWidget(self.visualizer_3d)

        # --- Tab 2-7: Scopes ---
        self.canvas_output = ScopePlotCanvas()
        self.canvas_mrac = ScopePlotCanvas()
        self.canvas_adaptive = ScopePlotCanvas()
        self.canvas_gain_var = ScopePlotCanvas()
        self.canvas_cartesian = ScopePlotCanvas()
        self.metrics_table = MetricsTable()

        self.scope_tabs.addTab(viz_frame, "3D Visualizer Simulation")
        self.scope_tabs.addTab(self.canvas_output,   "Basic System Output")
        self.scope_tabs.addTab(self.canvas_mrac,     "MRAC Performance Analysis")
        self.scope_tabs.addTab(self.canvas_adaptive, "Adaptive Parameters")
        self.scope_tabs.addTab(self.canvas_gain_var, "Optimization: Gain Variation")
        self.scope_tabs.addTab(self.canvas_cartesian,"Cartesian (Z, Y, X)")
        self.scope_tabs.addTab(self.metrics_table,   "Metrics")

        v_splitter.addWidget(self.scope_tabs)
        v_splitter.setSizes([350, 650])
        main_layout.addWidget(v_splitter)

        self._draw_empty_scopes()
        self.visualizer_3d.build_scene(self.sim_config)

    def _draw_empty_scopes(self):
        canvases = [self.canvas_output, self.canvas_mrac,
                    self.canvas_adaptive, self.canvas_gain_var,
                    self.canvas_cartesian]
        titles = [
            "Basic System Output — Angular Displacement & Velocity",
            "MRAC Performance Analysis — Controlled Variable vs Reference Model",
            "Adaptive Parameters & Robustness Analysis",
            "Optimization — Effect of Adaptive Gain Variation (γ)",
            "Cartesian Trajectory (Z, Y, X Translations)"
        ]
        for canvas, title in zip(canvases, titles):
            ax = canvas.fig.add_subplot(111)
            canvas.apply_style(ax)
            ax.set_title(title, fontsize=9)
            ax.text(0.5, 0.5, "▶  Run simulation to view results",
                    transform=ax.transAxes, ha='center', va='center',
                    fontsize=9, color=Theme.TEXT_MUTED, style='italic')
            try:
                canvas.fig.tight_layout()
            except Exception:
                pass
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

        # Plot tabs 1-5 immediately
        self._plot_output_dasar(result)
        self._plot_mrac_analysis(result)
        self._plot_adaptive_params(result)
        self._plot_cartesian_trajectory(result)
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
                transform=ax.transAxes, ha=ha, va=va, fontsize=6,
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
        q1_deg = np.degrees(r.q[:, 0])
        q1_target = np.degrees(r.q_desired[-1, 0])
        ax1.plot(t, q1_deg, color=Theme.BLUE, lw=1.8,
                 label='Output q₁ (°)')
        ax1.plot(t, np.degrees(r.q_desired[:, 0]), ':', color=Theme.PURPLE,
                 lw=1.2, alpha=.7, label=f'Reference ({q1_target:.1f}°)')
        ax1.axhline(q1_target, color=Theme.GREEN, lw=0.8, ls='--', alpha=0.5)
        ax1.set_ylabel('Degrees (°)')
        ax1.set_title('Angular Displacement — Joint 1', fontsize=6)
        self._legend(ax1, loc='lower right')

        # (b) Angular Velocity - Joint 1
        ax2 = c.fig.add_subplot(222); c.apply_style(ax2)
        ax2.plot(t, np.degrees(r.dq[:, 0]), color=Theme.RED, lw=1.5,
                 label='dq₁ (°/s)')
        ax2.axhline(0, color=Theme.BORDER, lw=.5)
        ax2.set_ylabel('Degrees/s (°/s)')
        ax2.set_title('Angular Velocity — Joint 1', fontsize=6)
        self._legend(ax2, loc='upper right')

        # (c) Angular Displacement - Joint 2
        ax3 = c.fig.add_subplot(223); c.apply_style(ax3)
        q2_deg = np.degrees(r.q[:, 1])
        q2_target = np.degrees(r.q_desired[-1, 1])
        ax3.plot(t, q2_deg, color=Theme.ORANGE, lw=1.8,
                 label='Output q₂ (°)')
        ax3.plot(t, np.degrees(r.q_desired[:, 1]), ':', color=Theme.PURPLE,
                 lw=1.2, alpha=.7, label=f'Reference ({q2_target:.1f}°)')
        ax3.axhline(q2_target, color=Theme.GREEN, lw=0.8, ls='--', alpha=0.5)
        ax3.set_xlabel('Time (s)')
        ax3.set_ylabel('Degrees (°)')
        ax3.set_title('Angular Displacement — Joint 2', fontsize=6)
        self._legend(ax3, loc='lower right')

        # (d) Angular Velocity - Joint 2
        ax4 = c.fig.add_subplot(224); c.apply_style(ax4)
        ax4.plot(t, np.degrees(r.dq[:, 1]), color=Theme.CYAN, lw=1.5,
                 label='dq₂ (°/s)')
        ax4.axhline(0, color=Theme.BORDER, lw=.5)
        ax4.set_xlabel('Time (s)')
        ax4.set_ylabel('Degrees/s (°/s)')
        ax4.set_title('Angular Velocity — Joint 2', fontsize=6)
        self._legend(ax4, loc='upper right')

        c.fig.suptitle('Basic System Output Graphs', fontsize=8,
                       fontweight='bold', color=Theme.MPL_TEXT, y=1.0)
        try:
            c.fig.tight_layout()
        except Exception:
            pass
        c.draw()

    def _plot_cartesian_trajectory(self, r):
        """
        Draws a 2D Geometric Schematic of the robot's Forward Kinematics,
        illustrating frames, translation vectors, and rotation (Z, Y, X).
        Uses the final steady-state position from the simulation.
        """
        a1, a2, d1 = 0.192, 0.936, 1.601
        
        # Take final steady-state joint angles
        q1 = r.q[-1, 0]
        q2 = r.q[-1, 1]
        
        # Compute joint positions
        P0 = np.array([0, 0, 0])
        P1 = np.array([0, 0, d1])
        
        c1, s1 = np.cos(q1), np.sin(q1)
        P2 = P1 + np.array([a1*c1, a1*s1, 0])
        
        c2, s2 = np.cos(q2), np.sin(q2)
        P3 = P2 + np.array([a2*c1*c2, a2*s1*c2, a2*s2])

        c = self.canvas_cartesian
        c.fig.clf()

        # ─── 1. TOP VIEW (XY Plane) - Showing Azimuth & X/Y Translation ───
        ax1 = c.fig.add_subplot(121)
        c.apply_style(ax1)
        ax1.set_title("Top View (X-Y Plane) — Azimuth $q_1$", fontsize=6, fontweight='bold')
        
        # Robot Links
        ax1.plot([P0[0], P2[0]], [P0[1], P2[1]], color='#7f8c8d', lw=4, label='Link 1 Offset')
        ax1.plot([P2[0], P3[0]], [P2[1], P3[1]], color='#ffaa00', lw=3, label='Link 2 (Dish)')
        
        # Base Frame (O_A)
        sc = 0.4
        ax1.quiver(0, 0, sc, 0, color='#ff4444', scale=0.5, scale_units='xy', width=0.015)
        ax1.quiver(0, 0, 0, sc, color='#44cc44', scale=0.5, scale_units='xy', width=0.015)
        ax1.text(sc*1.1, 0, 'X₀', color='#ff4444', fontweight='bold')
        ax1.text(0, sc*1.1, 'Y₀', color='#44cc44', fontweight='bold')
        
        # Joint Frame (O_B) at P2
        ax1.quiver(P2[0], P2[1], sc*c1, sc*s1, color='#ff4444', scale=0.5, scale_units='xy', width=0.01)
        ax1.quiver(P2[0], P2[1], -sc*s1, sc*c1, color='#44cc44', scale=0.5, scale_units='xy', width=0.01)
        ax1.text(P2[0]+sc*c1*1.1, P2[1]+sc*s1*1.1, "X₁'", color='#ff4444')
        ax1.text(P2[0]-sc*s1*1.1, P2[1]+sc*c1*1.1, "Y₁'", color='#44cc44')
        
        # Translation Vector O_A -> P3
        ax1.annotate("", xy=(P3[0], P3[1]), xytext=(0, 0),
                     arrowprops=dict(arrowstyle="->", color="#9b59b6", lw=2, ls="--"))
        ax1.text(P3[0]/2, P3[1]/2 + 0.1, "Translasi XY", color="#9b59b6", fontweight='bold')
        
        ax1.plot(P3[0], P3[1], 'ro', ms=6, zorder=5)
        ax1.text(P3[0]+0.05, P3[1]+0.05, "EE (P)", fontweight='bold')
        ax1.axis('equal')
        ax1.set_xlabel("X (m)")
        ax1.set_ylabel("Y (m)")
        self._legend(ax1, loc='lower right')

        # ─── 2. SIDE VIEW (Radius-Z Plane) - Showing Elevation & Z Translasi ───
        ax2 = c.fig.add_subplot(122)
        c.apply_style(ax2)
        ax2.set_title("Side View (Radius-Z Plane) — Elevation $q_2$", fontsize=6, fontweight='bold')
        
        # Calculate horizontal radius
        R0, R1 = 0, 0
        R2 = np.sqrt(P2[0]**2 + P2[1]**2)
        R3 = np.sqrt(P3[0]**2 + P3[1]**2)
        
        # Robot Links
        ax2.plot([R0, R1], [P0[2], P1[2]], color='#34495e', lw=5, label='Base Pillar')
        ax2.plot([R1, R2], [P1[2], P2[2]], color='#7f8c8d', lw=4)
        ax2.plot([R2, R3], [P2[2], P3[2]], color='#ffaa00', lw=3, label='Link 2 (Dish)')
        
        # Base axes (Radius & Z)
        ax2.quiver(0, 0, sc, 0, color='#ff4444', scale=0.5, scale_units='xy', width=0.015)
        ax2.quiver(0, 0, 0, sc, color='#4488ff', scale=0.5, scale_units='xy', width=0.015)
        ax2.text(sc*1.1, 0, 'Radius (XY)', color='#ff4444', fontweight='bold')
        ax2.text(0, sc*1.1, 'Z₀', color='#4488ff', fontweight='bold')
        
        # Elevation Frame at Joint 2
        ax2.quiver(R2, P2[2], sc*c2, sc*s2, color='#ff4444', scale=1, scale_units='xy', width=0.01)
        ax2.quiver(R2, P2[2], -sc*s2, sc*c2, color='#4488ff', scale=1, scale_units='xy', width=0.01)
        ax2.text(R2+sc*c2*1.1, P2[2]+sc*s2*1.1, "X₂'", color='#ff4444')
        ax2.text(R2-sc*s2*1.1, P2[2]+sc*c2*1.1, "Z₂'", color='#4488ff')
        
        # Translation Vector Base -> EE
        ax2.annotate("", xy=(R3, P3[2]), xytext=(0, 0),
                     arrowprops=dict(arrowstyle="->", color="#9b59b6", lw=2, ls="--"))
        ax2.text(R3/2 - 0.2, (P3[2])/2 + 0.1, "Translasi Total P", color="#9b59b6", fontweight='bold')

        ax2.plot(R3, P3[2], 'ro', ms=6, zorder=5)
        ax2.text(R3+0.05, P3[2]+0.05, "EE (P)", fontweight='bold')
        
        ax2.axis('equal')
        ax2.set_xlabel("Horizontal Radius (m)")
        ax2.set_ylabel("Z Height (m)")
        self._legend(ax2, loc='lower right')

        # === FINAL VALUES TABLE OUTSIDE PLOT ===
        text_table = (
            f"  FINAL STEADY-STATE VALUES\n"
            f"────────────────────────────────\n"
            f" Azimuth (q₁)   : {np.degrees(q1):>7.2f}°\n"
            f" Elevation (q₂) : {np.degrees(q2):>7.2f}°\n"
            f" Translation X  : {P3[0]:>7.3f} m\n"
            f" Translation Y  : {P3[1]:>7.3f} m\n"
            f" Translation Z  : {P3[2]:>7.3f} m\n"
            f" Total Distance : {np.linalg.norm(P3):>7.3f} m"
        )
        props = dict(boxstyle='round,pad=0.5', facecolor='#f8f9fa', alpha=0.9, edgecolor='#bdc3c7')
        ax2.text(1.05, 0.96, text_table, transform=ax2.transAxes, fontsize=8,
                 verticalalignment='top', bbox=props, family='monospace')

        c.fig.suptitle('Geometric Transformation Schematic (Vectors & Coordinate Frames)', 
                       fontsize=9, fontweight='bold', color=Theme.MPL_TEXT, y=0.95)
        try:
            # Leave empty space on the right (20% of width) to display the table outside
            c.fig.tight_layout(rect=[0, 0, 1.1, 1])
        except Exception:
            pass
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
        ax1.set_title('Controlled Variable vs Reference Model — Joint 1', fontsize=6)
        self._legend(ax1, loc='lower right')

        # (b) Controlled Variable vs Reference Model - Joint 2
        ax2 = c.fig.add_subplot(222); c.apply_style(ax2)
        ax2.plot(t, np.degrees(r.q[:, 1]), color=Theme.ORANGE, lw=2,
                 label='Controlled Variable (q₂)')
        ax2.plot(t, np.degrees(r.q_ref[:, 1]), '--', color=Theme.GREEN, lw=1.5,
                 label='Reference Model (qm₂)')
        ax2.set_ylabel('Degrees (°)')
        ax2.set_title('Controlled Variable vs Reference Model — Joint 2', fontsize=6)
        self._legend(ax2, loc='lower right')

        # (c) Error Signal - Joint 1
        ax3 = c.fig.add_subplot(223); c.apply_style(ax3)
        e1 = np.degrees(r.error[:, 0])
        ax3.plot(t, e1, color=Theme.RED, lw=1.5, label='e₁ = q₁ - qm₁')
        ax3.fill_between(t, e1, 0, color=Theme.RED, alpha=0.1)
        ax3.axhline(0, color=Theme.BORDER, lw=.5)
        ax3.set_xlabel('Time (s)')
        ax3.set_ylabel('Error (°)')
        ax3.set_title('Error Signal — Joint 1', fontsize=6)
        self._legend(ax3, loc='upper right')

        # (d) Error Signal - Joint 2
        ax4 = c.fig.add_subplot(224); c.apply_style(ax4)
        e2 = np.degrees(r.error[:, 1])
        ax4.plot(t, e2, color=Theme.ORANGE, lw=1.5, label='e₂ = q₂ - qm₂')
        ax4.fill_between(t, e2, 0, color=Theme.ORANGE, alpha=0.1)
        ax4.axhline(0, color=Theme.BORDER, lw=.5)
        ax4.set_xlabel('Time (s)')
        ax4.set_ylabel('Error (°)')
        ax4.set_title('Error Signal — Joint 2', fontsize=6)
        self._legend(ax4, loc='upper right')

        c.fig.suptitle('MRAC Performance Analysis', fontsize=8,
                       fontweight='bold', color=Theme.MPL_TEXT, y=1.0)
        try:
            c.fig.tight_layout()
        except Exception:
            pass
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
            fontsize=6
        )
        ax1_twin.tick_params(colors=Theme.MPL_TICK)
        ax1_twin.spines['right'].set_color(Theme.BORDER)
        # Combined legend
        lns = ln1 + ln2
        labs = [l.get_label() for l in lns]
        ax1.legend(lns, labs, facecolor=Theme.BG_LIGHT, edgecolor=Theme.BORDER,
                   labelcolor=Theme.TEXT_PRIMARY, fontsize=6, loc='upper right')

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
            fontsize=6
        )
        ax2_twin.tick_params(colors=Theme.MPL_TICK)
        ax2_twin.spines['right'].set_color(Theme.BORDER)
        lns2 = ln3 + ln4
        labs2 = [l.get_label() for l in lns2]
        ax2.legend(lns2, labs2, facecolor=Theme.BG_LIGHT, edgecolor=Theme.BORDER,
                   labelcolor=Theme.TEXT_PRIMARY, fontsize=6, loc='upper right')

        c.fig.suptitle('Adaptive Parameters & Robustness Analysis', fontsize=8,
                       fontweight='bold', color=Theme.MPL_TEXT, y=1.0)
        try:
            c.fig.tight_layout()
        except Exception:
            pass
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
        q1_target = np.degrees(ref_result.q_desired[-1, 0])
        ax1.plot(ref_result.t, np.degrees(ref_result.q_desired[:, 0]),
                 ':', color=Theme.TEXT_MUTED, lw=1.5, label=f'Target ({q1_target:.1f}°)')
        ax1.set_ylabel('Degrees (°)')
        ax1.set_title('Effect of Adaptive Gain Variation (γ) on Output — Joint 1',
                       fontsize=6)
        self._legend(ax1, loc='lower right', ncol=3)

        # (b) Joint 2
        ax2 = c.fig.add_subplot(212, sharex=ax1); c.apply_style(ax2)
        for idx, (gamma_val, result) in enumerate(gain_results):
            color = colors[idx % len(colors)]
            ax2.plot(result.t, np.degrees(result.q[:, 1]),
                     color=color, lw=1.3, label=f'γ = {gamma_val}')
        q2_target = np.degrees(ref_result.q_desired[-1, 1])
        ax2.plot(ref_result.t, np.degrees(ref_result.q_desired[:, 1]),
                 ':', color=Theme.TEXT_MUTED, lw=1.5, label=f'Target ({q2_target:.1f}°)')
        ax2.set_xlabel('Time (s)')
        ax2.set_ylabel('Degrees (°)')
        ax2.set_title('Effect of Adaptive Gain Variation (γ) on Output — Joint 2',
                       fontsize=6)
        self._legend(ax2, loc='lower right', ncol=3)

        c.fig.suptitle('Optimization — Adaptive Gain Variation (γ)', fontsize=8,
                       fontweight='bold', color=Theme.MPL_TEXT, y=1.0)
        try:
            c.fig.tight_layout()
        except Exception:
            pass
        c.draw()

    # ── Export ────────────────────────────────────────────────
    def _save_individual_subplot(self, original_canvas, ax_index, nrows, ncols,
                                  filepath, title=None):
        """Re-render a single subplot from a canvas into its own figure file."""
        from matplotlib.figure import Figure
        from matplotlib.backends.backend_agg import FigureCanvasAgg

        src_ax = original_canvas.fig.axes[ax_index]

        fig = Figure(figsize=(8, 5), dpi=180)
        fig.patch.set_facecolor(Theme.MPL_BG)
        canvas_agg = FigureCanvasAgg(fig)
        ax = fig.add_subplot(111)

        # Copy style
        ax.set_facecolor(Theme.MPL_AXES)
        ax.tick_params(colors=Theme.MPL_TICK)
        ax.xaxis.label.set_color(Theme.MPL_TEXT)
        ax.yaxis.label.set_color(Theme.MPL_TEXT)
        ax.title.set_color(Theme.MPL_TEXT)
        ax.title.set_fontweight('bold')
        for spine in ax.spines.values():
            spine.set_color(Theme.BORDER)
        ax.grid(True, color=Theme.MPL_GRID, alpha=0.6, linewidth=0.5)

        # Copy all lines
        for line in src_ax.get_lines():
            ax.plot(line.get_xdata(), line.get_ydata(),
                    color=line.get_color(), lw=line.get_linewidth(),
                    ls=line.get_linestyle(), alpha=line.get_alpha(),
                    label=line.get_label())

        # Copy fill_between (PolyCollections)
        for coll in src_ax.collections:
            try:
                paths = coll.get_paths()
                if paths:
                    fc = coll.get_facecolor()
                    alpha = coll.get_alpha()
                    ax.fill_between([], [], alpha=alpha if alpha else 0.1)
                    # Re-add the poly
                    from matplotlib.collections import PolyCollection
                    import copy
                    new_coll = PolyCollection(
                        [p.vertices for p in paths],
                        facecolors=fc,
                        edgecolors=coll.get_edgecolor(),
                        alpha=alpha if alpha else 0.1
                    )
                    ax.add_collection(new_coll)
            except Exception:
                pass

        ax.set_xlim(src_ax.get_xlim())
        ax.set_ylim(src_ax.get_ylim())
        ax.set_xlabel(src_ax.get_xlabel())
        ax.set_ylabel(src_ax.get_ylabel())
        ax.set_title(title if title else src_ax.get_title(), fontsize=6)

        # Copy legend
        handles, labels = src_ax.get_legend_handles_labels()
        if labels:
            ax.legend(facecolor=Theme.BG_LIGHT, edgecolor=Theme.BORDER,
                      labelcolor=Theme.TEXT_PRIMARY, fontsize=9)

        try:
            fig.tight_layout()
        except Exception:
            pass
        fig.savefig(filepath, dpi=180, facecolor=Theme.MPL_BG,
                    bbox_inches='tight')

    def _export_results(self):
        if not self.sim_result:
            QMessageBox.warning(self, "No Data", "Run a simulation first!")
            return

        folder = QFileDialog.getExistingDirectory(self, "Select Output Folder")
        if not folder:
            return

        try:
            r = self.sim_result

            # ── Create subfolders ──
            graphs_dir = os.path.join(folder, 'graphs_combined')
            plots_dir = os.path.join(folder, 'individual_plots')
            data_dir = os.path.join(folder, 'data')
            for d in [graphs_dir, plots_dir, data_dir]:
                os.makedirs(d, exist_ok=True)

            # ═══════════════════════════════════════════════════
            # 1) Combined figures (4 canvas PNGs)
            # ═══════════════════════════════════════════════════
            for canvas, name in [
                (self.canvas_output,   'basic_system_output_graphs'),
                (self.canvas_mrac,     'mrac_performance_analysis_graphs'),
                (self.canvas_adaptive, 'adaptive_params_robustness_graphs'),
                (self.canvas_gain_var, 'gain_variation_optimization_graphs'),
            ]:
                canvas.fig.savefig(
                    os.path.join(graphs_dir, f'{name}.png'),
                    dpi=180, facecolor=Theme.MPL_BG, bbox_inches='tight'
                )

            # ═══════════════════════════════════════════════════
            # 2) Individual subplot exports
            # ═══════════════════════════════════════════════════

            # Tab 1: Basic System Output (4 subplots, 2×2)
            tab1_plots = [
                (0, 'angular_displacement_joint1', 'Angular Displacement — Joint 1'),
                (1, 'angular_velocity_joint1',     'Angular Velocity — Joint 1'),
                (2, 'angular_displacement_joint2', 'Angular Displacement — Joint 2'),
                (3, 'angular_velocity_joint2',     'Angular Velocity — Joint 2'),
            ]
            for ax_idx, fname, title in tab1_plots:
                if ax_idx < len(self.canvas_output.fig.axes):
                    self._save_individual_subplot(
                        self.canvas_output, ax_idx, 2, 2,
                        os.path.join(plots_dir, f'{fname}.png'), title
                    )

            # Tab 2: MRAC Performance Analysis (4 subplots, 2×2)
            tab2_plots = [
                (0, 'controlled_vs_reference_joint1', 'Controlled Variable vs Reference Model — Joint 1'),
                (1, 'controlled_vs_reference_joint2', 'Controlled Variable vs Reference Model — Joint 2'),
                (2, 'error_signal_joint1',            'Error Signal — Joint 1'),
                (3, 'error_signal_joint2',            'Error Signal — Joint 2'),
            ]
            for ax_idx, fname, title in tab2_plots:
                if ax_idx < len(self.canvas_mrac.fig.axes):
                    self._save_individual_subplot(
                        self.canvas_mrac, ax_idx, 2, 2,
                        os.path.join(plots_dir, f'{fname}.png'), title
                    )

            # Tab 3: Adaptive Parameters (2 subplots with twin axes → 4 axes total)
            # The main axes are at index 0 and 2 (twins are at 1 and 3)
            tab3_axes = self.canvas_adaptive.fig.axes
            if len(tab3_axes) >= 2:
                # For twin-axis plots, export the whole subplot pair as one figure
                self._export_twin_axis_subplot(
                    self.canvas_adaptive, main_idx=0,
                    filepath=os.path.join(plots_dir, 'adaptive_parameter_joint1.png'),
                    title='Adaptive Parameter (dα₁) vs Dissipative — Joint 1'
                )
            if len(tab3_axes) >= 4:
                self._export_twin_axis_subplot(
                    self.canvas_adaptive, main_idx=2,
                    filepath=os.path.join(plots_dir, 'adaptive_parameter_joint2.png'),
                    title='Adaptive Parameter (dα₂) vs Dissipative — Joint 2'
                )

            # Tab 4: Gain Variation (2 subplots, 2×1)
            tab4_axes = self.canvas_gain_var.fig.axes
            tab4_plots = [
                (0, 'gain_variation_joint1', 'Gain Variation (γ) — Joint 1'),
                (1, 'gain_variation_joint2', 'Gain Variation (γ) — Joint 2'),
            ]
            for ax_idx, fname, title in tab4_plots:
                if ax_idx < len(tab4_axes):
                    self._save_individual_subplot(
                        self.canvas_gain_var, ax_idx, 2, 1,
                        os.path.join(plots_dir, f'{fname}.png'), title
                    )

            # ═══════════════════════════════════════════════════
            # 3) 3D Visualizer screenshot
            # ═══════════════════════════════════════════════════
            try:
                viz_path = os.path.join(plots_dir, '3d_robot_arm_visualizer.png')
                self.visualizer_3d.plotter.screenshot(viz_path)
            except Exception:
                pass  # 3D capture may fail if not rendered yet

            # ═══════════════════════════════════════════════════
            # 4) Simulation data CSV
            # ═══════════════════════════════════════════════════
            csv_path = os.path.join(data_dir, 'simulation_data.csv')
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

            # ═══════════════════════════════════════════════════
            # 5) Metrics text + CSV
            # ═══════════════════════════════════════════════════
            met_path = os.path.join(data_dir, 'metrics.txt')
            with open(met_path, 'w', encoding='utf-8') as f:
                f.write("MRAC 2-DoF Simulation Metrics\n" + "=" * 40 + "\n")
                for k, v in r.metrics.items():
                    f.write(f"{k}: {v:.6f}\n")

            met_csv_path = os.path.join(data_dir, 'metrics.csv')
            with open(met_csv_path, 'w', encoding='utf-8') as f:
                f.write("metric,value\n")
                for k, v in r.metrics.items():
                    f.write(f"{k},{v:.8f}\n")

            # ═══════════════════════════════════════════════════
            # 6) Configuration parameters
            # ═══════════════════════════════════════════════════
            cfg_path = os.path.join(data_dir, 'simulation_config.txt')
            c = self.sim_config
            with open(cfg_path, 'w', encoding='utf-8') as f:
                f.write("MRAC 2-DoF Simulation Configuration\n" + "=" * 40 + "\n\n")
                f.write(f"[Controller]\n")
                f.write(f"  gamma1 = {c.controller.gamma1}\n")
                f.write(f"  gamma2 = {c.controller.gamma2}\n\n")
                f.write(f"[Reference Model]\n")
                f.write(f"  omega_n = {c.reference.omega_n}\n")
                f.write(f"  zeta    = {c.reference.zeta}\n\n")
                f.write(f"[Simulation]\n")
                f.write(f"  t_end = {c.simulation.t_end}\n")
                f.write(f"  dt    = {c.simulation.dt}\n\n")
                f.write(f"[Physical]\n")
                f.write(f"  m1 = {c.physical.m1}\n")
                f.write(f"  m2 = {c.physical.m2}\n")
                f.write(f"  a1 = {c.physical.a1}\n")
                f.write(f"  a2 = {c.physical.a2}\n")
                f.write(f"  d1 = {c.physical.d1}\n")

            # Count exported files
            total_files = 0
            for d in [graphs_dir, plots_dir, data_dir]:
                total_files += len([f for f in os.listdir(d) if os.path.isfile(os.path.join(d, f))])

            QMessageBox.information(
                self, "Export Complete",
                f"✓ {total_files} files exported to:\n{folder}\n\n"
                f"📁 graphs_combined/  — Combined figure PNGs\n"
                f"📁 individual_plots/ — Individual subplot PNGs + 3D\n"
                f"📁 data/             — CSV, metrics, config"
            )
            self.statusBar().showMessage(f"✓ Exported {total_files} files to {folder}", 5000)
        except Exception as e:
            QMessageBox.critical(self, "Export Error", str(e))

    def _export_twin_axis_subplot(self, original_canvas, main_idx, filepath, title):
        """Export a twin-axis subplot (main ax + its twin) as a standalone figure."""
        from matplotlib.figure import Figure
        from matplotlib.backends.backend_agg import FigureCanvasAgg

        all_axes = original_canvas.fig.axes
        main_ax = all_axes[main_idx]
        twin_ax = all_axes[main_idx + 1] if (main_idx + 1) < len(all_axes) else None

        fig = Figure(figsize=(8, 5), dpi=180)
        fig.patch.set_facecolor(Theme.MPL_BG)
        FigureCanvasAgg(fig)
        ax = fig.add_subplot(111)

        # Style
        ax.set_facecolor(Theme.MPL_AXES)
        ax.tick_params(colors=Theme.MPL_TICK)
        ax.xaxis.label.set_color(Theme.MPL_TEXT)
        ax.yaxis.label.set_color(Theme.MPL_TEXT)
        ax.title.set_color(Theme.MPL_TEXT)
        ax.title.set_fontweight('bold')
        for spine in ax.spines.values():
            spine.set_color(Theme.BORDER)
        ax.grid(True, color=Theme.MPL_GRID, alpha=0.6, linewidth=0.5)

        # Plot main axis lines
        lines1 = []
        for line in main_ax.get_lines():
            ln, = ax.plot(line.get_xdata(), line.get_ydata(),
                          color=line.get_color(), lw=line.get_linewidth(),
                          ls=line.get_linestyle(), label=line.get_label())
            lines1.append(ln)

        ax.set_xlabel(main_ax.get_xlabel())
        ax.set_ylabel(main_ax.get_ylabel())
        ax.yaxis.label.set_color(main_ax.yaxis.label.get_color())
        ax.set_title(title, fontsize=6)

        # Plot twin axis lines
        lines2 = []
        if twin_ax:
            ax2 = ax.twinx()
            ax2.tick_params(colors=Theme.MPL_TICK)
            ax2.spines['right'].set_color(Theme.BORDER)
            for line in twin_ax.get_lines():
                ln, = ax2.plot(line.get_xdata(), line.get_ydata(),
                               color=line.get_color(), lw=line.get_linewidth(),
                               ls=line.get_linestyle(), label=line.get_label())
                lines2.append(ln)
            ax2.set_ylabel(twin_ax.get_ylabel())
            ax2.yaxis.label.set_color(twin_ax.yaxis.label.get_color())

        # Combined legend
        all_lines = lines1 + lines2
        all_labels = [l.get_label() for l in all_lines]
        if all_labels:
            ax.legend(all_lines, all_labels,
                      facecolor=Theme.BG_LIGHT, edgecolor=Theme.BORDER,
                      labelcolor=Theme.TEXT_PRIMARY, fontsize=9,
                      loc='upper right')

        try:
            fig.tight_layout()
        except Exception:
            pass
        fig.savefig(filepath, dpi=180, facecolor=Theme.MPL_BG,
                    bbox_inches='tight')


def run_app():
    """Launch the Simulink GUI application."""
    import ctypes
    try:
        # Tell Windows this is a distinct app so it shows the custom taskbar icon
        ctypes.windll.shell32.SetCurrentProcessExplicitAppUserModelID("herian.mrac.simulink.1.0")
    except Exception:
        pass

    app = QApplication(sys.argv)
    
    # Set global application icon
    icon_path = resource_path("mrac_icon.ico")
    if os.path.exists(icon_path):
        app.setWindowIcon(QIcon(icon_path))
        
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
    window.showMaximized()
    sys.exit(app.exec())
