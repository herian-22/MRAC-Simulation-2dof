"""
scope.py — Matplotlib scope canvases for simulation results.

Dark-themed matplotlib FigureCanvas widgets for embedding in PySide6.
"""

from PySide6.QtWidgets import QSizePolicy
from matplotlib.backends.backend_qtagg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure
import matplotlib
matplotlib.use('QtAgg')

from gui.theme import Theme


class ScopePlotCanvas(FigureCanvas):
    """Dark-themed matplotlib canvas for scope views."""

    def __init__(self, parent=None, width=8, height=5, dpi=100):
        self.fig = Figure(figsize=(width, height), dpi=dpi)
        self.fig.patch.set_facecolor(Theme.MPL_BG)
        super().__init__(self.fig)
        self.setParent(parent)
        self.setSizePolicy(QSizePolicy.Policy.Expanding,
                           QSizePolicy.Policy.Expanding)

    def clear(self):
        self.fig.clear()

    def apply_style(self, ax):
        """Apply dark theme to a matplotlib axis."""
        ax.set_facecolor(Theme.MPL_AXES)
        ax.tick_params(colors=Theme.MPL_TICK)
        ax.xaxis.label.set_color(Theme.MPL_TEXT)
        ax.yaxis.label.set_color(Theme.MPL_TEXT)
        ax.title.set_color(Theme.MPL_TEXT)
        ax.title.set_fontweight('bold')
        for spine in ax.spines.values():
            spine.set_color(Theme.BORDER)
        ax.grid(True, color=Theme.MPL_GRID, alpha=0.6, linewidth=0.5)
