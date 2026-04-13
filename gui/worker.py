"""
worker.py — Background simulation thread.

Runs the ODE solver in a QThread to keep the GUI responsive.
"""

from PySide6.QtCore import QThread, Signal
from models.config import SimConfig
from simulation.simulator import run_simulation


class SimulationWorker(QThread):
    """Background thread for running the ODE simulation."""
    finished = Signal(object)
    error_occurred = Signal(str)

    def __init__(self, config: SimConfig):
        super().__init__()
        self.config = config

    def run(self):
        try:
            result = run_simulation(self.config, verbose=False)
            self.finished.emit(result)
        except Exception as e:
            self.error_occurred.emit(str(e))
