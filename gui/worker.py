"""
worker.py — Background simulation thread.

Runs the ODE solver in a QThread to keep the GUI responsive.
Supports both single-run and batch (gain variation) simulation.
"""

from PySide6.QtCore import QThread, Signal
from models.config import SimConfig
from simulation.simulator import run_simulation
import copy


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


class GainVariationWorker(QThread):
    """Background thread for running batch gamma-variation simulations."""
    finished = Signal(object)  # emits list of (gamma_value, SimResult)
    error_occurred = Signal(str)

    # Gamma values to sweep (representative range)
    GAMMA_VALUES = [200, 500, 750, 1000, 1500]

    def __init__(self, config: SimConfig):
        super().__init__()
        self.config = config

    def run(self):
        try:
            results = []
            for gamma_val in self.GAMMA_VALUES:
                cfg = copy.deepcopy(self.config)
                cfg.controller.gamma1 = gamma_val
                cfg.controller.gamma2 = gamma_val
                cfg.label = f"γ={gamma_val}"
                result = run_simulation(cfg, verbose=False)
                results.append((gamma_val, result))
            self.finished.emit(results)
        except Exception as e:
            self.error_occurred.emit(str(e))
