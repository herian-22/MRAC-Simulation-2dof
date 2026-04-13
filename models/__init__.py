"""
models — Physical & mathematical models for the 2-DoF satellite dish.

Contains:
    - config: Simulation parameter dataclasses
    - dynamics: Euler-Lagrange dynamics (M, C, G)
    - kinematics: DH forward kinematics
    - reference_model: 2nd-order reference model for MRAC
"""

from models.config import (
    PhysicalParams, ControllerParams, ReferenceModelParams,
    SimulationParams, SimConfig, default_config,
    sweep_gamma, sweep_trajectory, full_sweep
)
from models.dynamics import Dynamics2DoF
from models.kinematics import ForwardKinematics
from models.reference_model import ReferenceModel
