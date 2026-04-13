"""
config.py — Simulation Parameter Configuration for 2-DoF Satellite Dish MRAC

Contains SimConfig dataclass and factory functions for parameter iteration.
Reference: Soares et al. (2021) - Adaptive Controller for Automatic Maneuver
"""

from dataclasses import dataclass, field
from typing import List, Tuple
import numpy as np
import itertools


@dataclass
class PhysicalParams:
    """Physical parameters of the 2-DoF satellite antenna."""
    # --- Link 1 (Azimuth) ---
    m1: float = 29.16        # Mass of link 1 (kg) - From Table 2 specs in paper
    a1: float = 0.19         # Denavit-Hartenberg parameter a1 (m)
    d1: float = 1.60         # Denavit-Hartenberg parameter d1 (m)
    
    # --- Link 2 (Elevation) + Dish ---
    m2: float = 97.39        # Mass of link 2 (kg) - From Table 2 specs in paper
    a2: float = 0.97         # Denavit-Hartenberg parameter a2 (m)

    g: float = 9.81          # Gravitational acceleration (m/s²)


@dataclass
class ControllerParams:
    """Computed Torque + MRAC controller parameters."""
    # --- Computed Torque PD Gains ---
    Kp1: float = 100.0       # Proportional gain joint 1
    Kp2: float = 100.0       # Proportional gain joint 2
    Kv1: float = 20.0        # Derivative gain joint 1
    Kv2: float = 20.0        # Derivative gain joint 2

    # --- MRAC Adaptive Gains (MIT Rule) ---
    gamma1: float = 750.0    # Adaptive gain joint 1
    gamma2: float = 900.0    # Adaptive gain joint 2

    # --- MRAC adaptive parameter bounds ---
    theta_max: float = 50.0  # Upper bound of adaptive parameter
    theta_min: float = -50.0 # Lower bound of adaptive parameter


@dataclass
class ReferenceModelParams:
    """Second-order reference model parameters."""
    overshoot: float = 0.15  # Target overshoot 15%
    peak_time: float = 1.8   # Target peak time 1.8 seconds

    @property
    def zeta(self) -> float:
        """Damping ratio derived from target overshoot."""
        ln_os = np.log(self.overshoot)
        return -ln_os / np.sqrt(np.pi**2 + ln_os**2)

    @property
    def omega_n(self) -> float:
        """Natural frequency derived from peak time and damping ratio."""
        z = self.zeta
        return np.pi / (self.peak_time * np.sqrt(1 - z**2))


@dataclass
class SimulationParams:
    """Simulation parameters."""
    t_start: float = 0.0     # Start time (s)
    t_end: float = 10.0      # End time (s)
    dt: float = 0.001        # Time step (s)
    method: str = 'RK45'     # ODE solver method

    # --- Trajectory ---
    trajectory_type: str = 'step'  # 'step', 'sinusoidal', 'multipoint'

    # Target position for step response (rad)
    q1_target: float = np.radians(60)   # Target Azimuth (u1 = 60 deg)
    q2_target: float = np.radians(30)   # Target Elevation (u2 = 30 deg)

    # Sinusoidal trajectory parameters
    sin_amplitude: float = np.radians(30)
    sin_frequency: float = 0.5  # Hz

    # Multi-point trajectory waypoints (rad)
    waypoints: List[Tuple[float, float, float]] = field(default_factory=lambda: [
        (0.0, np.radians(0), np.radians(0)),
        (2.5, np.radians(45), np.radians(30)),
        (5.0, np.radians(90), np.radians(60)),
        (7.5, np.radians(45), np.radians(15)),
        (10.0, np.radians(0), np.radians(0)),
    ])


@dataclass
class SimConfig:
    """Complete configuration for a single simulation run."""
    physical: PhysicalParams = field(default_factory=PhysicalParams)
    controller: ControllerParams = field(default_factory=ControllerParams)
    reference: ReferenceModelParams = field(default_factory=ReferenceModelParams)
    simulation: SimulationParams = field(default_factory=SimulationParams)
    label: str = "default"

    def summary_str(self) -> str:
        """Summary string for file naming."""
        return (f"g1={self.controller.gamma1:.0f}_"
                f"g2={self.controller.gamma2:.0f}_"
                f"traj={self.simulation.trajectory_type}")


# ===============================================================
#  Factory Functions for Iteration
# ===============================================================

def default_config() -> SimConfig:
    """Default configuration according to PRD."""
    return SimConfig(label="default")


def sweep_gamma(
    gamma1_values: List[float] = None,
    gamma2_values: List[float] = None,
    n_steps: int = 5
) -> List[SimConfig]:
    """
    Generate a list of configurations with sweep of γ₁ and γ₂.
    """
    if gamma1_values is None:
        gamma1_values = [200.0, 500.0, 750.0, 1000.0, 1500.0]
    if gamma2_values is None:
        gamma2_values = [200.0, 500.0, 750.0, 1000.0, 1500.0]

    configs = []
    for g1, g2 in itertools.product(gamma1_values, gamma2_values):
        cfg = SimConfig(label=f"gamma_g1={g1:.0f}_g2={g2:.0f}")
        cfg.controller.gamma1 = g1
        cfg.controller.gamma2 = g2
        configs.append(cfg)

    return configs


def sweep_trajectory(
    trajectory_types: List[str] = None
) -> List[SimConfig]:
    """
    Generate configurations for each trajectory type.
    """
    if trajectory_types is None:
        trajectory_types = ['step', 'sinusoidal', 'multipoint']

    configs = []
    for traj in trajectory_types:
        cfg = SimConfig(label=f"traj_{traj}")
        cfg.simulation.trajectory_type = traj
        configs.append(cfg)

    return configs


def full_sweep(
    gamma1_values: List[float] = None,
    gamma2_values: List[float] = None,
    trajectory_types: List[str] = None,
    n_steps: int = 3
) -> List[SimConfig]:
    """
    Full parameter sweep: γ₁ × γ₂ × trajectory type.
    """
    if gamma1_values is None:
        gamma1_values = [200.0, 500.0, 750.0, 1000.0, 1500.0]
    if gamma2_values is None:
        gamma2_values = [200.0, 500.0, 750.0, 1000.0, 1500.0]
    if trajectory_types is None:
        trajectory_types = ['step', 'sinusoidal', 'multipoint']

    configs = []
    for g1, g2, traj in itertools.product(gamma1_values, gamma2_values, trajectory_types):
        cfg = SimConfig(label=f"full_g1={g1:.0f}_g2={g2:.0f}_{traj}")
        cfg.controller.gamma1 = g1
        cfg.controller.gamma2 = g2
        cfg.simulation.trajectory_type = traj
        configs.append(cfg)

    return configs
