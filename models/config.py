"""
config.py — Konfigurasi Parameter Simulasi MRAC Satellite Dish 2-DoF

Berisi dataclass SimConfig dan factory functions untuk iterasi parameter.
Referensi: Soares et al. (2021) - Adaptive Controller for Automatic Maneuver
"""

from dataclasses import dataclass, field
from typing import List, Tuple
import numpy as np
import itertools


@dataclass
class PhysicalParams:
    """Parameter fisik antena parabola 2-DoF."""
    # --- Link 1 (Azimuth) ---
    m1: float = 8.0          # Massa link 1 (kg)
    l1: float = 0.40         # Panjang link 1 (m)
    lc1: float = 0.20        # Jarak ke pusat massa link 1 (m)
    I1: float = 0.10         # Momen inersia link 1 (kg·m²)

    # --- Link 2 (Elevation) + Dish ---
    m2: float = 12.0         # Massa link 2 + dish (kg)  — dish diam 1.6m
    l2: float = 0.80         # Panjang efektif link 2 (m)  — radius dish
    lc2: float = 0.40        # Jarak ke pusat massa link 2 (m)
    I2: float = 0.80         # Momen inersia link 2 (kg·m²) — dish besar

    g: float = 9.81          # Percepatan gravitasi (m/s²)


@dataclass
class ControllerParams:
    """Parameter kontroler Computed Torque + MRAC."""
    # --- Computed Torque PD Gains ---
    Kp1: float = 100.0       # Proportional gain joint 1
    Kp2: float = 100.0       # Proportional gain joint 2
    Kv1: float = 20.0        # Derivative gain joint 1
    Kv2: float = 20.0        # Derivative gain joint 2

    # --- MRAC Adaptive Gains (MIT Rule) ---
    gamma1: float = 770.0    # Gain adaptif joint 1 (range: 710-830)
    gamma2: float = 935.0    # Gain adaptif joint 2 (range: 880-990)

    # --- MRAC adaptive parameter bounds ---
    theta_max: float = 50.0  # Batas atas parameter adaptif
    theta_min: float = -50.0 # Batas bawah parameter adaptif


@dataclass
class ReferenceModelParams:
    """Parameter model referensi orde-2."""
    overshoot: float = 0.15  # Target overshoot 15%
    peak_time: float = 1.8   # Target peak time 1.8 detik

    @property
    def zeta(self) -> float:
        """Damping ratio dari target overshoot."""
        ln_os = np.log(self.overshoot)
        return -ln_os / np.sqrt(np.pi**2 + ln_os**2)

    @property
    def omega_n(self) -> float:
        """Natural frequency dari peak time dan damping ratio."""
        z = self.zeta
        return np.pi / (self.peak_time * np.sqrt(1 - z**2))


@dataclass
class SimulationParams:
    """Parameter simulasi."""
    t_start: float = 0.0     # Waktu mulai (s)
    t_end: float = 10.0      # Waktu akhir (s)
    dt: float = 0.001        # Time step (s)
    method: str = 'RK45'     # ODE solver method

    # --- Trajectory ---
    trajectory_type: str = 'step'  # 'step', 'sinusoidal', 'multipoint'

    # Target posisi untuk step response (rad)
    q1_target: float = np.radians(45)   # Azimuth target
    q2_target: float = np.radians(30)   # Elevation target

    # Parameter sinusoidal trajectory
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
    """Konfigurasi lengkap untuk satu simulasi."""
    physical: PhysicalParams = field(default_factory=PhysicalParams)
    controller: ControllerParams = field(default_factory=ControllerParams)
    reference: ReferenceModelParams = field(default_factory=ReferenceModelParams)
    simulation: SimulationParams = field(default_factory=SimulationParams)
    label: str = "default"

    def summary_str(self) -> str:
        """String ringkasan untuk penamaan file."""
        return (f"g1={self.controller.gamma1:.0f}_"
                f"g2={self.controller.gamma2:.0f}_"
                f"traj={self.simulation.trajectory_type}")


# ═══════════════════════════════════════════════════════════════
#  Factory Functions untuk Iterasi
# ═══════════════════════════════════════════════════════════════

def default_config() -> SimConfig:
    """Konfigurasi default sesuai PRD."""
    return SimConfig(label="default")


def sweep_gamma(
    gamma1_values: List[float] = None,
    gamma2_values: List[float] = None,
    n_steps: int = 5
) -> List[SimConfig]:
    """
    Generate list konfigurasi dengan sweep γ₁ dan γ₂.
    """
    if gamma1_values is None:
        gamma1_values = np.linspace(710, 830, n_steps).tolist()
    if gamma2_values is None:
        gamma2_values = np.linspace(880, 990, n_steps).tolist()

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
    Generate konfigurasi untuk setiap jenis trajektori.
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
        gamma1_values = np.linspace(710, 830, n_steps).tolist()
    if gamma2_values is None:
        gamma2_values = np.linspace(880, 990, n_steps).tolist()
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
