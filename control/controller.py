"""
controller.py — Kontroler Computed Torque + MRAC (MIT Rule)

Strategi kontrol berlapis:
1. Computed Torque  : Linearisasi umpan balik (menghilangkan non-linearitas)
2. MRAC (MIT Rule)  : Adaptasi online untuk menangani ketidakpastian model

Hukum kontrol:
    τ = M(q)·(q̈_d + Kv·ė + Kp·e + u_mrac) + C(q, q̇)·q̇ + G(q)

Adaptasi MIT Rule:
    dθ/dt = -γ · e · (∂ym/∂θ) ≈ -γ · e · sensitivity

Referensi: Soares et al. (2021)
"""

import numpy as np
from models.config import ControllerParams
from models.dynamics import Dynamics2DoF
from models.reference_model import ReferenceModel


class ComputedTorqueController:
    """
    Kontroler Computed Torque (feedback linearization).

    τ = M(q)·(q̈_d + Kv·ė + Kp·e) + C(q, q̇)·q̇ + G(q)
    """

    def __init__(self, dynamics: Dynamics2DoF, params: ControllerParams):
        self.dynamics = dynamics
        self.Kp = np.diag([params.Kp1, params.Kp2])
        self.Kv = np.diag([params.Kv1, params.Kv2])

    def compute_torque(
        self,
        q: np.ndarray,
        dq: np.ndarray,
        q_d: np.ndarray,
        dq_d: np.ndarray,
        ddq_d: np.ndarray,
        u_adaptive: np.ndarray = None
    ) -> np.ndarray:
        """
        Menghitung torsi kontroler.

        Args:
            q, dq:          State aktual [posisi, kecepatan]
            q_d, dq_d, ddq_d: Desired trajectory
            u_adaptive:     Sinyal kontrol adaptif tambahan (dari MRAC)

        Returns:
            tau: Vektor torsi [τ1, τ2] (Nm)
        """
        # Error tracking
        e = q_d - q
        de = dq_d - dq

        # Auxiliary control input (PD + feedforward + adaptive)
        v = ddq_d + self.Kv @ de + self.Kp @ e
        if u_adaptive is not None:
            v += u_adaptive

        # Computed Torque law
        M, C, G = self.dynamics.compute_dynamics(q, dq)
        tau = M @ v + C @ dq + G

        return tau


class MRACController:
    """
    Model Reference Adaptive Controller menggunakan MIT Rule.

    Mengadaptasi parameter kontroler secara online untuk meminimalkan
    error antara output plant dan model referensi.
    """

    def __init__(
        self,
        dynamics: Dynamics2DoF,
        params: ControllerParams,
        ref_model_j1: ReferenceModel,
        ref_model_j2: ReferenceModel
    ):
        self.dynamics = dynamics
        self.params = params
        self.ct_controller = ComputedTorqueController(dynamics, params)

        # Reference models per joint
        self.ref_models = [ref_model_j1, ref_model_j2]

        # Adaptive gains (MIT Rule)
        self.gamma = np.array([params.gamma1, params.gamma2])

        # Adaptive parameters (θ) — initialized to zero
        # θ = [θ_r, θ_q, θ_dq] for each joint (feedforward, position, velocity)
        self.n_params_per_joint = 3
        self.theta = np.zeros(2 * self.n_params_per_joint)  # [θ_j1(3), θ_j2(3)]

    def adaptive_signal(
        self,
        q: np.ndarray,
        dq: np.ndarray,
        r: np.ndarray
    ) -> np.ndarray:
        """
        Menghitung sinyal kontrol adaptif u_mrac.

        u_mrac_i = θ_r_i · r_i + θ_q_i · q_i + θ_dq_i · dq_i

        Args:
            q: Sudut joint aktual [q1, q2]
            dq: Kecepatan joint aktual [dq1, dq2]
            r: Input referensi [r1, r2]

        Returns:
            u_mrac: Sinyal kontrol adaptif [u1, u2]
        """
        u = np.zeros(2)
        for i in range(2):
            idx = i * self.n_params_per_joint
            theta_r = self.theta[idx]
            theta_q = self.theta[idx + 1]
            theta_dq = self.theta[idx + 2]
            u[i] = theta_r * r[i] + theta_q * q[i] + theta_dq * dq[i]

        return u

    def adaptation_law(
        self,
        e: np.ndarray,
        q: np.ndarray,
        dq: np.ndarray,
        r: np.ndarray,
        xm: np.ndarray
    ) -> np.ndarray:
        """
        MIT Rule: dθ/dt = -γ · e · φ(t)

        Dimana φ(t) adalah regressor vector (sensitivity).

        Args:
            e: Error [e1, e2] = q - qm (aktual - referensi)
            q: Sudut joint aktual
            dq: Kecepatan joint aktual
            r: Input referensi
            xm: State model referensi [[qm1, dqm1], [qm2, dqm2]]

        Returns:
            dtheta: Turunan parameter adaptif (6,)
        """
        dtheta = np.zeros_like(self.theta)

        for i in range(2):
            idx = i * self.n_params_per_joint
            gamma_i = self.gamma[i]

            # Sensitivity (regressor vector)
            ref = self.ref_models[i]
            wn2 = ref.omega_n ** 2

            # Normalized sensitivity untuk stabilitas yang lebih baik
            norm_factor = 1.0 / (1.0 + r[i]**2 + q[i]**2 + dq[i]**2)

            # MIT Rule update
            dtheta[idx]     = -gamma_i * e[i] * r[i] * norm_factor     # θ_r
            dtheta[idx + 1] = -gamma_i * e[i] * q[i] * norm_factor     # θ_q
            dtheta[idx + 2] = -gamma_i * e[i] * dq[i] * norm_factor    # θ_dq

        return dtheta

    def compute_full_torque(
        self,
        q: np.ndarray,
        dq: np.ndarray,
        q_d: np.ndarray,
        dq_d: np.ndarray,
        ddq_d: np.ndarray,
        r: np.ndarray
    ) -> np.ndarray:
        """
        Menghitung total torsi: Computed Torque + MRAC.

        Args:
            q, dq:          State aktual
            q_d, dq_d, ddq_d: Desired (dari reference model)
            r:              Input referensi (setpoint)

        Returns:
            tau: Total torsi [τ1, τ2] (Nm)
        """
        # Sinyal adaptif
        u_mrac = self.adaptive_signal(q, dq, r)

        # Computed torque + adaptive
        tau = self.ct_controller.compute_torque(q, dq, q_d, dq_d, ddq_d, u_mrac)

        return tau

    def clip_theta(self):
        """Clamp parameter adaptif agar tidak diverge."""
        self.theta = np.clip(
            self.theta,
            self.params.theta_min,
            self.params.theta_max
        )
