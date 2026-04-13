"""
controller.py — Computed Torque + MRAC (MIT Rule) Controller

Layered control strategy:
1. Computed Torque  : Feedback linearization (removes nonlinearities)
2. MRAC (MIT Rule)  : Online adaptation to handle model uncertainty

Control law:
    τ = M(q)·(q̈_d + Kv·ė + Kp·e + u_mrac) + C(q, q̇)·q̇ + G(q)

MIT Rule Adaptation:
    dθ/dt = -γ · e · (∂ym/∂θ) ≈ -γ · e · sensitivity

Reference: Soares et al. (2021)
"""

import numpy as np
from models.config import ControllerParams
from models.dynamics import Dynamics2DoF
from models.reference_model import ReferenceModel


class ComputedTorqueController:
    """
    Computed Torque Controller (feedback linearization).

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
        Calculates the controller torque.
        """
        # Tracking error
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
    Model Reference Adaptive Controller using MIT Rule for 2-DoF.
    Specifically adjusts 1 adaptive parameter alpha_i per joint to compensate
    for dissipative torque (alpha_i \approx b_i).
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
        self.ct_controller = ComputedTorqueController(dynamics, params) # Keep just in case
        self.ref_models = [ref_model_j1, ref_model_j2]
        self.gamma = np.array([params.gamma1, params.gamma2])
        
        # Adaptive parameters
        self.alpha = np.zeros(2)

    def adaptation_law(self, e: np.ndarray, dphi: np.ndarray) -> np.ndarray:
        """
        MIT Rule: d(alpha_i)/dt = -gamma_i * e_i * d(phi_i)/dt
        """
        dalpha = np.zeros(2)
        dalpha[0] = -self.gamma[0] * e[0] * dphi[0]
        dalpha[1] = -self.gamma[1] * e[1] * dphi[1]
        return dalpha

    def compute_full_torque(
        self,
        q: np.ndarray,
        dq: np.ndarray,
        u_ref: np.ndarray,
        qm: np.ndarray,
        dqm: np.ndarray
    ) -> np.ndarray:
        """
        Calculates the total torque according to the model in the paper: 
        Computed Torque + Adaptive Control (friction compensation)
        Tracking target: Reference Model (qm, dqm)
        """
        M, C, G = self.dynamics.compute_dynamics(q, dq)
        
        ref1 = self.ref_models[0]
        ref2 = self.ref_models[1]
        
        # Acceleration feedforward from reference model dynamics
        # qm_ddot = omega^2 * r - 2*zeta*omega*qm_dot - omega^2 * qm
        ddqm = np.zeros(2)
        ddqm[0] = (ref1.omega_n**2 * u_ref[0]) - (2 * ref1.zeta * ref1.omega_n * dqm[0]) - (ref1.omega_n**2 * qm[0])
        ddqm[1] = (ref2.omega_n**2 * u_ref[1]) - (2 * ref2.zeta * ref2.omega_n * dqm[1]) - (ref2.omega_n**2 * qm[1])
        
        # PD Tracking Error (relative to reference model trajectory)
        e_q = qm - q
        e_dq = dqm - dq
        
        v = ddqm + self.ct_controller.Kv @ e_dq + self.ct_controller.Kp @ e_q
        
        tau_m = M @ v + C @ dq + G
        tau_a = np.array([self.alpha[0] * dq[0], self.alpha[1] * dq[1]])
        
        return tau_m + tau_a
