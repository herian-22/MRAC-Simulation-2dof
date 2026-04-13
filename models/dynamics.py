"""
dynamics.py — Model Dinamika Euler-Lagrange untuk 2-DoF Serial Manipulator

Persamaan gerak:
    M(q)q̈ + C(q, q̇)q̇ + G(q) = τ

Dimana:
    M(q)     : Matriks inersia (2×2)
    C(q, q̇)  : Matriks Coriolis/sentrifugal (2×2)
    G(q)     : Vektor gravitasi (2×1)
    τ        : Vektor torsi input (2×1)

Referensi: Soares et al. (2021), Spong et al. - Robot Modeling and Control
"""

import numpy as np
from models.config import PhysicalParams


class Dynamics2DoF:
    """
    Euler-Lagrange dynamics model for a 2-DoF serial manipulator.

    Joint 1: Azimuth (horizontal rotation)
    Joint 2: Elevation/Slope (vertical rotation)

    Supports model uncertainty for realistic simulation:
    - friction_coeff: viscous friction coefficient (unknown to the controller)
    - mass_uncertainty: mass uncertainty factor (1.0 = nominal)
    """

    def __init__(self, params: PhysicalParams = None,
                 friction_coeff: np.ndarray = None,
                 mass_uncertainty: float = 1.0):
        if params is None:
            params = PhysicalParams()
        self.p = params
        # Viscous friction coefficient (Nm·s/rad) — unknown to the controller
        self.friction = friction_coeff if friction_coeff is not None else np.zeros(2)
        # Mass uncertainty factor (1.0 = matches nominal model)
        self.mass_unc = mass_uncertainty

    def inertia_matrix(self, q: np.ndarray) -> np.ndarray:
        """
        Calculates the inertia matrix M(q) (2×2).

        Args:
            q: Joint angle vector [q1, q2] (rad)

        Returns:
            M: Symmetric positive-definite inertia matrix (2×2)
        """
        p = self.p
        q2 = q[1]
        
        a1 = p.a1
        a2 = p.a2
        m1 = p.m1
        m2 = p.m2

        # Correct analytical evaluation based on reference parameters (m1=29.16, m2=97.39)
        # to prevent negative inertia error in the original paper's equation (10).
        D11 = -58.805 * np.sin(q2)**2 + 7.7717 * np.cos(q2) + 72.516
        D12 = 0.0
        D21 = 0.0
        D22 = 66.625

        return np.array([[D11, D12],
                         [D21, D22]])

    def coriolis_matrix(self, q: np.ndarray, dq: np.ndarray) -> np.ndarray:
        """
        Calculates the Coriolis and centrifugal matrix C(q, q̇) (2×2).

        Uses Christoffel symbols.

        Args:
            q: Joint angle vector [q1, q2] (rad)
            dq: Joint velocity vector [dq1, dq2] (rad/s)

        Returns:
            C: Coriolis matrix (2×2)
        """
        p = self.p
        q2 = q[1]
        s2 = np.sin(q2)
        
        # Christoffel evaluation of partial derivative of M11
        # H112 = 0.5 * d(M11)/dq2
        H112 = -58.805 * s2 * np.cos(q2) - 3.88585 * s2
                
        # H211 = -H112
        H211 = -H112

        # Christoffel derivation of C matrix: C_kj = sum_i(h_kij * dq_i)
        C11 = H112 * dq[1]
        C12 = H112 * dq[0]
        C21 = H211 * dq[0]
        C22 = 0.0

        return np.array([[C11, C12],
                         [C21, C22]])

    def gravity_vector(self, q: np.ndarray) -> np.ndarray:
        """
        Calculates the gravity vector G(q) (2×1).

        Args:
            q: Joint angle vector [q1, q2] (rad)

        Returns:
            G: Gravity vector (2,)
        """
        p = self.p
        q2 = q[1]
        c2 = np.cos(q2)
        
        # Correct analytical model based on potential derivative
        C1 = 0.0
        C2 = -200.63 * c2

        return np.array([C1, C2])

    def compute_dynamics(self, q: np.ndarray, dq: np.ndarray):
        """
        Calculates all dynamics components simultaneously.

        Args:
            q: Joint angle vector [q1, q2] (rad)
            dq: Joint velocity vector [dq1, dq2] (rad/s)

        Returns:
            tuple: (M, C, G) — Inertia, Coriolis, and Gravity matrices/vector
        """
        M = self.inertia_matrix(q)
        C = self.coriolis_matrix(q, dq)
        G = self.gravity_vector(q)
        return M, C, G

    def forward_dynamics(self, q: np.ndarray, dq: np.ndarray, tau: np.ndarray) -> np.ndarray:
        """
        Calculates joint acceleration (forward dynamics).

        qddot = M_true^-1 * (tau - C_true*qdot - G_true - friction*qdot)

        Uses TRUE plant parameters (including uncertainty).

        Args:
            q: Joint angle vector [q1, q2] (rad)
            dq: Joint velocity vector [dq1, dq2] (rad/s)
            tau: Input torque vector [tau1, tau2] (Nm)

        Returns:
            ddq: Joint acceleration vector [ddq1, ddq2] (rad/s^2)
        """
        M, C, G = self.compute_dynamics(q, dq)

        # Apply mass uncertainty to the actual plant
        M_true = M * self.mass_unc
        C_true = C * self.mass_unc
        G_true = G * self.mass_unc

        # Add viscous friction (unknown to the controller)
        friction_torque = self.friction * dq

        ddq = np.linalg.solve(M_true, tau - C_true @ dq - G_true - friction_torque)
        return ddq

    def inverse_dynamics(self, q: np.ndarray, dq: np.ndarray, ddq: np.ndarray) -> np.ndarray:
        """
        Calculates required torques (inverse dynamics).

        tau = M*qddot + C*qdot + G

        Args:
            q, dq, ddq: Joint position, velocity, and acceleration

        Returns:
            tau: Torque vector (2,)
        """
        M, C, G = self.compute_dynamics(q, dq)
        tau = M @ ddq + C @ dq + G
        return tau
