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
    Model dinamika Euler-Lagrange untuk manipulator serial 2-DoF.

    Joint 1: Azimuth (rotasi horizontal)
    Joint 2: Elevation/Slope (rotasi vertikal)

    Mendukung model uncertainty untuk simulasi realistis:
    - friction_coeff: koefisien gesekan viskos (tidak diketahui oleh kontroler)
    - mass_uncertainty: faktor ketidakpastian massa (1.0 = nominal)
    """

    def __init__(self, params: PhysicalParams = None,
                 friction_coeff: np.ndarray = None,
                 mass_uncertainty: float = 1.0):
        if params is None:
            params = PhysicalParams()
        self.p = params
        # Friction koefisien viskos (Nm·s/rad) — tidak diketahui kontroler
        self.friction = friction_coeff if friction_coeff is not None else np.zeros(2)
        # Faktor ketidakpastian massa (1.0 = sesuai model nominal)
        self.mass_unc = mass_uncertainty

    def inertia_matrix(self, q: np.ndarray) -> np.ndarray:
        """
        Menghitung matriks inersia M(q) (2×2).

        Args:
            q: Vektor sudut joint [q1, q2] (rad)

        Returns:
            M: Matriks inersia simetrik positif-definit (2×2)
        """
        p = self.p
        c2 = np.cos(q[1])

        # Elemen matriks inersia
        m11 = (p.I1 + p.I2
               + p.m1 * p.lc1**2
               + p.m2 * (p.l1**2 + p.lc2**2 + 2 * p.l1 * p.lc2 * c2))
        m12 = p.I2 + p.m2 * (p.lc2**2 + p.l1 * p.lc2 * c2)
        m21 = m12
        m22 = p.I2 + p.m2 * p.lc2**2

        return np.array([[m11, m12],
                         [m21, m22]])

    def coriolis_matrix(self, q: np.ndarray, dq: np.ndarray) -> np.ndarray:
        """
        Menghitung matriks Coriolis dan sentrifugal C(q, q̇) (2×2).

        Menggunakan simbol Christoffel.

        Args:
            q: Vektor sudut joint [q1, q2] (rad)
            dq: Vektor kecepatan joint [dq1, dq2] (rad/s)

        Returns:
            C: Matriks Coriolis (2×2)
        """
        p = self.p
        s2 = np.sin(q[1])
        h = p.m2 * p.l1 * p.lc2 * s2

        c11 = -h * dq[1]
        c12 = -h * (dq[0] + dq[1])
        c21 = h * dq[0]
        c22 = 0.0

        return np.array([[c11, c12],
                         [c21, c22]])

    def gravity_vector(self, q: np.ndarray) -> np.ndarray:
        """
        Menghitung vektor gravitasi G(q) (2×1).

        Args:
            q: Vektor sudut joint [q1, q2] (rad)

        Returns:
            G: Vektor gravitasi (2,)
        """
        p = self.p
        c1 = np.cos(q[0])
        c12 = np.cos(q[0] + q[1])

        g1 = (p.m1 * p.lc1 + p.m2 * p.l1) * p.g * c1 + p.m2 * p.lc2 * p.g * c12
        g2 = p.m2 * p.lc2 * p.g * c12

        return np.array([g1, g2])

    def compute_dynamics(self, q: np.ndarray, dq: np.ndarray):
        """
        Menghitung semua komponen dinamika sekaligus.

        Args:
            q: Vektor sudut joint [q1, q2] (rad)
            dq: Vektor kecepatan joint [dq1, dq2] (rad/s)

        Returns:
            tuple: (M, C, G) — matriks inersia, Coriolis, gravitasi
        """
        M = self.inertia_matrix(q)
        C = self.coriolis_matrix(q, dq)
        G = self.gravity_vector(q)
        return M, C, G

    def forward_dynamics(self, q: np.ndarray, dq: np.ndarray, tau: np.ndarray) -> np.ndarray:
        """
        Menghitung percepatan joint (forward dynamics).

        q̈ = M_true⁻¹(τ - C_true·q̇ - G_true - friction·q̇)

        Menggunakan parameter TRUE plant (termasuk uncertainty).

        Args:
            q: Vektor sudut joint [q1, q2] (rad)
            dq: Vektor kecepatan joint [dq1, dq2] (rad/s)
            tau: Vektor torsi input [τ1, τ2] (Nm)

        Returns:
            ddq: Vektor percepatan joint [ddq1, ddq2] (rad/s²)
        """
        M, C, G = self.compute_dynamics(q, dq)

        # Terapkan ketidakpastian massa pada plant sebenarnya
        M_true = M * self.mass_unc
        C_true = C * self.mass_unc
        G_true = G * self.mass_unc

        # Tambahkan gesekan viskos (tidak diketahui kontroler)
        friction_torque = self.friction * dq

        ddq = np.linalg.solve(M_true, tau - C_true @ dq - G_true - friction_torque)
        return ddq

    def inverse_dynamics(self, q: np.ndarray, dq: np.ndarray, ddq: np.ndarray) -> np.ndarray:
        """
        Menghitung torsi yang diperlukan (inverse dynamics).

        τ = M·q̈ + C·q̇ + G

        Args:
            q, dq, ddq: Posisi, kecepatan, percepatan joint

        Returns:
            tau: Vektor torsi (2,)
        """
        M, C, G = self.compute_dynamics(q, dq)
        tau = M @ ddq + C @ dq + G
        return tau
