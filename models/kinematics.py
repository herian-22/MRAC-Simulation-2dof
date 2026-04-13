"""
kinematics.py — Kinematika Denavit-Hartenberg untuk 2-DoF Serial Manipulator

Menggunakan notasi DH standar untuk memetakan sudut joint
ke posisi end-effector (ujung antena parabola) dalam ruang 3D.

Referensi: Soares et al. (2021), Craig - Introduction to Robotics
"""

import numpy as np
from models.config import PhysicalParams


class ForwardKinematics:
    """
    Kinematika langsung menggunakan konvensi Denavit-Hartenberg
    untuk manipulator serial 2-DoF (Azimuth + Elevation).
    """

    def __init__(self, params: PhysicalParams = None):
        if params is None:
            params = PhysicalParams()
        self.p = params

    @staticmethod
    def dh_transform(a: float, alpha: float, d: float, theta: float) -> np.ndarray:
        """
        Membuat matriks transformasi homogen 4×4 dari parameter DH.

        Args:
            a:     Panjang link (m)
            alpha: Twist link (rad)
            d:     Offset link (m)
            theta: Sudut joint (rad)

        Returns:
            T: Matriks transformasi 4×4
        """
        ct = np.cos(theta)
        st = np.sin(theta)
        ca = np.cos(alpha)
        sa = np.sin(alpha)

        return np.array([
            [ct, -st * ca,  st * sa, a * ct],
            [st,  ct * ca, -ct * sa, a * st],
            [0,   sa,       ca,      d     ],
            [0,   0,        0,       1     ]
        ])

    def dh_table(self, q: np.ndarray) -> list:
        """
        Tabel parameter DH untuk satellite dish 2-DoF.

        Joint 1 (Azimuth):  Rotasi di sumbu Z
        Joint 2 (Elevation): Rotasi di sumbu Y → alpha = -π/2

        Args:
            q: Vektor sudut joint [q1, q2] (rad)

        Returns:
            list: [[a, alpha, d, theta], ...] untuk setiap link
        """
        p = self.p
        return [
            # [a,     alpha,       d,   theta]
            [0,       -np.pi/2,  p.l1,  q[0]],   # Joint 1: Azimuth
            [p.l2,    0,         0,     q[1]],    # Joint 2: Elevation
        ]

    def forward(self, q: np.ndarray) -> np.ndarray:
        """
        Menghitung posisi end-effector dalam ruang 3D.

        Args:
            q: Vektor sudut joint [q1, q2] (rad)

        Returns:
            pos: Posisi end-effector [x, y, z] (m)
        """
        T = self.transform_matrix(q)
        return T[:3, 3]

    def transform_matrix(self, q: np.ndarray) -> np.ndarray:
        """
        Menghitung matriks transformasi total T₀₂ (base → end-effector).

        Args:
            q: Vektor sudut joint [q1, q2] (rad)

        Returns:
            T: Matriks transformasi homogen 4×4
        """
        table = self.dh_table(q)
        T = np.eye(4)
        for row in table:
            T = T @ self.dh_transform(*row)
        return T

    def jacobian(self, q: np.ndarray, delta: float = 1e-6) -> np.ndarray:
        """
        Menghitung Jacobian geometrik secara numerik (finite difference).

        Args:
            q: Vektor sudut joint [q1, q2] (rad)
            delta: Perturbasi untuk finite difference

        Returns:
            J: Matriks Jacobian (3×2) — hanya posisi
        """
        n = len(q)
        pos0 = self.forward(q)
        J = np.zeros((3, n))

        for i in range(n):
            q_pert = q.copy()
            q_pert[i] += delta
            pos_pert = self.forward(q_pert)
            J[:, i] = (pos_pert - pos0) / delta

        return J

    def workspace_points(self, n_samples: int = 50) -> np.ndarray:
        """
        Menghasilkan titik-titik workspace (reachable space) antena.

        Args:
            n_samples: Jumlah sampel per joint

        Returns:
            points: Array (N, 3) posisi end-effector
        """
        q1_range = np.linspace(-np.pi, np.pi, n_samples)
        q2_range = np.linspace(-np.pi/2, np.pi/2, n_samples)
        points = []

        for q1 in q1_range:
            for q2 in q2_range:
                pos = self.forward(np.array([q1, q2]))
                points.append(pos)

        return np.array(points)
