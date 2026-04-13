"""
kinematics.py — Denavit-Hartenberg Kinematics for 2-DoF Serial Manipulator

Uses standard DH notation to map joint angles to end-effector position
(the tip of the satellite dish) in 3D space.

Reference: Soares et al. (2021), Craig - Introduction to Robotics
"""

import numpy as np
from models.config import PhysicalParams


class ForwardKinematics:
    """
    Forward kinematics using Denavit-Hartenberg convention
    for a 2-DoF serial manipulator (Azimuth + Elevation).
    """

    def __init__(self, params: PhysicalParams = None):
        if params is None:
            params = PhysicalParams()
        self.p = params

    @staticmethod
    def dh_transform(a: float, alpha: float, d: float, theta: float) -> np.ndarray:
        """
        Creates a 4×4 homogeneous transformation matrix from DH parameters.

        Args:
            a:     Link length (m)
            alpha: Link twist (rad)
            d:     Link offset (m)
            theta: Joint angle (rad)

        Returns:
            T: 4×4 transformation matrix
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
        DH parameter table for the 2-DoF satellite dish.

        Joint 1 (Azimuth):   Rotation around Z-axis
        Joint 2 (Elevation): Rotation around Y-axis → alpha = -π/2 (or pi/2 depending on convention)

        Args:
            q: Joint angle vector [q1, q2] (rad)

        Returns:
            list: [[a, alpha, d, theta], ...] for each link
        """
        p = self.p
        return [
            # [a,     alpha,       d,   theta]
            [p.a1,    np.pi/2,   p.d1,  q[0]],   # Joint 1: Azimuth (alpha = pi/2 so +theta2 rotates upward)
            [p.a2,    0,         0,     q[1]],   # Joint 2: Elevation
        ]

    def forward(self, q: np.ndarray) -> np.ndarray:
        """
        Calculates the end-effector position in 3D space.

        Args:
            q: Joint angle vector [q1, q2] (rad)

        Returns:
            pos: End-effector position [x, y, z] (m)
        """
        T = self.transform_matrix(q)
        return T[:3, 3]

    def transform_matrix(self, q: np.ndarray) -> np.ndarray:
        """
        Calculates the total transformation matrix T02 (base → end-effector).

        Args:
            q: Joint angle vector [q1, q2] (rad)

        Returns:
            T: 4×4 homogeneous transformation matrix
        """
        table = self.dh_table(q)
        T = np.eye(4)
        for row in table:
            T = T @ self.dh_transform(*row)
        return T

    def jacobian(self, q: np.ndarray, delta: float = 1e-6) -> np.ndarray:
        """
        Calculates the geometric Jacobian numerically (finite difference).

        Args:
            q: Joint angle vector [q1, q2] (rad)
            delta: Perturbation for finite difference

        Returns:
            J: Jacobian matrix (3×2) — position only
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
        Generates workspace points (reachable space) for the antenna.

        Args:
            n_samples: Number of samples per joint

        Returns:
            points: Array (N, 3) of end-effector positions
        """
        q1_range = np.linspace(-np.pi, np.pi, n_samples)
        q2_range = np.linspace(-np.pi/2, np.pi/2, n_samples)
        points = []

        for q1 in q1_range:
            for q2 in q2_range:
                pos = self.forward(np.array([q1, q2]))
                points.append(pos)

        return np.array(points)
