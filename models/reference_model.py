"""
reference_model.py — 2nd-Order Reference Model for MRAC

The reference model defines the desired ideal response.
Specifications from the PRD:
    - Target Overshoot: 15%
    - Peak Time: 1.8 seconds

Reference model transfer function:
    Gm(s) = omega_n^2 / (s^2 + 2*zeta*omega_n*s + omega_n^2)

Reference: Soares et al. (2021)
"""

import numpy as np
from models.config import ReferenceModelParams


class ReferenceModel:
    """
    Standard 2nd-order reference model for MRAC.

    State-space representation:
        xdot_m = Am*xm + Bm*r

    Where:
        Am = [[0, 1], [-omega_n^2, -2*zeta*omega_n]]
        Bm = [[0], [omega_n^2]]
    """

    def __init__(self, params: ReferenceModelParams = None):
        if params is None:
            params = ReferenceModelParams()
        self.params = params
        self.zeta = params.zeta
        self.omega_n = params.omega_n

        # State-space matrices
        self.Am = np.array([
            [0.0, 1.0],
            [-self.omega_n**2, -2.0 * self.zeta * self.omega_n]
        ])
        self.Bm = np.array([
            [0.0],
            [self.omega_n**2]
        ])

    def state_derivative(self, xm: np.ndarray, r: float) -> np.ndarray:
        """
        Calculates the reference model state derivative.

        xdot_m = Am*xm + Bm*r

        Args:
            xm: Reference model state [qm, dqm] (2,)
            r:  Reference input (setpoint)

        Returns:
            dxm: State derivative [dqm, ddqm] (2,)
        """
        r_vec = np.array([r])
        return self.Am @ xm + (self.Bm @ r_vec).flatten()

    def step_response_analytical(self, t: np.ndarray, r: float = 1.0) -> np.ndarray:
        """
        Analytical step response for a 2nd-order reference model.

        Args:
            t: Time array (N,)
            r: Step input amplitude

        Returns:
            y: Reference model output (N,)
        """
        zeta = self.zeta
        wn = self.omega_n
        wd = wn * np.sqrt(1 - zeta**2)  # Damped natural frequency

        y = r * (1 - np.exp(-zeta * wn * t) *
                 (np.cos(wd * t) + (zeta / np.sqrt(1 - zeta**2)) * np.sin(wd * t)))
        return y

    def get_specs(self) -> dict:
        """
        Returns the reference model specifications.

        Returns:
            dict: Reference model parameters and metrics
        """
        zeta = self.zeta
        wn = self.omega_n
        wd = wn * np.sqrt(1 - zeta**2)

        settling_time_2pct = 4.0 / (zeta * wn)
        rise_time = (np.pi - np.arctan2(np.sqrt(1 - zeta**2), zeta)) / wd

        return {
            'zeta': zeta,
            'omega_n': wn,
            'omega_d': wd,
            'overshoot_pct': self.params.overshoot * 100,
            'peak_time': self.params.peak_time,
            'settling_time_2pct': settling_time_2pct,
            'rise_time': rise_time,
        }
