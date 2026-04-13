"""
reference_model.py — Model Referensi Orde-2 untuk MRAC

Model referensi mendefinisikan respon ideal yang diinginkan.
Spesifikasi dari PRD:
    - Overshoot target: 15%
    - Peak time: 1.8 detik

Transfer function model referensi:
    Gm(s) = ωn² / (s² + 2ζωn·s + ωn²)

Referensi: Soares et al. (2021)
"""

import numpy as np
from models.config import ReferenceModelParams


class ReferenceModel:
    """
    Model referensi orde-2 standar untuk MRAC.

    State-space representation:
        ẋm = Am·xm + Bm·r

    Dimana:
        Am = [[0, 1], [-ωn², -2ζωn]]
        Bm = [[0], [ωn²]]
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
        Menghitung turunan state model referensi.

        ẋm = Am·xm + Bm·r

        Args:
            xm: State model referensi [qm, dqm] (2,)
            r:  Input referensi (setpoint)

        Returns:
            dxm: Turunan state [dqm, ddqm] (2,)
        """
        r_vec = np.array([r])
        return self.Am @ xm + (self.Bm @ r_vec).flatten()

    def step_response_analytical(self, t: np.ndarray, r: float = 1.0) -> np.ndarray:
        """
        Respon step analitik model referensi orde-2.

        Args:
            t: Array waktu (N,)
            r: Amplitudo step input

        Returns:
            y: Output model referensi (N,)
        """
        zeta = self.zeta
        wn = self.omega_n
        wd = wn * np.sqrt(1 - zeta**2)  # Damped natural frequency

        y = r * (1 - np.exp(-zeta * wn * t) *
                 (np.cos(wd * t) + (zeta / np.sqrt(1 - zeta**2)) * np.sin(wd * t)))
        return y

    def get_specs(self) -> dict:
        """
        Mengembalikan spesifikasi model referensi.

        Returns:
            dict: Parameter dan metrik model referensi
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
