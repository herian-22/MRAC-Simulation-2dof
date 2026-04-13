"""
simulator.py — Engine Simulasi MRAC untuk Satellite Dish 2-DoF

Menjalankan integrasi numerik dari sistem plant + kontroler + model referensi.
Mendukung single run dan batch/iterasi.

Referensi: Soares et al. (2021)
"""

import numpy as np
from scipy.integrate import solve_ivp
from dataclasses import dataclass, field
from typing import List, Callable

from models.config import SimConfig
from models.dynamics import Dynamics2DoF
from models.reference_model import ReferenceModel
from control.controller import MRACController


@dataclass
class SimResult:
    """Hasil satu simulasi."""
    config: SimConfig
    t: np.ndarray                # Waktu (N,)
    q: np.ndarray                # Sudut joint (N, 2)
    dq: np.ndarray               # Kecepatan joint (N, 2)
    q_ref: np.ndarray            # Output model referensi (N, 2)
    dq_ref: np.ndarray           # Kecepatan model referensi (N, 2)
    q_desired: np.ndarray        # Setpoint/trajectory desired (N, 2)
    error: np.ndarray            # Error tracking e = q - q_ref (N, 2)
    tau: np.ndarray              # Torsi kontroler (N, 2)
    theta_adapt: np.ndarray      # Parameter adaptif (N, 6)
    end_effector: np.ndarray     # Posisi end-effector (N, 3)

    # --- Metrik ---
    metrics: dict = field(default_factory=dict)


def trajectory_generator(config: SimConfig) -> Callable:
    """
    Membuat fungsi trajectory berdasarkan konfigurasi.

    Args:
        config: Konfigurasi simulasi

    Returns:
        func(t) → (q_d, dq_d, ddq_d): Fungsi trajectory
    """
    sim = config.simulation
    traj_type = sim.trajectory_type

    if traj_type == 'step':
        q_target = np.array([sim.q1_target, sim.q2_target])

        def trajectory(t):
            q_d = q_target.copy()
            dq_d = np.zeros(2)
            ddq_d = np.zeros(2)
            return q_d, dq_d, ddq_d

        return trajectory

    elif traj_type == 'sinusoidal':
        amp = sim.sin_amplitude
        freq = sim.sin_frequency

        def trajectory(t):
            omega = 2 * np.pi * freq
            q_d = amp * np.sin(omega * t) * np.array([1.0, 0.7])
            dq_d = amp * omega * np.cos(omega * t) * np.array([1.0, 0.7])
            ddq_d = -amp * omega**2 * np.sin(omega * t) * np.array([1.0, 0.7])
            return q_d, dq_d, ddq_d

        return trajectory

    elif traj_type == 'multipoint':
        waypoints = sim.waypoints
        # Build smooth spline-like trajectory dari waypoints
        times = [wp[0] for wp in waypoints]
        q1_pts = [wp[1] for wp in waypoints]
        q2_pts = [wp[2] for wp in waypoints]

        from scipy.interpolate import CubicSpline
        cs1 = CubicSpline(times, q1_pts, bc_type='clamped')
        cs2 = CubicSpline(times, q2_pts, bc_type='clamped')

        def trajectory(t):
            t_clamp = np.clip(t, times[0], times[-1])
            q_d = np.array([cs1(t_clamp), cs2(t_clamp)])
            dq_d = np.array([cs1(t_clamp, 1), cs2(t_clamp, 1)])
            ddq_d = np.array([cs1(t_clamp, 2), cs2(t_clamp, 2)])
            return q_d, dq_d, ddq_d

        return trajectory

    else:
        raise ValueError(f"Unknown trajectory type: {traj_type}")


def run_simulation(config: SimConfig, verbose: bool = True) -> SimResult:
    """
    Menjalankan satu simulasi MRAC lengkap.

    Args:
        config: Konfigurasi simulasi
        verbose: Print progress

    Returns:
        SimResult: Hasil simulasi
    """
    if verbose:
        print(f"  Running: {config.label}...")

    # --- Inisialisasi komponen ---
    # Controller menggunakan model NOMINAL (tanpa uncertainty)
    controller_dynamics = Dynamics2DoF(config.physical)

    # Plant SEBENARNYA memiliki ketidakpastian (gesekan + massa berbeda)
    # Ini yang membuat MRAC diperlukan!
    plant_dynamics = Dynamics2DoF(
        config.physical,
        friction_coeff=np.array([2.5, 1.8]),  # Gesekan viskos (Nm·s/rad)
        mass_uncertainty=1.25                  # Massa 25% lebih berat dari model
    )

    ref_model_j1 = ReferenceModel(config.reference)
    ref_model_j2 = ReferenceModel(config.reference)
    controller = MRACController(controller_dynamics, config.controller, ref_model_j1, ref_model_j2)
    traj_func = trajectory_generator(config)

    # --- Import kinematics ---
    from models.kinematics import ForwardKinematics
    fk = ForwardKinematics(config.physical)

    # --- State vector ---
    # x = [q1, q2, dq1, dq2, qm1, dqm1, qm2, dqm2, theta(6)]
    # Total: 2 + 2 + 2 + 2 + 6 = 14 states
    n_states = 14

    def pack_state(q, dq, xm1, xm2, theta):
        return np.concatenate([q, dq, xm1, xm2, theta])

    def unpack_state(x):
        q = x[0:2]
        dq = x[2:4]
        xm1 = x[4:6]
        xm2 = x[6:8]
        theta = x[8:14]
        return q, dq, xm1, xm2, theta

    # --- Initial conditions ---
    q0 = np.zeros(2)
    dq0 = np.zeros(2)
    xm1_0 = np.zeros(2)
    xm2_0 = np.zeros(2)
    theta0 = np.zeros(6)
    x0 = pack_state(q0, dq0, xm1_0, xm2_0, theta0)

    def system_ode(t, x):
        """ODE sistem lengkap: plant + reference model + adaptation."""
        q, dq, xm1, xm2, theta = unpack_state(x)

        # Update controller theta
        controller.theta = theta.copy()

        # Trajectory desired
        q_d, dq_d, ddq_d = traj_func(t)
        r = q_d  # Reference input = desired position

        # Reference model dynamics
        dxm1 = ref_model_j1.state_derivative(xm1, r[0])
        dxm2 = ref_model_j2.state_derivative(xm2, r[1])

        # Error: plant output - reference model output
        qm = np.array([xm1[0], xm2[0]])
        e = q - qm

        # Desired from reference model for computed torque
        q_ref = qm
        dq_ref = np.array([xm1[1], xm2[1]])
        ddq_ref = np.array([
            ref_model_j1.Am[1, :] @ xm1 + ref_model_j1.Bm[1, 0] * r[0],
            ref_model_j2.Am[1, :] @ xm2 + ref_model_j2.Bm[1, 0] * r[1]
        ])

        # Compute torque
        tau = controller.compute_full_torque(q, dq, q_ref, dq_ref, ddq_ref, r)

        # Clamp torque (realistic actuator limits)
        tau = np.clip(tau, -500.0, 500.0)

        # Plant dynamics: forward dynamics
        ddq = plant_dynamics.forward_dynamics(q, dq, tau)

        # MIT Rule adaptation
        xm_states = np.array([[xm1[0], xm1[1]], [xm2[0], xm2[1]]])
        dtheta = controller.adaptation_law(e, q, dq, r, xm_states)

        # Pack derivatives
        dx = pack_state(dq, ddq, dxm1, dxm2, dtheta)

        return dx

    # --- Time span ---
    sim = config.simulation
    t_span = (sim.t_start, sim.t_end)
    t_eval = np.arange(sim.t_start, sim.t_end, sim.dt)

    # --- Solve ODE ---
    sol = solve_ivp(
        system_ode,
        t_span,
        x0,
        method=sim.method,
        t_eval=t_eval,
        rtol=1e-8,
        atol=1e-10,
        max_step=0.01
    )

    if not sol.success:
        print(f"  Warning ODE solver: {sol.message}")

    # --- Extract results ---
    t_out = sol.t
    N = len(t_out)

    q_out = sol.y[0:2, :].T
    dq_out = sol.y[2:4, :].T
    xm1_out = sol.y[4:6, :].T
    xm2_out = sol.y[6:8, :].T
    theta_out = sol.y[8:14, :].T

    qm_out = np.column_stack([xm1_out[:, 0], xm2_out[:, 0]])
    dqm_out = np.column_stack([xm1_out[:, 1], xm2_out[:, 1]])
    error_out = q_out - qm_out

    # Compute desired trajectory and torque at each time step
    qd_out = np.zeros((N, 2))
    tau_out = np.zeros((N, 2))
    ee_out = np.zeros((N, 3))

    for i in range(N):
        t_i = t_out[i]
        q_i = q_out[i]
        dq_i = dq_out[i]

        qd_out[i], _, _ = traj_func(t_i)
        ee_out[i] = fk.forward(q_i)

        # Recompute torque for recording
        controller.theta = theta_out[i].copy()
        r_i = qd_out[i]
        q_ref_i = qm_out[i]
        dq_ref_i = dqm_out[i]
        ddq_ref_i = np.array([
            ref_model_j1.Am[1, :] @ xm1_out[i] + ref_model_j1.Bm[1, 0] * r_i[0],
            ref_model_j2.Am[1, :] @ xm2_out[i] + ref_model_j2.Bm[1, 0] * r_i[1]
        ])
        tau_out[i] = controller.compute_full_torque(
            q_i, dq_i, q_ref_i, dq_ref_i, ddq_ref_i, r_i
        )
        tau_out[i] = np.clip(tau_out[i], -500.0, 500.0)

    # --- Compute Metrics ---
    metrics = compute_metrics(t_out, q_out, qm_out, qd_out, error_out, tau_out)

    result = SimResult(
        config=config,
        t=t_out,
        q=q_out,
        dq=dq_out,
        q_ref=qm_out,
        dq_ref=dqm_out,
        q_desired=qd_out,
        error=error_out,
        tau=tau_out,
        theta_adapt=theta_out,
        end_effector=ee_out,
        metrics=metrics
    )

    if verbose:
        print(f"    Done. SS Error J1={metrics['ss_error_j1']:.6f} rad, "
              f"J2={metrics['ss_error_j2']:.6f} rad")

    return result


def compute_metrics(
    t: np.ndarray,
    q: np.ndarray,
    q_ref: np.ndarray,
    q_desired: np.ndarray,
    error: np.ndarray,
    tau: np.ndarray
) -> dict:
    """
    Menghitung metrik performa simulasi.

    Returns:
        dict: Metrik (steady-state error, overshoot, settling time, dll.)
    """
    N = len(t)
    metrics = {}

    # Steady-state error (rata-rata 10% terakhir)
    last_10pct = int(0.9 * N)
    metrics['ss_error_j1'] = np.mean(np.abs(error[last_10pct:, 0]))
    metrics['ss_error_j2'] = np.mean(np.abs(error[last_10pct:, 1]))

    # Peak error
    metrics['peak_error_j1'] = np.max(np.abs(error[:, 0]))
    metrics['peak_error_j2'] = np.max(np.abs(error[:, 1]))

    # Overshoot (untuk step response)
    if q_desired[-1, 0] != 0:
        overshoot_j1 = (np.max(q[:, 0]) - q_desired[-1, 0]) / q_desired[-1, 0]
        metrics['overshoot_j1_pct'] = max(0, overshoot_j1) * 100
    else:
        metrics['overshoot_j1_pct'] = 0.0

    if q_desired[-1, 1] != 0:
        overshoot_j2 = (np.max(q[:, 1]) - q_desired[-1, 1]) / q_desired[-1, 1]
        metrics['overshoot_j2_pct'] = max(0, overshoot_j2) * 100
    else:
        metrics['overshoot_j2_pct'] = 0.0

    # Settling time (2% band)
    for joint_idx, joint_name in enumerate(['j1', 'j2']):
        target = q_desired[-1, joint_idx]
        if target != 0:
            band = 0.02 * abs(target)
            settled = np.abs(q[:, joint_idx] - target) < band
            # Temukan waktu terakhir keluar dari band
            if np.all(settled):
                metrics[f'settling_time_{joint_name}'] = 0.0
            else:
                last_outside = np.where(~settled)[0][-1]
                if last_outside < N - 1:
                    metrics[f'settling_time_{joint_name}'] = t[last_outside]
                else:
                    metrics[f'settling_time_{joint_name}'] = t[-1]
        else:
            metrics[f'settling_time_{joint_name}'] = 0.0

    # Max torque
    metrics['max_torque_j1'] = np.max(np.abs(tau[:, 0]))
    metrics['max_torque_j2'] = np.max(np.abs(tau[:, 1]))

    # RMS error
    metrics['rms_error_j1'] = np.sqrt(np.mean(error[:, 0]**2))
    metrics['rms_error_j2'] = np.sqrt(np.mean(error[:, 1]**2))

    # ISE (Integral Square Error)
    dt = t[1] - t[0] if len(t) > 1 else 0.001
    metrics['ise_j1'] = np.sum(error[:, 0]**2) * dt
    metrics['ise_j2'] = np.sum(error[:, 1]**2) * dt

    return metrics


def run_batch(
    configs: List[SimConfig],
    verbose: bool = True
) -> List[SimResult]:
    """
    Menjalankan batch simulasi (iterasi).

    Args:
        configs: List konfigurasi
        verbose: Print progress

    Returns:
        List[SimResult]: Hasil semua simulasi
    """
    results = []
    total = len(configs)

    if verbose:
        print(f"\n{'='*60}")
        print(f"  BATCH SIMULATION: {total} configurations")
        print(f"{'='*60}")

    for i, cfg in enumerate(configs):
        if verbose:
            print(f"\n[{i+1}/{total}]", end="")
        result = run_simulation(cfg, verbose=verbose)
        results.append(result)

    if verbose:
        print(f"\n{'='*60}")
        print(f"  ALL {total} SIMULATIONS COMPLETED")
        print(f"{'='*60}\n")

    return results
