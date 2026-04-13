"""
simulator.py — MRAC Simulation Engine for 2-DoF Satellite Dish

Performs numerical integration of the plant + controller + reference model system.
Supports single runs and batch iterations.

Reference: Soares et al. (2021)
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
    """Results of a single simulation run."""
    config: SimConfig
    t: np.ndarray                # Time (N,)
    q: np.ndarray                # Joint angles (N, 2)
    dq: np.ndarray               # Joint velocities (N, 2)
    q_ref: np.ndarray            # Reference model output (N, 2)
    dq_ref: np.ndarray           # Reference model velocity (N, 2)
    q_desired: np.ndarray        # Target trajectory/setpoint (N, 2)
    error: np.ndarray            # Tracking error e = q - q_ref (N, 2)
    tau: np.ndarray              # Controller torque (N, 2)
    alpha_adapt: np.ndarray      # Adaptive parameters (N, 2)
    end_effector: np.ndarray     # End-effector position (N, 3)
    friction_torque: np.ndarray = None  # Friction/dissipative torque (N, 2)

    # --- Metrics ---
    metrics: dict = field(default_factory=dict)


def trajectory_generator(config: SimConfig) -> Callable:
    """
    Creates a trajectory function based on configuration.
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
    Runs a complete MRAC simulation.
    """
    if verbose:
        print(f"  Running: {config.label}...")

    controller_dynamics = Dynamics2DoF(config.physical)
    plant_dynamics = Dynamics2DoF(config.physical)
    # Reduce extreme friction so that transient (overshoot) is visible
    plant_dynamics.friction = np.array([80.0, 80.0])
    plant_dynamics.mass_unc = 1.0 # Paper explicitly added only dissipative force.

    ref_model_j1 = ReferenceModel(config.reference)
    ref_model_j2 = ReferenceModel(config.reference)
    controller = MRACController(controller_dynamics, config.controller, ref_model_j1, ref_model_j2)
    traj_func = trajectory_generator(config)

    from models.kinematics import ForwardKinematics
    fk = ForwardKinematics(config.physical)

    def pack_state(q, dq, xm1, xm2, phi1, phi2, alpha):
        return np.concatenate([q, dq, xm1, xm2, phi1, phi2, alpha])

    def unpack_state(x):
        q = x[0:2]
        dq = x[2:4]
        xm1 = x[4:6]
        xm2 = x[6:8]
        phi1 = x[8:10]
        phi2 = x[10:12]
        alpha = x[12:14]
        return q, dq, xm1, xm2, phi1, phi2, alpha

    x0 = pack_state(np.zeros(2), np.zeros(2), np.zeros(2), np.zeros(2), np.zeros(2), np.zeros(2), np.zeros(2))

    def system_ode(t, x):
        q, dq, xm1, xm2, phi1, phi2, alpha = unpack_state(x)

        controller.alpha = alpha.copy()

        q_d, _, _ = traj_func(t)
        r = q_d

        # Reference model dynamics
        dxm1 = ref_model_j1.state_derivative(xm1, r[0])
        dxm2 = ref_model_j2.state_derivative(xm2, r[1])

        # Filter phi dynamics for MIT rule (input is q_i / omega_n^2 so that Omega cancels out)
        dphi1 = ref_model_j1.state_derivative(phi1, q[0] / ref_model_j1.omega_n**2)
        dphi2 = ref_model_j2.state_derivative(phi2, q[1] / ref_model_j2.omega_n**2)

        # Error (relative to reference model trajectory)
        qm = np.array([xm1[0], xm2[0]])
        dqm = np.array([xm1[1], xm2[1]])
        e = q - qm

        # dphi parameter for sensitivity is phi_dot (velocity of phi)
        dphi_val = np.array([phi1[1], phi2[1]])
        
        # Adaptation law
        dalpha = controller.adaptation_law(e, dphi_val)

        # Compute torque tracking the reference model state (qm, dqm)
        tau = controller.compute_full_torque(q, dq, r, qm, dqm)
        tau = np.clip(tau, -500.0, 500.0)

        # Plant dynamics: forward dynamics
        ddq = plant_dynamics.forward_dynamics(q, dq, tau)

        # Pack derivatives
        dx = pack_state(dq, ddq, dxm1, dxm2, dphi1, dphi2, dalpha)
        return dx

    sim = config.simulation
    t_span = (sim.t_start, sim.t_end)
    t_eval = np.arange(sim.t_start, sim.t_end, sim.dt)

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

    t_out = sol.t
    N = len(t_out)

    q_out = sol.y[0:2, :].T
    dq_out = sol.y[2:4, :].T
    xm1_out = sol.y[4:6, :].T
    xm2_out = sol.y[6:8, :].T
    alpha_out = sol.y[12:14, :].T

    qm_out = np.column_stack([xm1_out[:, 0], xm2_out[:, 0]])
    dqm_out = np.column_stack([xm1_out[:, 1], xm2_out[:, 1]])
    error_out = q_out - qm_out

    # Compute desired trajectory and torque at each time step
    qd_out = np.zeros((N, 2))
    tau_out = np.zeros((N, 2))
    ee_out = np.zeros((N, 3))
    friction_out = np.zeros((N, 2))

    for i in range(N):
        t_i = t_out[i]
        q_i = q_out[i]
        dq_i = dq_out[i]

        qd_out[i], _, _ = traj_func(t_i)
        ee_out[i] = fk.forward(q_i)

        # Record friction/dissipative torque
        friction_out[i] = plant_dynamics.friction * dq_i

        controller.alpha = alpha_out[i]
        tau_out[i] = controller.compute_full_torque(q_i, dq_i, qd_out[i], qm_out[i], dqm_out[i])
        tau_out[i] = np.clip(tau_out[i], -500.0, 500.0)

    # Compute Metrics
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
        alpha_adapt=alpha_out,
        end_effector=ee_out,
        friction_torque=friction_out,
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
    Calculates simulation performance metrics.
    """
    N = len(t)
    metrics = {}

    last_10pct = int(0.9 * N)
    metrics['ss_error_j1'] = np.mean(np.abs(error[last_10pct:, 0]))
    metrics['ss_error_j2'] = np.mean(np.abs(error[last_10pct:, 1]))

    metrics['peak_error_j1'] = np.max(np.abs(error[:, 0]))
    metrics['peak_error_j2'] = np.max(np.abs(error[:, 1]))

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

    for joint_idx, joint_name in enumerate(['j1', 'j2']):
        target = q_desired[-1, joint_idx]
        if target != 0:
            band = 0.02 * abs(target)
            settled = np.abs(q[:, joint_idx] - target) < band
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

    metrics['max_torque_j1'] = np.max(np.abs(tau[:, 0]))
    metrics['max_torque_j2'] = np.max(np.abs(tau[:, 1]))

    metrics['rms_error_j1'] = np.sqrt(np.mean(error[:, 0]**2))
    metrics['rms_error_j2'] = np.sqrt(np.mean(error[:, 1]**2))

    dt = t[1] - t[0] if len(t) > 1 else 0.001
    metrics['ise_j1'] = np.sum(error[:, 0]**2) * dt
    metrics['ise_j2'] = np.sum(error[:, 1]**2) * dt

    return metrics


def run_batch(
    configs: List[SimConfig],
    verbose: bool = True
) -> List[SimResult]:
    """
    Runs a batch of simulations (iterations).
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
