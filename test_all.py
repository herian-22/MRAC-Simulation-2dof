"""
test_all.py — Comprehensive test script for MRAC 2-DoF system.

Tests each module independently, then runs a full simulation to verify
the entire pipeline works end-to-end.

Usage:
    python test_all.py
"""

import sys
import io
import traceback
import numpy as np

# Force UTF-8 on Windows
if sys.platform == 'win32':
    sys.stdout = io.TextIOWrapper(sys.stdout.buffer, encoding='utf-8', errors='replace')
    sys.stderr = io.TextIOWrapper(sys.stderr.buffer, encoding='utf-8', errors='replace')


PASS = 0
FAIL = 0


def test(name, func):
    """Run a test function and report pass/fail."""
    global PASS, FAIL
    try:
        func()
        print(f"  [PASS] {name}")
        PASS += 1
    except Exception as e:
        print(f"  [FAIL] {name}")
        traceback.print_exc()
        FAIL += 1


def banner(title):
    print(f"\n{'='*60}")
    print(f"  {title}")
    print(f"{'='*60}")


# ═══════════════════════════════════════════════════════════════
#  1. TEST: models.config
# ═══════════════════════════════════════════════════════════════

def test_config():
    banner("TEST: models.config")

    def test_physical_params():
        from models.config import PhysicalParams
        p = PhysicalParams()
        assert p.m1 == 8.0, f"m1 should be 8.0, got {p.m1}"
        assert p.m2 == 12.0, f"m2 should be 12.0, got {p.m2}"
        assert p.g == 9.81
        assert p.l1 == 0.40
        assert p.l2 == 0.80

    def test_controller_params():
        from models.config import ControllerParams
        c = ControllerParams()
        assert c.gamma1 == 770.0
        assert c.gamma2 == 935.0
        assert c.Kp1 == 100.0
        assert c.theta_max == 50.0

    def test_reference_params():
        from models.config import ReferenceModelParams
        r = ReferenceModelParams()
        assert r.overshoot == 0.15
        assert r.peak_time == 1.8
        # Check computed properties
        assert 0 < r.zeta < 1, f"zeta must be (0,1), got {r.zeta}"
        assert r.omega_n > 0, f"omega_n must be > 0, got {r.omega_n}"

    def test_sim_config():
        from models.config import SimConfig, default_config
        cfg = default_config()
        assert cfg.label == "default"
        assert cfg.simulation.trajectory_type == 'step'
        s = cfg.summary_str()
        assert 'g1=770' in s
        assert 'step' in s

    def test_sweep_gamma():
        from models.config import sweep_gamma
        configs = sweep_gamma(n_steps=3)
        assert len(configs) == 9  # 3 x 3
        for c in configs:
            assert 'gamma' in c.label

    def test_sweep_trajectory():
        from models.config import sweep_trajectory
        configs = sweep_trajectory()
        assert len(configs) == 3
        types = [c.simulation.trajectory_type for c in configs]
        assert 'step' in types
        assert 'sinusoidal' in types
        assert 'multipoint' in types

    test("PhysicalParams defaults", test_physical_params)
    test("ControllerParams defaults", test_controller_params)
    test("ReferenceModelParams zeta/omega_n", test_reference_params)
    test("SimConfig + default_config()", test_sim_config)
    test("sweep_gamma generates correct count", test_sweep_gamma)
    test("sweep_trajectory generates 3 types", test_sweep_trajectory)


# ═══════════════════════════════════════════════════════════════
#  2. TEST: models.dynamics
# ═══════════════════════════════════════════════════════════════

def test_dynamics():
    banner("TEST: models.dynamics")

    def test_inertia_matrix():
        from models.dynamics import Dynamics2DoF
        dyn = Dynamics2DoF()
        q = np.array([0.0, 0.0])
        M = dyn.inertia_matrix(q)
        assert M.shape == (2, 2), f"M shape should be (2,2), got {M.shape}"
        # M should be symmetric
        assert np.allclose(M, M.T), "M must be symmetric"
        # M should be positive definite
        eigvals = np.linalg.eigvals(M)
        assert np.all(eigvals > 0), f"M must be PD, eigvals={eigvals}"

    def test_coriolis_matrix():
        from models.dynamics import Dynamics2DoF
        dyn = Dynamics2DoF()
        q = np.array([np.pi/4, np.pi/6])
        dq = np.array([1.0, 0.5])
        C = dyn.coriolis_matrix(q, dq)
        assert C.shape == (2, 2)
        # C should be zero when dq=0
        C0 = dyn.coriolis_matrix(q, np.zeros(2))
        assert np.allclose(C0, 0), "C should be zero when dq=0"

    def test_gravity_vector():
        from models.dynamics import Dynamics2DoF
        dyn = Dynamics2DoF()
        q = np.array([0.0, 0.0])
        G = dyn.gravity_vector(q)
        assert G.shape == (2,)
        assert G[0] != 0, "G[0] should be nonzero at q=[0,0]"

    def test_forward_dynamics():
        from models.dynamics import Dynamics2DoF
        dyn = Dynamics2DoF()
        q = np.zeros(2)
        dq = np.zeros(2)
        tau = np.zeros(2)
        ddq = dyn.forward_dynamics(q, dq, tau)
        assert ddq.shape == (2,)
        # With zero torque, gravity should cause acceleration
        assert np.any(ddq != 0), "ddq should be nonzero (gravity effect)"

    def test_inverse_dynamics():
        from models.dynamics import Dynamics2DoF
        dyn = Dynamics2DoF()
        q = np.array([0.3, 0.2])
        dq = np.array([0.1, 0.05])
        ddq = np.array([0.0, 0.0])
        tau = dyn.inverse_dynamics(q, dq, ddq)
        assert tau.shape == (2,)
        # Verify consistency: if we apply this tau, ddq should be ~0
        ddq_check = dyn.forward_dynamics(q, dq, tau)
        assert np.allclose(ddq_check, ddq, atol=1e-6), \
            f"Inverse/forward mismatch: {ddq_check} vs {ddq}"

    def test_friction_uncertainty():
        from models.dynamics import Dynamics2DoF
        dyn_nom = Dynamics2DoF()
        dyn_unc = Dynamics2DoF(friction_coeff=np.array([2.5, 1.8]),
                                mass_uncertainty=1.25)
        q = np.array([0.5, 0.3])
        dq = np.array([1.0, 0.5])
        tau = np.array([10.0, 5.0])
        ddq_nom = dyn_nom.forward_dynamics(q, dq, tau)
        ddq_unc = dyn_unc.forward_dynamics(q, dq, tau)
        # They should differ
        assert not np.allclose(ddq_nom, ddq_unc), \
            "Nominal and uncertain plant should give different accelerations"

    test("Inertia matrix M(q) is SPD", test_inertia_matrix)
    test("Coriolis matrix C(q,dq)", test_coriolis_matrix)
    test("Gravity vector G(q)", test_gravity_vector)
    test("Forward dynamics ddq = M^-1(tau - Cdq - G)", test_forward_dynamics)
    test("Inverse/forward dynamics consistency", test_inverse_dynamics)
    test("Friction + mass uncertainty affects dynamics", test_friction_uncertainty)


# ═══════════════════════════════════════════════════════════════
#  3. TEST: models.kinematics
# ═══════════════════════════════════════════════════════════════

def test_kinematics():
    banner("TEST: models.kinematics")

    def test_dh_transform():
        from models.kinematics import ForwardKinematics
        T = ForwardKinematics.dh_transform(0, 0, 0, 0)
        assert T.shape == (4, 4)
        assert np.allclose(T, np.eye(4)), "Identity DH params -> I4"

    def test_forward_home():
        from models.kinematics import ForwardKinematics
        fk = ForwardKinematics()
        pos = fk.forward(np.array([0.0, 0.0]))
        assert pos.shape == (3,)
        assert pos[2] > 0, "Z should be positive (upright antenna)"

    def test_forward_varies():
        from models.kinematics import ForwardKinematics
        fk = ForwardKinematics()
        p1 = fk.forward(np.array([0.0, 0.0]))
        p2 = fk.forward(np.array([np.pi/4, np.pi/6]))
        assert not np.allclose(p1, p2), "Different joints -> different EE positions"

    def test_jacobian():
        from models.kinematics import ForwardKinematics
        fk = ForwardKinematics()
        q = np.array([0.3, 0.2])
        J = fk.jacobian(q)
        assert J.shape == (3, 2)
        # Numerical check: J * dq ≈ dp
        dq = np.array([0.001, 0.001])
        p0 = fk.forward(q)
        p1 = fk.forward(q + dq)
        dp_actual = p1 - p0
        dp_approx = J @ dq
        assert np.allclose(dp_actual, dp_approx, atol=1e-3), \
            f"Jacobian approximation mismatch"

    test("DH transform identity", test_dh_transform)
    test("Forward kinematics at home", test_forward_home)
    test("FK varies with joint angles", test_forward_varies)
    test("Jacobian numerical consistency", test_jacobian)


# ═══════════════════════════════════════════════════════════════
#  4. TEST: models.reference_model
# ═══════════════════════════════════════════════════════════════

def test_reference_model():
    banner("TEST: models.reference_model")

    def test_state_derivative():
        from models.reference_model import ReferenceModel
        ref = ReferenceModel()
        xm = np.array([0.0, 0.0])
        dxm = ref.state_derivative(xm, 1.0)
        assert dxm.shape == (2,)
        assert dxm[1] > 0, "Positive input should cause positive acceleration"

    def test_step_response():
        from models.reference_model import ReferenceModel
        ref = ReferenceModel()
        t = np.linspace(0, 10, 1000)
        y = ref.step_response_analytical(t, r=1.0)
        assert y.shape == (1000,)
        # Final value should approach 1.0
        assert abs(y[-1] - 1.0) < 0.02, f"y(inf) should ~1.0, got {y[-1]}"
        # Peak should be ~1.15 (15% overshoot)
        peak = np.max(y)
        assert 1.10 < peak < 1.20, f"Peak should be ~1.15, got {peak}"

    def test_specs():
        from models.reference_model import ReferenceModel
        ref = ReferenceModel()
        specs = ref.get_specs()
        assert 'zeta' in specs
        assert 'omega_n' in specs
        assert specs['overshoot_pct'] == 15.0

    def test_state_space_matrices():
        from models.reference_model import ReferenceModel
        ref = ReferenceModel()
        assert ref.Am.shape == (2, 2)
        assert ref.Bm.shape == (2, 1)
        # Am eigenvalues should have negative real parts (stable)
        eig = np.linalg.eigvals(ref.Am)
        assert np.all(np.real(eig) < 0), f"Am should be stable, eigvals={eig}"

    test("State derivative shape and sign", test_state_derivative)
    test("Step response converges to 1.0 with ~15% overshoot", test_step_response)
    test("Specs dictionary", test_specs)
    test("Am is Hurwitz (stable)", test_state_space_matrices)


# ═══════════════════════════════════════════════════════════════
#  5. TEST: control.controller
# ═══════════════════════════════════════════════════════════════

def test_controller():
    banner("TEST: control.controller")

    def test_computed_torque():
        from models.dynamics import Dynamics2DoF
        from models.config import ControllerParams
        from control.controller import ComputedTorqueController
        dyn = Dynamics2DoF()
        ct = ComputedTorqueController(dyn, ControllerParams())
        q = np.array([0.0, 0.0])
        dq = np.zeros(2)
        q_d = np.array([0.5, 0.3])
        tau = ct.compute_torque(q, dq, q_d, np.zeros(2), np.zeros(2))
        assert tau.shape == (2,)
        # Torque should be nonzero to drive toward target
        assert np.any(tau != 0)

    def test_mrac_adaptive_signal():
        from models.dynamics import Dynamics2DoF
        from models.config import ControllerParams, ReferenceModelParams
        from models.reference_model import ReferenceModel
        from control.controller import MRACController
        dyn = Dynamics2DoF()
        ref1 = ReferenceModel(ReferenceModelParams())
        ref2 = ReferenceModel(ReferenceModelParams())
        mrac = MRACController(dyn, ControllerParams(), ref1, ref2)
        # Initially theta=0, so adaptive signal should be 0
        u = mrac.adaptive_signal(np.zeros(2), np.zeros(2), np.array([0.5, 0.3]))
        assert np.allclose(u, 0), "Initial adaptive signal should be zero"

    def test_mit_rule_update():
        from models.dynamics import Dynamics2DoF
        from models.config import ControllerParams, ReferenceModelParams
        from models.reference_model import ReferenceModel
        from control.controller import MRACController
        dyn = Dynamics2DoF()
        ref1 = ReferenceModel(ReferenceModelParams())
        ref2 = ReferenceModel(ReferenceModelParams())
        mrac = MRACController(dyn, ControllerParams(), ref1, ref2)
        e = np.array([0.1, 0.05])
        q = np.array([0.3, 0.2])
        dq = np.array([0.05, 0.02])
        r = np.array([0.5, 0.3])
        xm = np.array([[0.25, 0.01], [0.18, 0.01]])
        dtheta = mrac.adaptation_law(e, q, dq, r, xm)
        assert dtheta.shape == (6,)
        # dtheta should be nonzero when error is nonzero
        assert np.any(dtheta != 0), "Adaptation should produce nonzero updates"

    def test_full_torque():
        from models.dynamics import Dynamics2DoF
        from models.config import ControllerParams, ReferenceModelParams
        from models.reference_model import ReferenceModel
        from control.controller import MRACController
        dyn = Dynamics2DoF()
        ref1 = ReferenceModel(); ref2 = ReferenceModel()
        mrac = MRACController(dyn, ControllerParams(), ref1, ref2)
        q = np.zeros(2); dq = np.zeros(2)
        r = np.array([0.5, 0.3])
        tau = mrac.compute_full_torque(q, dq, r, np.zeros(2), np.zeros(2), r)
        assert tau.shape == (2,)

    test("Computed torque produces nonzero output", test_computed_torque)
    test("MRAC initial adaptive signal is zero", test_mrac_adaptive_signal)
    test("MIT Rule produces nonzero dtheta", test_mit_rule_update)
    test("Full torque (CT + MRAC) runs", test_full_torque)


# ═══════════════════════════════════════════════════════════════
#  6. TEST: simulation.simulator (FULL ODE RUN)
# ═══════════════════════════════════════════════════════════════

def test_simulation():
    banner("TEST: simulation.simulator (full ODE — may take ~10s)")

    def test_full_run():
        from models.config import default_config
        from simulation.simulator import run_simulation
        cfg = default_config()
        cfg.simulation.t_end = 3.0  # Short for speed
        result = run_simulation(cfg, verbose=True)

        # Check shapes
        N = len(result.t)
        assert N > 100, f"Should have many time points, got {N}"
        assert result.q.shape == (N, 2)
        assert result.dq.shape == (N, 2)
        assert result.q_ref.shape == (N, 2)
        assert result.error.shape == (N, 2)
        assert result.tau.shape == (N, 2)
        assert result.theta_adapt.shape == (N, 6)
        assert result.end_effector.shape == (N, 3)

        # Check metrics exist
        m = result.metrics
        required_keys = [
            'ss_error_j1', 'ss_error_j2',
            'overshoot_j1_pct', 'overshoot_j2_pct',
            'settling_time_j1', 'settling_time_j2',
            'max_torque_j1', 'max_torque_j2',
            'rms_error_j1', 'rms_error_j2',
            'ise_j1', 'ise_j2',
        ]
        for key in required_keys:
            assert key in m, f"Missing metric: {key}"

        # Check that plant actually moves toward target
        final_q1 = result.q[-1, 0]
        target_q1 = cfg.simulation.q1_target
        assert abs(final_q1 - target_q1) < np.radians(5), \
            f"q1 should be near target. final={np.degrees(final_q1):.1f}, " \
            f"target={np.degrees(target_q1):.1f}"

    def test_trajectory_types():
        from models.config import default_config
        from simulation.simulator import run_simulation
        for traj in ['step', 'sinusoidal']:
            cfg = default_config()
            cfg.simulation.trajectory_type = traj
            cfg.simulation.t_end = 2.0
            result = run_simulation(cfg, verbose=False)
            assert len(result.t) > 50, f"{traj} trajectory should produce data"

    test("Full simulation run (step, 3s)", test_full_run)
    test("Multiple trajectory types", test_trajectory_types)


# ═══════════════════════════════════════════════════════════════
#  7. TEST: GUI imports (no display needed)
# ═══════════════════════════════════════════════════════════════

def test_gui_imports():
    banner("TEST: gui module imports")

    def test_imports():
        from gui.theme import Theme, build_stylesheet
        assert hasattr(Theme, 'BG_DARKEST')
        assert isinstance(build_stylesheet(), str)

    def test_blocks_import():
        from gui.blocks import SimulinkBlock, SignalArrow, FeedbackArrow, BLOCK_DEFINITIONS
        assert 'input' in BLOCK_DEFINITIONS
        assert 'plant' in BLOCK_DEFINITIONS
        assert 'mrac' in BLOCK_DEFINITIONS
        assert 'scope' in BLOCK_DEFINITIONS
        assert len(BLOCK_DEFINITIONS) == 6

    def test_modules_import():
        from gui.diagram import BlockDiagramView
        from gui.properties import PropertiesPanel
        from gui.scope import ScopePlotCanvas
        from gui.metrics import MetricsTable
        from gui.worker import SimulationWorker
        from gui.app import SimulinkGUI

    test("Theme + stylesheet", test_imports)
    test("Blocks definitions (6 types)", test_blocks_import)
    test("All GUI modules importable", test_modules_import)


# ═══════════════════════════════════════════════════════════════
#  MAIN
# ═══════════════════════════════════════════════════════════════

if __name__ == "__main__":
    print("\n" + "=" * 60)
    print("  MRAC 2-DoF SYSTEM — COMPREHENSIVE TEST SUITE")
    print("=" * 60)

    test_config()
    test_dynamics()
    test_kinematics()
    test_reference_model()
    test_controller()
    test_simulation()
    test_gui_imports()

    print(f"\n{'='*60}")
    print(f"  RESULTS: {PASS} passed, {FAIL} failed")
    print(f"{'='*60}")

    if FAIL > 0:
        print(f"\n  !!! {FAIL} test(s) FAILED — please investigate !!!")
        sys.exit(1)
    else:
        print(f"\n  All {PASS} tests PASSED!")
        sys.exit(0)
