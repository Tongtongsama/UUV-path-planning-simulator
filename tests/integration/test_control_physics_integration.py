import numpy as np

from core import ControlInput, Trajectory
from simulation.control_tracking_demo import run_control_tracking_demo


def position_error(state, reference):
    return np.hypot(reference.pose.x - state.pose.x, reference.pose.z - state.pose.z)


def test_closed_loop_reduces_position_error_with_and_without_current():
    result = run_control_tracking_demo(dt=0.05, steps=1200)
    initial = position_error(result.no_current.trajectory[0], result.reference)
    for run in (result.no_current, result.constant_current):
        assert isinstance(run.trajectory, Trajectory)
        assert len(run.trajectory) == 1201
        assert len(run.controls) == 1200
        assert position_error(run.trajectory[-1], result.reference) < 0.15 * initial
        assert all(np.all(np.isfinite(state.to_numpy())) for state in run.trajectory)
        assert all(isinstance(control, ControlInput) for control in run.controls)


def test_controller_outputs_are_physics_admissible_and_bounded():
    result = run_control_tracking_demo(dt=0.05, steps=300)
    for run in (result.no_current, result.constant_current):
        assert all(control.tau_y == 0.0 for control in run.controls)
        assert all(control.tau_z == 0.0 for control in run.controls)
        assert all(control.tau_k == 0.0 for control in run.controls)
        assert all(control.tau_n == 0.0 for control in run.controls)
        assert max(abs(control.tau_x) for control in run.controls) <= 20.0
        assert max(abs(control.tau_m) for control in run.controls) <= 8.0


def test_closed_loop_demo_is_deterministic():
    first = run_control_tracking_demo(dt=0.1, steps=20)
    second = run_control_tracking_demo(dt=0.1, steps=20)
    assert first == second
