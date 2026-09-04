"""Offline convergence validation for the Physics v0.4 reference backend."""

from __future__ import annotations

import json
import platform
from pathlib import Path

import numpy as np
import scipy
from scipy.integrate import solve_ivp

from physics import EulerIntegrator, Python3DOFDynamics, RK4Integrator
from simulation.vehicle_config import load_synthetic_parameters


OUTPUT_PATH = Path(__file__).resolve().parent / "reference_cases" / "physics_v0.4_results.json"
STEP_SIZES = (0.2, 0.1, 0.05, 0.025)


def _callable(dynamics, control, current):
    def derivative(state):
        return dynamics.derivatives(state, control, current)

    return derivative


def _fixed_final(integrator, derivative, initial, duration, dt):
    state = np.array(initial, dtype=np.float64, copy=True)
    steps = round(duration / dt)
    if not np.isclose(steps * dt, duration):
        raise ValueError("duration must be an integer multiple of dt")
    for _ in range(steps):
        state = integrator.step(derivative, state, dt)
    return state


def _reference_final(derivative, initial, duration):
    result = solve_ivp(
        lambda _time, state: derivative(state).to_numpy(),
        (0.0, duration),
        np.asarray(initial, dtype=np.float64),
        method="DOP853",
        rtol=1e-11,
        atol=1e-13,
    )
    if not result.success:
        raise RuntimeError(result.message)
    return result.y[:, -1], result.nfev


def _observed_orders(errors):
    return [
        float(np.log(previous / current) / np.log(2.0))
        for previous, current in zip(errors, errors[1:])
    ]


def _case(name, initial, control, current, duration):
    dynamics = Python3DOFDynamics(load_synthetic_parameters())
    derivative = _callable(
        dynamics,
        np.asarray(control, dtype=np.float64),
        np.asarray(current, dtype=np.float64),
    )
    reference, evaluations = _reference_final(derivative, initial, duration)
    methods = {}
    for method_name, integrator in (
        ("euler", EulerIntegrator()),
        ("rk4", RK4Integrator()),
    ):
        errors = []
        finals = []
        for dt in STEP_SIZES:
            final = _fixed_final(integrator, derivative, initial, duration, dt)
            finals.append(final.tolist())
            errors.append(float(np.linalg.norm(final - reference, ord=2)))
        methods[method_name] = {
            "final_states": finals,
            "l2_final_errors": errors,
            "observed_orders": _observed_orders(errors),
        }
    return {
        "name": name,
        "duration": duration,
        "initial_state": list(initial),
        "control": list(control),
        "world_current": list(current),
        "solve_ivp": {
            "method": "DOP853",
            "rtol": 1e-11,
            "atol": 1e-13,
            "function_evaluations": evaluations,
            "final_state": reference.tolist(),
        },
        "fixed_step_sizes": list(STEP_SIZES),
        "methods": methods,
    }


def run_validation():
    cases = [
        _case(
            "damped_free_response",
            [0.0, 0.0, 0.15, 1.0, 0.2, 0.0],
            [0.0, 0.0, 0.0],
            [0.0, 0.0],
            5.0,
        ),
        _case(
            "constant_surge_thrust",
            [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            [10.0, 0.0, 0.0],
            [0.0, 0.0],
            5.0,
        ),
        _case(
            "constant_world_current_with_pitch",
            [0.0, 0.0, 0.2, 0.4, -0.1, 0.15],
            [2.0, 0.0, -0.5],
            [0.3, -0.1],
            5.0,
        ),
    ]
    return {
        "schema_version": 1,
        "model": "uuv_3dof_synthetic_v1",
        "environment": {
            "python": platform.python_version(),
            "numpy": np.__version__,
            "scipy": scipy.__version__,
            "platform": platform.platform(),
        },
        "cases": cases,
    }


def main():
    results = run_validation()
    OUTPUT_PATH.parent.mkdir(parents=True, exist_ok=True)
    OUTPUT_PATH.write_text(json.dumps(results, indent=2) + "\n", encoding="utf-8")
    for case in results["cases"]:
        euler = case["methods"]["euler"]
        rk4 = case["methods"]["rk4"]
        print(case["name"])
        print(f"  Euler finest error: {euler['l2_final_errors'][-1]:.6e}")
        print(f"  RK4 finest error:   {rk4['l2_final_errors'][-1]:.6e}")
        print(f"  Euler final order:  {euler['observed_orders'][-1]:.3f}")
        print(f"  RK4 final order:    {rk4['observed_orders'][-1]:.3f}")
    print(f"Wrote {OUTPUT_PATH}")


if __name__ == "__main__":
    main()
