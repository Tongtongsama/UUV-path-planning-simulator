# Physics v0.4 Validation Report

**Status:** Active validation record  
**Model:** `uuv_3dof_synthetic_v1`  
**Validation date:** 2026-09-03

## 1. Scope

This report records reproducible evidence for the Python 3-DOF reference
backend. It validates frame and state mappings, parameter constraints,
analytical special cases, fixed-step integration, current handling, and
regression behaviour. It does not identify a real vehicle or validate the
synthetic coefficients against experiments.

The normative equations remain in `physics_v0.4_specification.md`; their
derivation is recorded in `physics_3dof_derivation.md`.

## 2. Environments

| Purpose | Python | NumPy | SciPy | Notes |
|---|---:|---:|---:|---|
| Normal project tests | 3.10.9 | 2.2.6 | Not installed | SciPy validation test skipped |
| Offline reference validation | 3.11.5 | 1.24.3 | 1.11.1 | Windows 10; Anaconda environment |

SciPy is not imported by `physics/` or used by the normal simulation path.
The offline comparison command is:

```text
C:\ProgramData\anaconda3\python.exe -m validation.compare_fossen_model
```

## 3. Reference method

`scipy.integrate.solve_ivp` with method `DOP853`, `rtol=1e-11`, and
`atol=1e-13` provides the adaptive reference endpoint. Fixed-step Euler and
RK4 use `dt = [0.2, 0.1, 0.05, 0.025]` seconds. Error is the Euclidean norm of
the six-component final-state difference.

All cases run for 5 seconds with the same continuous dynamics and synthetic
parameter configuration. This isolates integrator error; it is not an
independent validation of the physical equations.

## 4. Convergence results

### 4.1 Damped free response

Initial state is `[0,0,0.15,1.0,0.2,0]`, with zero control and zero current.

| dt (s) | Euler final error | RK4 final error |
|---:|---:|---:|
| 0.200 | 4.150750e-2 | 5.176465e-7 |
| 0.100 | 2.039192e-2 | 3.262170e-8 |
| 0.050 | 1.010845e-2 | 2.048434e-9 |
| 0.025 | 5.032768e-3 | 1.282156e-10 |

Finest observed orders are Euler `1.006` and RK4 `3.998`.

### 4.2 Constant surge thrust

Initial state is zero, control is `[10,0,0]`, and current is zero.

| dt (s) | Euler final error | RK4 final error |
|---:|---:|---:|
| 0.200 | 7.765056e-2 | 5.256994e-8 |
| 0.100 | 3.845672e-2 | 3.176080e-9 |
| 0.050 | 1.913755e-2 | 1.957051e-10 |
| 0.025 | 9.546232e-3 | 1.269953e-11 |

Finest observed orders are Euler `1.003` and RK4 `3.946`.

### 4.3 Constant world current with pitch motion

Initial state is `[0,0,0.2,0.4,-0.1,0.15]`, control is `[2,0,-0.5]`, and
world current is `[0.3,-0.1]` m/s. This exercises the world/body current
transform and added-mass current-rotation correction.

| dt (s) | Euler final error | RK4 final error |
|---:|---:|---:|
| 0.200 | 7.494513e-2 | 3.756609e-6 |
| 0.100 | 3.460082e-2 | 2.547554e-7 |
| 0.050 | 1.664056e-2 | 1.594126e-8 |
| 0.025 | 8.162604e-3 | 9.490281e-10 |

Finest observed orders are Euler `1.028` and RK4 `4.070`.

## 5. Analytical and structural checks

The automated suite additionally verifies:

- the four ENU/SNAME hand-computed direction cases;
- upward world current maps to negative body heave at zero pitch;
- Core/reduced state and generalized-force ordering;
- inactive-DOF rejection;
- diagonal positive mass and non-negative damping/added-mass constraints;
- `input_matrix` selector admissibility and baseline `tau_z` rejection;
- exact reduced Coriolis entries and skew symmetry;
- `nu.T @ C(nu) @ nu = 0` within `1e-12`;
- zero-input/zero-current equilibrium;
- restoring pitch acceleration points toward zero pitch;
- axis-aligned force-to-acceleration scalar cases;
- linear free-surge response agrees with
  `u(t)=u(0)*exp(-D_u*t/M_u)` to relative tolerance `1e-9`;
- deterministic immutable backend stepping and exact timestamp expression.

## 6. Regression and smoke evidence

The normal Python environment reports `384 passed, 1 skipped`; the skipped
test is the SciPy-only convergence test. The validation environment reports
the SciPy convergence test passing. Its full repository suite is not used as
regression evidence because that environment does not currently contain
Shapely; Core/Environment regression evidence comes from the normal project
environment.

The executable smoke scenario reports:

```text
Physics v0.4 smoke demo completed
Free response: u 1.000 -> 0.168 m/s
Constant thrust: x=16.171 m, u=1.630 m/s
Pitch response peak pitch: 0.257 rad
NoCurrent equals explicit zero: True
All trajectories inside boundary: True
```

## 7. Reproducible artifact

- Results: `validation/reference_cases/physics_v0.4_results.json`
- Generator: `validation/compare_fossen_model.py`
- SHA-256: `c17b22b5a3d7c156a759d6cc4158cc7ecb5acba03eb8f60ead019e91fc703662`
- Schema version: 1

The JSON preserves environment versions, inputs, tolerances, reference final
states, every fixed-step final state, errors, and observed orders.

## 8. Current conclusion and limitations

The fixed-step implementation passes the intended numerical and software
contract checks. The observed convergence trends support first-order Euler and
fourth-order RK4 behaviour for all three cases, including non-zero world
current and pitch motion.

This does not establish real-UUV fidelity. Remaining higher-level evidence
includes experimental or independently parameterized vehicle comparison,
long-horizon sensitivity analysis, and future Gazebo-backend comparison. Such
work must use new provenance records and must not retroactively describe the
synthetic model as REMUS 100.
