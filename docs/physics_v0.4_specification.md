# Physics v0.4 Specification

**Status:** Draft normative specification  
**Applies to:** Physics v0.4 Python 3-DOF reference backend  
**Authority:** Physics implementation and tests must conform to this document.

This document defines the project's internal Physics contract. External source
selection, licenses, copied or adapted code, parameter provenance, and rejected
candidates belong in `open_source_reuse.md`. Verification results and plots
belong in a future `physics_v0.4_validation.md`.

Normative terms **MUST**, **MUST NOT**, **SHOULD**, and **MAY** indicate required,
prohibited, recommended, and optional behaviour respectively.

## 1. Purpose and scope

Physics v0.4 MUST implement a deterministic, pure-Python, vertical-plane 3-DOF
UUV reference physics backend. It receives and returns Core data types and
supports fixed-step simulation under no-current and constant-current inputs.
It MUST remain independent of ROS 2, Gazebo, and the Environment package.

The v0.4 implementation includes:

- a backend `step()` contract;
- reduced-state extraction and reconstruction;
- audited ENU/SNAME kinematic and current transformations;
- canonical 3-DOF parameter validation;
- matrix-form 3-DOF dynamics;
- Euler and fixed-step RK4 integration;
- SciPy `solve_ivp` comparison as offline validation only;
- synthetic parameters, mathematical tests, and a Physics smoke scenario.

The following are outside v0.4:

- a Gazebo backend or ROS 2 bridge;
- 6-DOF dynamics;
- collision detection or collision response;
- thruster, propeller, rudder, or fin dynamics;
- a complete REMUS 100 reproduction;
- parameter identification, CFD, waves, and turbulence;
- planner, trajectory-generation, or controller logic;
- mesh-based hydrodynamics or added-mass estimation.

## 2. Dependency contract

The allowed runtime dependency direction is:

```text
physics
├── core
└── numpy
```

SciPy is permitted only in validation utilities and tests that use
`solve_ivp` as a high-accuracy reference. The real-time/default fixed-step
simulation path MUST NOT call SciPy.

Physics:

- MUST NOT import `environment` or query `WorldModel`;
- MUST NOT import Planner, Controller, Simulation, ROS 2, or Gazebo;
- MUST NOT expose third-party simulator, ROS, Gazebo, or SDF types through its
  public interface;
- MUST accept environmental current only as the numerical value defined in
  Section 4;
- SHOULD use NumPy for all vector, matrix, and linear-system operations;
- MUST use `numpy.linalg.solve`, not an explicit matrix inverse, when solving
  the equations of motion.

The Simulation layer owns orchestration:

```python
current_velocity = world.current_at(state.pose, state.timestamp)
next_state = physics.step(state, control, current_velocity, dt)
```

## 3. Coordinate and frame contract

### 3.1 World frame

The world frame is ENU:

```text
x: East; also the vehicle-forward direction at zero pose
y: North
z: Up
```

### 3.2 Body frame

The vehicle body frame follows the SNAME convention:

```text
x_b: forward
y_b: starboard
z_b: down
```

### 3.3 Pitch convention

For the v0.4 vertical-plane model:

- `Pose.pitch > 0` MUST mean nose-up;
- `Twist.q > 0` MUST increase nose-up pitch;
- `ControlInput.tau_m > 0` MUST act in the positive-`q` direction.

This definition is normative for Physics even if a generic Euler-angle
library uses a differently labelled rotation convention.

### 3.4 Reduced state and Core mapping

The reduced configuration, velocity, and internal state are:

\[
\eta_r = [x, z, \theta]^T,
\qquad
\nu = [u, w, q]^T,
\qquad
y = [x, z, \theta, u, w, q]^T.
\]

| Reduced quantity | Core field | Frame and sign |
|---|---|---|
| \(x\) | `Pose.x` | world, East-positive |
| \(z\) | `Pose.z` | world, up-positive |
| \(\theta\) | `Pose.pitch` | nose-up-positive |
| \(u\) | `Twist.u` | body, forward-positive |
| \(w\) | `Twist.w` | body, down-positive |
| \(q\) | `Twist.q` | body, nose-up-rate-positive |
| \(\tau_u\) | `ControlInput.tau_x` | body surge force |
| \(\tau_w\) | `ControlInput.tau_z` | body heave force, down-positive |
| \(\tau_q\) | `ControlInput.tau_m` | body pitch moment |

Inactive Core components are `Pose.y`, `Pose.roll`, `Pose.yaw`, `Twist.v`,
`Twist.p`, and `Twist.r`. Input states MUST be rejected if the magnitude of
any inactive component exceeds `1e-12`; the backend MUST NOT silently discard
non-zero inactive motion. Reconstructed states MUST set every inactive
component to exactly `0.0`.

### 3.5 Kinematic mapping

The reduced body-to-world velocity mapping is:

\[
\dot{\eta}_r = J(\theta)\nu,
\]

\[
J(\theta)=
\begin{bmatrix}
\cos\theta & \sin\theta & 0\\
\sin\theta & -\cos\theta & 0\\
0 & 0 & 1
\end{bmatrix}.
\]

Therefore:

\[
\dot{x}=u\cos\theta+w\sin\theta,
\]

\[
\dot{z}=u\sin\theta-w\cos\theta,
\]

\[
\dot{\theta}=q.
\]

The upper-left matrix is not an ordinary planar rotation: it combines attitude
rotation with the body-z-down to world-z-up sign change. Implementations SHOULD
therefore use explicit names such as `body_to_world_velocity_matrix()` rather
than `rotation_matrix_2d()`.

The frame audit MUST include these hand-computed cases:

| State | Required result |
|---|---|
| \(\theta=0,u>0,w=0\) | \(\dot{x}>0,\dot{z}=0\) |
| \(\theta=0,u=0,w>0\) | \(\dot{x}=0,\dot{z}<0\) |
| \(\theta=+\pi/2,u>0,w=0\) | nose-up, \(\dot{z}>0\) |
| \(\theta=-\pi/2,u>0,w=0\) | nose-down, \(\dot{z}<0\) |

## 4. Current convention

Physics receives current velocity in the world ENU x-z plane:

\[
v_c^n=[v_{c,x},v_{c,z}]^T.
\]

The public input `current_velocity` MUST:

- have shape `(2,)`;
- use order `[v_cx, v_cz]`;
- use world ENU x-z coordinates;
- use metres per second;
- contain finite real values;
- use `v_cz > 0` for upward flow.

Let \(J_t\) be the upper-left 2-by-2 block of \(J\). It is symmetric and
orthogonal under this convention, so \(J_t^{-1}=J_t\). World current is mapped
to body current by:

\[
\begin{bmatrix}u_c\\w_c\end{bmatrix}
=
J_t(\theta)
\begin{bmatrix}v_{c,x}\\v_{c,z}\end{bmatrix}.
\]

The relative velocity used for hydrodynamic terms is:

\[
\nu_r=[u-u_c,\;w-w_c,\;q]^T.
\]

A uniform translational current has no angular-rate component. At zero pitch,
an upward current MUST produce `w_c < 0`, consistent with body `w` being
down-positive.

For one fixed `step()`, `current_velocity` is held constant through every
Euler/RK4 stage. A future time- or space-varying-current integration contract
MUST be introduced explicitly rather than hidden inside v0.4 Physics.

## 5. Public interfaces

The backend-wide contract contains only state advancement:

```python
from typing import Protocol
import numpy as np

from core import ControlInput, VehicleState


class PhysicsBackend(Protocol):
    def step(
        self,
        state: VehicleState,
        control: ControlInput,
        current_velocity: np.ndarray,
        dt: float,
    ) -> VehicleState:
        ...
```

Continuous local models use a separate interface:

```python
class ContinuousDynamics3DOF(Protocol):
    def derivatives(
        self,
        reduced_state: np.ndarray,
        reduced_control: np.ndarray,
        current_velocity: np.ndarray,
    ) -> StateDerivative3DOF:
        ...
```

`reduced_state` has shape `(6,)` in order `[x,z,pitch,u,w,q]`.
`reduced_control` has shape `(3,)` in order `[tau_x,tau_z,tau_m]`.
All inputs and outputs MUST use `float64`, have exact documented shapes, and
contain only finite values. Wrong types raise `TypeError`; wrong shapes,
non-finite values, invalid state components, and invalid `dt` raise
`ValueError`.

`dt` MUST be finite and strictly greater than zero. `step()` MUST return a new
`VehicleState` and MUST NOT mutate any input object or array.

## 6. Reduced-state mapping

Physics MUST provide pure functions equivalent to:

```python
extract_reduced_state(state: VehicleState) -> np.ndarray

rebuild_vehicle_state(
    reduced_state: np.ndarray,
    timestamp: float,
) -> VehicleState

extract_reduced_control(control: ControlInput) -> np.ndarray
```

`extract_reduced_state` validates inactive Core components before producing
`[x,z,pitch,u,w,q]`. `rebuild_vehicle_state` constructs a complete new Core
state and sets inactive fields to zero. It does not accept a template because
retaining inactive template fields could hide invalid state.

`extract_reduced_control` returns `[tau_x,tau_z,tau_m]`. It MUST reject
non-zero `tau_y`, `tau_k`, and `tau_n` rather than silently ignore them. Whether
`tau_z` affects the dynamics is decided by the parameterized input matrix.

## 7. Canonical parameter model

The v0.4 parameter schema is:

```python
@dataclass(frozen=True)
class UUV3DOFParameters:
    mass_rb: np.ndarray             # (3, 3)
    mass_added: np.ndarray          # (3, 3)
    damping_linear: np.ndarray      # (3, 3)
    damping_quadratic: np.ndarray   # (3, 3), diagonal in v0.4
    input_matrix: np.ndarray        # (3, 3)
    restoring_pitch_coefficient: float
```

The schema freezes representation and validation, not the final dissertation
vehicle values. The first configuration MUST be named
`uuv_3dof_synthetic_v1` and MUST be described as synthetic/test-only.

Required validation:

- every matrix has shape `(3,3)`, dtype `float64`, and finite values;
- `mass_rb` is symmetric positive definite;
- `mass_added` is symmetric positive semidefinite under the project's
  positive canonical added-mass convention;
- `M = mass_rb + mass_added` is symmetric positive definite;
- linear and quadratic damping are diagonal with non-negative coefficients;
- `input_matrix` is finite but MAY be rank-deficient;
- `restoring_pitch_coefficient` is finite and non-negative;
- constructor inputs are defensively copied and stored read-only.

Project damping coefficients are stored as positive dissipative quantities.
Literature derivatives such as negative `X_u` MUST be converted explicitly and
recorded in the reuse/provenance document.

The baseline dissertation configuration is underactuated:

\[
B_\tau=
\begin{bmatrix}
1&0&0\\
0&0&0\\
0&0&1
\end{bmatrix},
\qquad
B_\tau[\tau_x,\tau_z,\tau_m]^T=[\tau_x,0,\tau_m]^T.
\]

A separate fully actuated synthetic configuration MAY be used to isolate and
test the heave equation, but MUST NOT be described as the dissertation
baseline.

## 8. Dynamics contract

The target structural form is:

\[
M\dot{\nu}=
B_\tau\tau
-C_{RB}(\nu)\nu
-C_A(\nu_r)\nu_r
-D_L\nu_r
-D_Q(|\nu_r|\odot\nu_r)
-g(\eta_r),
\]

with:

\[
M=M_{RB}+M_A,
\qquad
g(\eta_r)=[0,0,k_\theta\sin\theta]^T.
\]

The implementation MUST solve this equation with `numpy.linalg.solve`.
Positive `k_theta` and the leading minus sign provide a restoring pitch moment
toward `pitch = 0`.

### 8.1 Coriolis derivation gate

The structural separation between rigid-body and added-mass Coriolis terms is
frozen. Their exact reduced 3-DOF matrix entries and signs are **not yet
approved**. Before `uuv_3dof.py` is implemented, the project MUST:

1. derive `C_RB` and `C_A` from a documented 3-DOF reduction;
2. identify whether each term uses absolute or relative velocity;
3. compare the result with a pinned authoritative/open-source reference;
4. verify the expected skew-symmetry or energy identity;
5. record the derivation and tests in this section or a linked derivation note.

No implementation may invent cross-coupling terms directly from scalar
intuition. If the first executable slice intentionally sets Coriolis to zero,
that simplification MUST be named, tested, and limited to a dedicated
sanity-check parameter/model configuration.

## 9. Integrator contract

Integrators operate on a pure derivative callable and a finite float64 state
vector. They do not import Core, Environment, or vehicle parameters.

### EulerIntegrator

- intended for hand calculations and one-step sanity tests;
- fixed-step and deterministic;
- not the default simulation integrator.

### RK4Integrator

- default Physics v0.4 integrator;
- classical explicit fixed-step fourth-order Runge-Kutta;
- deterministic and stateless;
- uses the caller-provided `dt` without adaptation.

### SciPy solve_ivp

- offline validation reference only;
- not an implementation of `PhysicsBackend`;
- not called by the normal fixed-step simulation loop.

Every backend step advances simulation time exactly as:

\[
t_{k+1}=t_k+dt.
\]

The returned timestamp MUST be finite. RK4 intermediate stages do not create
Core `VehicleState` objects; conversion occurs only at the public boundary.

## 10. Errors, determinism, and numerical policy

- Physics MUST reject non-finite state, control, current, parameter, derivative,
  and integration results.
- It MUST reject singular or invalid mass matrices during parameter creation,
  not during a later simulation step.
- It MUST NOT normalize, wrap, or clip pitch silently.
- It MUST NOT saturate controls; actuator constraints belong to a later layer.
- Identical state, control, current, parameters, integrator, and `dt` MUST
  produce identical output on the same supported numerical platform.
- Arrays returned across public boundaries MUST be independent of mutable
  internal parameter storage.

## 11. Package structure

The intended v0.4 layout is:

```text
physics/
├── __init__.py
├── backend.py
├── derivative.py
├── integrators.py
├── parameters.py
├── frames.py
└── uuv_3dof.py

config/vehicles/
├── uuv_3dof_synthetic_v1.yaml
└── parameter_sources.md

tests/physics/
├── test_frames.py
├── test_integrators.py
├── test_parameters.py
├── test_uuv_3dof_kinematics.py
├── test_uuv_3dof_dynamics.py
└── test_physics_step.py

simulation/
└── physics_smoke_demo.py

validation/
├── reference_cases/
└── compare_fossen_model.py
```

Files SHOULD be introduced incrementally; this tree does not authorize empty
placeholders or functionality outside the scope in Section 1.

## 12. Validation and acceptance criteria

Physics v0.4 is complete only when all applicable items are satisfied:

- [ ] The four hand-computed frame-audit cases pass.
- [ ] World-current to body-current conversion passes zero- and non-zero-pitch
      cases.
- [ ] Core/reduced-state/control mappings enforce shapes, ordering, inactive
      DOFs, and finite values.
- [ ] Parameter shape, symmetry, definiteness, diagonal damping, copying, and
      immutability tests pass.
- [ ] The approved Coriolis derivation gate in Section 8.1 is complete.
- [ ] Damping is demonstrably energy dissipative.
- [ ] Zero-input equilibrium behaves consistently with the restoring model.
- [ ] A simplified first-order damped surge case matches its analytical
      solution within a documented tolerance.
- [ ] Euler and RK4 errors decrease as `dt` decreases.
- [ ] RK4 exhibits an appropriate convergence trend against SciPy `solve_ivp`.
- [ ] Zero current and an explicit `[0,0]` current produce equivalent results.
- [ ] `step()` creates a new immutable `VehicleState` and advances timestamp by
      exactly `dt`.
- [ ] A Physics smoke scenario covers acceleration, damping decay, pitch input,
      current/no-current comparison, `Trajectory` output, and Environment
      boundary checks.
- [ ] Core and Environment regression suites remain green.
- [ ] No ROS 2, Gazebo, Planner, Controller, or Environment dependency exists
      inside the Physics package.
- [ ] Open-source code/parameters actually used are pinned and recorded in
      `open_source_reuse.md` before release.

## 13. Open-source and evidence boundary

Physics v0.4 follows the Fossen marine-craft modelling framework. This statement
does not by itself mean code or parameters were copied from a third party.

The future `open_source_reuse.md` is authoritative for:

- source name, repository/document, version or commit, and license;
- classification as dependency, adapted implementation, reference,
  parameter/data source, or rejected/deferred candidate;
- exact files/functions inspected or adapted;
- copyright and attribution obligations;
- adopted parameters, units, sign conversions, and confidence;
- validation performed and final decision status.

The future `physics_v0.4_validation.md` will report numerical cases, tolerances,
plots, reference outputs, convergence evidence, and deviations. Neither of
those documents may redefine the interfaces or mathematical conventions in
this specification.
