# Physics v0.4 Specification

**Status:** Active normative specification  
**Applies to:** Physics v0.4 Python 3-DOF reference backend  
**Authority:** Physics implementation and tests must conform to this document.

This document defines the project's internal Physics contract. External source
selection, licenses, copied or adapted code, parameter provenance, and rejected
candidates belong in `open_source_reuse.md`. Verification results and plots
belong in `physics_v0.4_validation.md`.

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

### 3.3 Reduced-model physical assumptions

The v0.4 reduced model uses the following physical idealizations:

- the body-frame origin is located at the vehicle centre of gravity (CG);
- the body x-z plane is a symmetry plane;
- products of inertia and off-diagonal added-mass terms are neglected;
- the vehicle is neutrally buoyant in translation, so the reduced restoring
  vector contains no constant surge or heave force;
- the centre of buoyancy lies vertically above the centre of gravity;
- the resulting pitch-restoring moment is represented by the reduced
  coefficient `restoring_pitch_coefficient` and the term
  `k_theta * sin(pitch)`.

These are explicit reference-model assumptions, not claims that every real UUV
has diagonal hydrodynamic matrices or exact neutral buoyancy. Moving the body
origin, introducing translational buoyancy imbalance, or using CG/CB geometry
directly requires a new parameter contract and a new dynamics derivation.

### 3.4 Pitch convention

For the v0.4 vertical-plane model:

- `Pose.pitch > 0` MUST mean nose-up;
- `Twist.q > 0` MUST increase nose-up pitch;
- `ControlInput.tau_m > 0` MUST act in the positive-`q` direction.

This definition is normative for Physics even if a generic Euler-angle
library uses a differently labelled rotation convention.

### 3.5 Reduced state and Core mapping

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

### 3.6 Kinematic mapping

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

The minus sign on the body-heave term in `z_dot` is normative. Any earlier
project formula using the opposite sign is superseded because Core world `z`
is up-positive while SNAME body `w` is down-positive.

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

Integrators consume the same derivative value type through a state-only
callable closed over control/current for one backend step:

```python
class DerivativeFunction3DOF(Protocol):
    def __call__(self, reduced_state: np.ndarray) -> StateDerivative3DOF:
        ...


class Integrator(Protocol):
    def step(
        self,
        derivative: DerivativeFunction3DOF,
        state: np.ndarray,
        dt: float,
    ) -> np.ndarray:
        ...
```

`ContinuousDynamics3DOF.derivatives()` and `DerivativeFunction3DOF` MUST
return `StateDerivative3DOF`, never a bare array. Integrators convert it with
`to_numpy()` and return a validated reduced state of shape `(6,)`.

The derivative result is formally defined as:

```python
@dataclass(frozen=True)
class StateDerivative3DOF:
    eta_dot: np.ndarray  # (3,), [x_dot, z_dot, pitch_dot]
    nu_dot: np.ndarray   # (3,), [u_dot, w_dot, q_dot]

    def to_numpy(self) -> np.ndarray:
        """Return [x_dot, z_dot, pitch_dot, u_dot, w_dot, q_dot]."""
        ...
```

`StateDerivative3DOF` MUST defensively convert and copy both inputs, validate
shape `(3,)` and finite real values, store read-only `float64` arrays, and
return a new read-only or independent `float64` array from `to_numpy()`.

`reduced_state` has shape `(6,)` in order `[x,z,pitch,u,w,q]`.
`reduced_control` has shape `(3,)` in order `[tau_x,tau_z,tau_m]`.

Public numerical inputs MAY be integer- or floating-point arrays/sequences, but
MUST be convertible to finite real `float64` values. Implementations MUST
reject complex, object, string, and boolean arrays rather than silently cast
them. Inputs are defensively converted and copied to `float64` before use. All
internal and returned numerical arrays MUST use `float64` and the exact
documented shape. Wrong semantic types raise `TypeError`; wrong shapes,
non-finite values, prohibited dtypes, invalid state components, and invalid
`dt` raise `ValueError`.

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
non-zero `tau_y`, `tau_k`, and `tau_n` rather than silently ignore them.

`ControlInput` represents generalized force already applied to the vehicle; it
is not a thruster command. The parameter `input_matrix` defines the admissible
actuation subspace for a model configuration. Before integration, `step()`
MUST reject any non-zero reduced-control component disabled by
`input_matrix`. In particular, the underactuated baseline MUST reject non-zero
`tau_z` with an explicit `ValueError`; it MUST NOT multiply it by zero and
continue silently. A future actuator-allocation layer MAY define an explicit
projection policy, but v0.4 does not.

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

- constructor inputs follow the safe real-to-`float64` conversion policy in
  Section 5; every stored matrix has shape `(3,3)`, dtype `float64`, and finite
  values;
- `mass_rb` is diagonal and positive definite;
- the surge and heave diagonal entries of `mass_rb` are equal to the same
  physical vehicle mass;
- `mass_added` is diagonal and positive semidefinite under the project's
  positive canonical added-mass convention;
- `M = mass_rb + mass_added` is symmetric positive definite;
- linear and quadratic damping are diagonal with non-negative coefficients;
- `input_matrix` is a diagonal selector whose entries are exactly `0.0` or
  `1.0`; it MAY be rank-deficient;
- `restoring_pitch_coefficient` is finite and non-negative;
- constructor inputs are defensively copied and stored read-only.

Off-diagonal rigid-body inertia, added mass, damping, and actuation coupling are
outside v0.4. Supporting them requires an expanded derivation and parameter
contract rather than merely relaxing validation.

Project damping coefficients are stored as positive dissipative quantities.
Literature derivatives such as negative `X_u` MUST be converted explicitly and
recorded in the reuse/provenance document.

Canonical added-mass entries are physical positive inertia contributions. For
the diagonal v0.4 model:

\[
M_A=\operatorname{diag}(A_u,A_w,A_q),\qquad A_u,A_w,A_q\geq0.
\]

When importing traditional hydrodynamic derivatives, the conversion is
`A_u = -X_udot`, `A_w = -Z_wdot`, and `A_q = -M_qdot`. Negative literature or
legacy-plugin derivative values MUST NOT be stored directly in `mass_added`.

Parameter and signal units are:

| Quantity | Unit |
|---|---|
| surge/heave rigid-body or added mass | kg |
| pitch rigid-body or added inertia | kg m^2 |
| surge/heave linear damping | N s/m |
| pitch linear damping | N m s/rad |
| surge/heave quadratic damping | N s^2/m^2 |
| pitch quadratic damping | N m s^2/rad^2 |
| restoring pitch coefficient | N m |
| `tau_x`, `tau_z` | N |
| `tau_m` | N m |
| position | m |
| pitch | rad |
| simulation time | s |

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

`UUV3DOFParameters` MUST provide an admissibility check equivalent to
`is_control_admissible(reduced_control, tolerance=1e-12)`. For the selector
matrix used in v0.4, a control is admissible only when every disabled component
is zero within tolerance. This check occurs before applying `B_tau`.

## 8. Dynamics contract

For a world-fixed uniform current, define the body-current derivative:

\[
\dot{\nu}_c=[-q w_c,\;q u_c,\;0]^T.
\]

This follows from `v_c_dot_body = -S(omega) v_c_body`. The target absolute-
velocity structural form is:

\[
M\dot{\nu}=
B_\tau\tau
-C_{RB}(\nu)\nu
-C_A(\nu_r)\nu_r
-D_L\nu_r
-D_Q(|\nu_r|\odot\nu_r)
-g(\eta_r)
+M_A\dot{\nu}_c,
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

### 8.1 Approved Coriolis reduction

The derivation gate is approved in
[`physics_3dof_derivation.md`](physics_3dof_derivation.md). With
`M_RB = diag(m,m,I_y)`, `M_A = diag(A_u,A_w,A_q)`, absolute
`nu = [u,w,q]`, and relative `nu_r = [u_r,w_r,q]`, v0.4 MUST use:

\[
C_{RB}(\nu)=
\begin{bmatrix}
0&m q&0\\
-m q&0&0\\
0&0&0
\end{bmatrix},
\]

\[
C_A(\nu_r)=
\begin{bmatrix}
0&0&A_w w_r\\
0&0&-A_u u_r\\
-A_w w_r&A_u u_r&0
\end{bmatrix}.
\]

Both matrices are skew-symmetric, so `x.T @ C(x) @ x == 0` up to floating-
point roundoff. Rigid-body Coriolis acts on absolute velocity; added-mass
Coriolis acts on relative velocity. The `M_A * nu_c_dot` term above is
mandatory when a non-zero world-fixed current is expressed in the rotating
body frame.

## 9. Integrator contract

Integrators operate on a pure derivative callable and a finite float64 state
vector. They do not import Core, Environment, or vehicle parameters.

Both fixed-step implementations MUST satisfy the `Integrator` Protocol in
Section 5 and accept only derivative callables returning
`StateDerivative3DOF`.

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

Every backend step computes simulation time using:

\[
t_{k+1}=t_k+dt.
\]

The backend MUST calculate the returned timestamp using exactly the expression
`state.timestamp + dt`; it MUST NOT derive it from an internal step counter or
an RK stage. Tests compare the result to the same expression, not to an ideal
decimal literal. The returned timestamp MUST be finite. RK4 intermediate stages
do not create Core `VehicleState` objects; conversion occurs only at the public
boundary.

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
└── uuv_3dof_synthetic_v1.yaml

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

- [x] The four hand-computed frame-audit cases pass.
- [x] World-current to body-current conversion passes zero- and non-zero-pitch
      cases.
- [x] Core/reduced-state/control mappings enforce shapes, ordering, inactive
      DOFs, and finite values.
- [x] Public numerical inputs accept safe real-to-`float64` conversion and
      reject complex, object, string, boolean, wrong-shape, and non-finite data.
- [x] `StateDerivative3DOF` shape, ordering, copying, finite-value, dtype, and
      immutability tests pass.
- [x] Parameter shape, diagonal structure, definiteness, units, copying, and
      immutability tests pass.
- [x] Underactuated parameters reject non-zero `tau_z`; fully actuated test
      parameters accept it.
- [x] The approved Coriolis derivation gate in Section 8.1 is complete.
- [x] Damping is demonstrably energy dissipative.
- [x] Zero-input equilibrium behaves consistently with the restoring model.
- [x] A simplified first-order damped surge case matches its analytical
      solution within a documented tolerance.
- [x] Euler and RK4 errors decrease as `dt` decreases.
- [x] RK4 exhibits an appropriate convergence trend against SciPy `solve_ivp`.
- [x] Zero current and an explicit `[0,0]` current produce equivalent results.
- [x] `step()` creates a new immutable `VehicleState` and advances timestamp by
      exactly `dt`.
- [x] A Physics smoke scenario covers acceleration, damping decay, pitch input,
      current/no-current comparison, `Trajectory` output, and Environment
      boundary checks.
- [x] Core and Environment regression suites remain green.
- [x] No ROS 2, Gazebo, Planner, Controller, or Environment dependency exists
      inside the Physics package.
- [x] Open-source code/parameters actually used are pinned and recorded in
      `open_source_reuse.md` before release.

## 13. Open-source and evidence boundary

Physics v0.4 follows the Fossen marine-craft modelling framework. This statement
does not by itself mean code or parameters were copied from a third party.

`open_source_reuse.md` is authoritative for:

- source name, repository/document, version or commit, and license;
- classification as dependency, adapted implementation, reference,
  parameter/data source, or rejected/deferred candidate;
- exact files/functions inspected or adapted;
- copyright and attribution obligations;
- adopted parameters, units, sign conversions, and confidence;
- validation performed and final decision status.

`physics_v0.4_validation.md` reports numerical cases, tolerances,
plots, reference outputs, convergence evidence, and deviations. Neither of
those documents may redefine the interfaces or mathematical conventions in
this specification.
