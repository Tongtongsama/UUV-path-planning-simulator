# Control v0.5 Specification

**Status:** Implemented normative specification  
**Applies to:** underactuated 3-DOF surge-pitch trajectory-tracking baseline  
**Authority:** Control implementation and tests must conform to this document.

This document defines the internal Control contract. It does not redefine Core
data types, Physics equations, trajectory sampling, actuator allocation, or
Environment queries. Normative terms **MUST**, **MUST NOT**, **SHOULD**, and
**MAY** indicate required, prohibited, recommended, and optional behaviour.

## 1. Purpose and scope

Control v0.5 MUST provide a deterministic baseline controller for the existing
vertical-plane 3-DOF Python Physics backend. It consumes a current
`VehicleState` and one already-selected reference `VehicleState`, then produces
a complete Core `ControlInput` containing admissible surge force and pitch
moment.

The v0.5 implementation includes:

- a stable controller protocol;
- immutable, validated controller parameters;
- a baseline cascaded position/velocity and pitch controller;
- explicit controller memory and reset behaviour;
- generalized-force limiting and anti-windup;
- reference/current-state validation;
- unit tests, closed-loop integration tests, and a small tracking demo.

The following are outside v0.5:

- path planning and obstacle avoidance;
- trajectory generation, interpolation, or `Trajectory.sample_at(t)`;
- direct heave actuation;
- thruster, fin, PWM/RPM, or actuator-allocation logic;
- vehicle-state estimation or filtering;
- adaptive, robust, sliding-mode, LQR, MPC, or learned control;
- ROS 2 and Gazebo interfaces;
- formal stability proof for the complete nonlinear closed loop;
- 6-DOF control.

## 2. Dependency and ownership contract

The allowed runtime dependency direction is:

```text
controller
├── core
└── numpy
```

Controller:

- MUST NOT import Physics, Environment, Simulation, Planner, ROS 2, or Gazebo;
- MUST NOT query current, obstacles, boundaries, or vehicle parameters;
- MUST NOT advance vehicle state or simulation time;
- MUST NOT select a reference from a `Trajectory`;
- MUST NOT convert generalized force into actuator commands;
- MUST return Core `ControlInput` values in the SNAME body frame.

Simulation owns orchestration:

```python
reference = reference_source.at_time(state.timestamp)
control = controller.step(state, reference, dt)
current = world.current_at(state.pose, state.timestamp)
next_state = physics.step(state, control, current, dt)
```

`reference_source.at_time(...)` is illustrative orchestration. Control v0.5
does not add interpolation or sampling methods to Core `Trajectory`.

## 3. Frame and signal contract

The controller uses the existing frozen conventions:

- current/reference positions: world ENU, with `x` East and `z` up;
- current/reference velocities: SNAME body frame, with `u` forward and `w`
  down;
- pitch and `q`: positive nose-up;
- output `tau_x`: positive body-forward force;
- output `tau_m`: positive nose-up pitch moment.

The baseline Physics selector is `diag(1,0,1)`. Therefore every output MUST
satisfy:

```text
tau_y = tau_z = tau_k = tau_n = 0.0
```

No output component may be silently repurposed as a thruster command.

## 4. Public interface

```python
from typing import Protocol

from core import ControlInput, VehicleState


class Controller3DOF(Protocol):
    def step(
        self,
        current: VehicleState,
        reference: VehicleState,
        dt: float,
    ) -> ControlInput:
        ...

    def reset(self) -> None:
        ...
```

`step()` MUST:

- require actual `VehicleState` inputs;
- validate all state values and timestamps as finite;
- reject non-zero inactive 3-DOF components using tolerance `1e-12`;
- require finite `dt > 0`;
- return a new `ControlInput` without mutating either state;
- produce only finite output values;
- update controller memory exactly once after a successful call;
- leave memory unchanged when validation fails.

The reference timestamp MAY differ from the current timestamp because
Simulation owns reference selection. Control v0.5 does not infer or alter
either timestamp.

## 5. Parameter and memory model

The baseline parameter schema is:

```python
@dataclass(frozen=True)
class CascadedPID3DOFParameters:
    position_to_surge_gain: float
    depth_to_pitch_gain: float
    maximum_surge_reference: float
    maximum_pitch_correction: float
    surge_kp: float
    surge_ki: float
    pitch_kp: float
    pitch_ki: float
    pitch_rate_kd: float
    maximum_tau_x: float
    maximum_tau_m: float
    surge_integral_limit: float
    pitch_integral_limit: float
```

All values MUST be finite and non-negative. Maximum reference, output, and
integral limits MUST be strictly positive. Gains MAY be zero so individual
terms can be isolated in tests.

Controller memory consists only of the two accumulated errors:

```python
surge_error_integral: float
pitch_error_integral: float
```

Memory MUST initialize to exact zero and `reset()` MUST restore exact zero.
It MUST NOT contain `VehicleState`, `Trajectory`, Environment, or Physics
objects.

## 6. Baseline control law

Let world position error be:

\[
e_x^n=x_d-x,\qquad e_z^n=z_d-z.
\]

The forward component of position error in the current body attitude is:

\[
e_{x_b}=\cos\theta\,e_x^n+\sin\theta\,e_z^n.
\]

### 6.1 Surge reference and error

\[
u_c=\operatorname{clip}
\left(u_d+k_xe_{x_b},-u_{max},u_{max}\right),
\qquad e_u=u_c-u.
\]

The unsaturated generalized surge force is:

\[
\tau_{x,raw}=K_{p,u}e_u+K_{i,u}I_u.
\]

### 6.2 Pitch reference and error

Depth correction is converted into a pitch correction consistent with the
commanded direction of travel. Define `s_u = +1` when `u_c >= 0` and `s_u =
-1` otherwise:

\[
\Delta\theta_c=\operatorname{clip}
\left(s_u k_z e_z^n,-\Delta\theta_{max},\Delta\theta_{max}\right),
\]

\[
\theta_c=\theta_d+\Delta\theta_c.
\]

The pitch error MUST use the shortest signed angular difference:

\[
e_\theta=\operatorname{wrap}_{[-\pi,\pi)}(\theta_c-\theta),
\qquad e_q=q_d-q.
\]

The unsaturated pitch moment is:

\[
\tau_{m,raw}=K_{p,\theta}e_\theta+K_{i,\theta}I_\theta
+K_{d,q}e_q.
\]

For forward motion, positive depth error produces a positive nose-up
correction. For commanded reverse motion the correction changes sign, so the
product `u*sin(pitch)` still drives world `z` toward the reference. The
`u_c == 0` tie uses the forward (`+1`) convention.

Reference `w` is not directly actuated and MUST NOT be treated as a heave-force
request. It may be retained for diagnostics but does not enter the v0.5 law.

### 6.3 Output limiting and anti-windup

Final commands are symmetric clips:

\[
\tau_x=\operatorname{clip}(\tau_{x,raw},-\tau_{x,max},\tau_{x,max}),
\]

\[
\tau_m=\operatorname{clip}(\tau_{m,raw},-\tau_{m,max},\tau_{m,max}).
\]

Each integral uses conditional integration. A candidate integral is computed
with `I_candidate = clip(I + error*dt, -I_max, I_max)`. It is accepted when the
corresponding raw output is unsaturated, or when the current error would drive
an already saturated output back toward the admissible range. Otherwise the
previous integral is retained. Output is then recomputed from the accepted
integral and clipped.

This is generalized-force limiting, not actuator allocation. The Controller
MUST NOT model thruster geometry or dynamics.

## 7. Reference contract

The reference is one complete Core `VehicleState`. For v0.5:

- active reference fields are `Pose.x`, `Pose.z`, `Pose.pitch`, `Twist.u`, and
  `Twist.q`;
- `Twist.w` is allowed but not directly controlled;
- `Pose.y`, `Pose.roll`, `Pose.yaw`, `Twist.v`, `Twist.p`, and `Twist.r` MUST be
  zero within tolerance;
- large reference discontinuities are accepted but will be constrained by
  reference and output limits;
- interpolation, look-ahead, end-of-trajectory behaviour, and time alignment
  belong to Simulation/Trajectory Generation.

## 8. Error handling and determinism

- Wrong semantic types raise `TypeError`.
- Invalid shapes, non-finite values, inactive components, invalid parameters,
  and invalid `dt` raise `ValueError`.
- Angles are not modified in Core objects; only the local pitch error is
  wrapped.
- Identical parameters, reset state, input/reference sequence, and `dt`
  produce identical output sequence on the same numerical platform.
- No logging, file I/O, random sampling, wall-clock access, or global mutable
  state is permitted in the controller package.

## 9. Intended package structure

```text
controller/
├── __init__.py
├── backend.py
├── parameters.py
├── angles.py
└── cascaded_pid_3dof.py

config/controllers/
└── cascaded_pid_3dof_baseline.yaml

tests/controller/
├── test_angles.py
├── test_parameters.py
├── test_cascaded_pid_3dof.py
└── test_controller_contract.py

tests/integration/
└── test_control_physics_integration.py

simulation/
└── control_tracking_demo.py
```

Files SHOULD be introduced incrementally. Configuration loading belongs at the
Simulation boundary so Controller remains independent of YAML.

## 10. Validation and acceptance criteria

Control v0.5 is complete only when:

- [x] pitch-error wrapping passes boundary cases around `+-pi`;
- [x] parameters reject negative, non-finite, boolean, and invalid-limit data;
- [x] zero tracking error produces zero output from reset state;
- [x] positive forward error produces positive `tau_x`;
- [x] an above-vehicle reference produces a positive nose-up `tau_m`;
- [x] pitch-rate feedback opposes pitch-rate error;
- [x] all inactive `ControlInput` fields are exactly zero;
- [x] output force and moment limits are enforced;
- [x] integral limits and conditional anti-windup are verified;
- [x] reset clears all controller memory;
- [x] invalid calls leave memory unchanged;
- [x] input/reference objects remain unchanged;
- [x] deterministic repeated sequences produce identical outputs;
- [x] Controller imports only Core and NumPy;
- [x] the underactuated output is accepted by `Python3DOFBackend`;
- [x] closed-loop surge and pitch regulation reduce their initial errors;
- [x] a vertical reference produces the expected pitch/depth response sign;
- [x] constant-current tracking remains finite and bounded for the documented
      smoke case;
- [x] all Core, Environment, and Physics regression tests remain green.

## 11. Explicit limitations

This baseline is a transparent research reference, not a claim of optimal or
robust UUV control. Position-to-pitch correction is a bounded heuristic outer
loop. Tracking performance depends on gains, the synthetic Physics parameters,
reference smoothness, current, and initial condition. Formal tuning,
stability-region analysis, actuator dynamics, and comparison with advanced
controllers require later validation documents and separate provenance.
