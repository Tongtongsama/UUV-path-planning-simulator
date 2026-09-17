# UUV Path Planning Simulator

A modular, research-oriented simulator for **Unmanned Underwater Vehicle
(UUV)** navigation, developed as part of an MSc dissertation at Nanyang
Technological University.

The project is intended as an extensible platform for studying **UUV
dynamics, configurable underwater environments, path planning, obstacle
avoidance, trajectory generation, and trajectory tracking** within a
common software architecture.

> **Current status:** Core v0.2, Environment v0.3, synthetic 3-DOF Physics
> v0.4, the underactuated Control baseline v0.5, Planner through v0.6b,
> Trajectory v0.7 and the Navigation v0.8 minimum closed loop are implemented and
> validated within their documented scopes. These milestones establish
> software and numerical contracts, not real-vehicle validation.

## Project Objectives

The simulator is being developed to:

-   provide a modular Python foundation for UUV navigation research;
-   support an initial nonlinear **3-DOF** vehicle model while keeping
    the architecture extensible toward **6-DOF** models;
-   represent configurable underwater environments, obstacles,
    boundaries, and environmental disturbances;
-   support multiple planning and control approaches without coupling
    the simulator to a single algorithm;
-   progress from unit and integration tests toward future ROS 2 and
    Gazebo validation.

The architecture deliberately separates **data models, environment
representation, physics, planning, control, simulation orchestration,
and external infrastructure**.

## Development Status

### v0.2 --- Core Data Model

  -----------------------------------------------------------------------
  Component               Role                    Status
  ----------------------- ----------------------- -----------------------
  `Pose`                  Position/orientation;   Complete
                          generalized             
                          configuration η         

  `Twist`                 Linear/angular          Complete
                          velocity; generalized   
                          velocity ν              

  `ControlInput`          Generalized forces and  Complete
                          moments τ               

  `VehicleState`          Time-stamped            Complete
                          `Pose + Twist`          

  `Path`                  Immutable geometric     Complete
                          sequence of `Pose`      
                          objects                 

  `Trajectory`            Immutable               Complete
                          time-parameterized      
                          sequence of             
                          `VehicleState` objects  

  Core unit tests         Independent Core        Complete
                          contract verification   

  Core integration smoke  Cross-object            Complete
  scenario                simulation-style        
                          validation              
  -----------------------------------------------------------------------

The Core interface is intentionally minimal. Interpolation, path
smoothing, resampling, actuator allocation, and simulation scheduling
are deferred to the modules that own those policies.

### v0.3 --- Environment / World Model

Environment v0.3 provides a lightweight planning world rather than a complete
ocean simulation. Its current implemented scope is:

-   an explicit Core ENU `(Pose.x, Pose.z)` to planning `[x,z]` mapping;
-   Shapely-backed circle, rectangle, and polygon static obstacles;
-   polygonal/rectangular operating boundaries and vehicle-clearance checks;
-   `NoCurrent` and two-dimensional `ConstantCurrent` fields;
-   stable `WorldModel` containment, collision, clearance, nearby-obstacle,
    and current queries;
-   validated Python/YAML scenarios as the single planning-world source;
-   unit tests and a deterministic Environment smoke scenario.

Shapely remains behind Environment domain interfaces, so Planner code does not
depend directly on the geometry backend. See
[`docs/environment_v0.3_specification.md`](docs/environment_v0.3_specification.md)
for the frozen query and coordinate semantics.

### v0.4 --- Physics Engine

Physics v0.4 is complete within its defined 3-DOF reference scope. It provides:

-   deterministic surge-heave-pitch dynamics in the vertical x-z plane;
-   frozen ENU world and SNAME body conventions, including
    `z_dot = u*sin(pitch) - w*cos(pitch)`;
-   world-current to body-current conversion and the rotating-current
    added-mass correction;
-   positive canonical added-mass and damping conventions;
-   diagonal validated rigid-body, added-mass, damping, and selector matrices;
-   underactuated baseline input selection `[tau_x, 0, tau_m]`;
-   immutable derivative values and a common Integrator protocol;
-   deterministic Euler and default RK4 fixed-step integration;
-   a Core-facing `Python3DOFBackend.step()` interface;
-   analytical, structural, convergence, integration, and smoke tests;
-   an offline SciPy `solve_ivp` comparison with reproducible result artifact.

The implemented configuration `uuv_3dof_synthetic_v1` is intentionally
synthetic and test-only. It must not be described as REMUS 100 or as a
validated physical vehicle.

Normative and evidential documents:

-   [`physics_v0.4_specification.md`](docs/physics_v0.4_specification.md)
-   [`physics_3dof_derivation.md`](docs/physics_3dof_derivation.md)
-   [`physics_v0.4_validation.md`](docs/physics_v0.4_validation.md)
-   [`open_source_reuse.md`](docs/open_source_reuse.md)

### v0.5 --- Controller

Control v0.5 provides a deterministic, PID-capable cascaded baseline for the
vertical-plane model:

-   a stable `Controller3DOF.step(current, reference, dt)` protocol;
-   validated immutable gains and explicit resettable controller memory;
-   world-position to body-surge and depth-to-pitch outer loops;
-   surge force and pitch moment feedback with conditional anti-windup;
-   exact-zero inactive forces and moments compatible with the Physics
    selector `diag(1,0,1)`;
-   a Simulation-owned YAML configuration loader;
-   unit, Controller/Physics integration, deterministic current-disturbance,
    and closed-loop tracking tests;
-   a reproducible 60-second no-current/constant-current tracking demo.

The baseline emits generalized forces, not thruster or fin commands. See
[`control_v0.5_specification.md`](docs/control_v0.5_specification.md) and
[`control_v0.5_validation.md`](docs/control_v0.5_validation.md).

## Core Data Model

``` text
η = [x, y, z, roll, pitch, yaw]^T    -> Pose
ν = [u, v, w, p, q, r]^T            -> Twist
x = [η; ν]                           -> VehicleState
τ = [X, Y, Z, K, M, N]^T            -> ControlInput

Pose sequence                         -> Path
Time-parameterized VehicleState seq. -> Trajectory
```

`VehicleState.to_numpy()` returns the **12-dimensional physical state
vector** `[η; ν]`. The timestamp is simulation metadata and is
deliberately excluded.

### Coordinate and Frame Conventions

-   **Pose:** ENU inertial/world frame --- x East, y North, z Up.
-   **Twist:** SNAME body-fixed marine frame --- u surge/forward, v
    sway/starboard, w heave/down, with p/q/r angular rates.
-   **ControlInput:** same SNAME body-fixed convention as `Twist`.
-   Frame transformations belong to the **Physics Engine**, not Core.

### Core Design Rules

-   Core classes are immutable frozen dataclasses.
-   Pose, Twist, and ControlInput retain complete six-component
    representations.
-   Active DOFs and direct actuation are Physics/vehicle-configuration
    concerns.
-   Underactuation and model dimensionality are separate concerns.
-   `VehicleState` composes `Pose` and `Twist` rather than duplicating
    their fields.
-   `Path` is geometric and carries no timestamps.
-   `Trajectory` uses `VehicleState.timestamp` as its single source of
    time information.
-   `Path` and `Trajectory` use tuples to preserve sequence
    immutability.
-   NumPy conversion interfaces use explicit vector-shape contracts.

## Intended Data Flow

``` text
                    Planner
                       |
                       v
                     Path
                       |
                       v
              Trajectory Generation
                       |
                       v
                  Trajectory
                       |
                       v
VehicleState ----> Controller
                       |
                       v
                 ControlInput
                       |
                       v
                 Physics Engine
                       |
                       v
               New VehicleState
```

The production Simulation Manager will orchestrate these modules after
the Environment and Physics interfaces are stable.

## Project Structure

``` text
uuv_simulator/
├── config/
│   ├── controllers/
│   │   └── cascaded_pid_3dof_baseline.yaml
│   ├── scenarios/
│   │   └── simple_static_world.yaml
│   └── vehicles/
│       └── uuv_3dof_synthetic_v1.yaml
├── controller/
│   ├── backend.py
│   ├── angles.py
│   ├── parameters.py
│   └── cascaded_pid_3dof.py
├── core/
│   ├── __init__.py
│   ├── pose.py
│   ├── twist.py
│   ├── control_input.py
│   ├── vehicle_state.py
│   ├── path.py
│   └── trajectory.py
├── docs/
├── environment/
│   ├── coordinates.py
│   ├── obstacle.py
│   ├── boundary.py
│   ├── current.py
│   ├── world.py
│   └── scenario.py
├── physics/
│   ├── backend.py
│   ├── derivative.py
│   ├── frames.py
│   ├── integrators.py
│   ├── parameters.py
│   └── uuv_3dof.py
├── planner/
├── simulation/
│   ├── core_integration_demo.py
│   ├── environment_smoke_demo.py
│   ├── physics_smoke_demo.py
│   ├── control_tracking_demo.py
│   ├── controller_config.py
│   └── vehicle_config.py
├── tests/
│   ├── core/
│   ├── environment/
│   ├── physics/
│   ├── controller/
│   └── integration/
├── validation/
│   ├── compare_fossen_model.py
│   └── reference_cases/
├── visualization/
├── ros2/
└── main.py
```

Some upper-layer packages are currently architectural placeholders and
will be implemented incrementally.

## Testing

### Optional Control v0.5.1 candidate

Restoring-moment feedforward is now implemented with default coefficient zero,
combined-request clipping and limits-aware conditional integration. Navigation
supplies its caps before the controller updates integrals. Old YAML and the old
step/reset API remain supported; the coefficient-6 candidate is explicitly
selected from `config/controllers/cascaded_pid_3dof_restoring_candidate.yaml`,
not installed as the global default.

Formal acceptance: **622 passed**, 17 repeated real-module cases. Horizontal,
ascending and descending candidate runs pass at 0.2 and 0.5 m/s. Limited ±20%
coefficient mismatch cases pass but do not establish general robustness.
See [Control v0.5.1 acceptance](docs/control_v0.5.1_validation.md) and
[low-speed terminal residual classification](docs/terminal_residual_classification_v051.md).
Low-speed detour still times out with near-static cross-track residual; no new
terminal strategy, corner slowdown or smoothing has been enabled.

```powershell
.\.venv\Scripts\python.exe -m validation.control_v051_acceptance --output artifacts/controller/v051_run_002
```

### Test environment

Use a project-local virtual environment so global pytest installations do not
override the declared `pytest>=7.0,<9.0` development range. On Windows PowerShell:

```powershell
python -m venv .venv
.\.venv\Scripts\python.exe -m pip install -r requirements-validation.txt -r requirements-visualization.txt
.\.venv\Scripts\python.exe -m pip check
.\.venv\Scripts\python.exe -m pytest tests/ -q -rs
```

The combined install includes development tests, optional SciPy reference tests
and plotting tests. For only the normal development dependencies, install
`requirements-dev.txt` instead. `.venv/` is already Git-ignored. Select
`.venv\Scripts\python.exe` as the IDE interpreter. The explicit executable
does not require activating PowerShell scripts or changing execution policy.
Use it for validation/simulation runners too; bare `python` or `pytest` in an
unactivated terminal may still invoke a different global installation.

Verified on 2026-09-13 in the isolated Python 3.10.9 environment: pytest 8.4.2,
`pip check` clean, **589 passed with no skips**, including SciPy 1.15.3 reference
validation. See [dependency verification](docs/dependency_environment_validation.md).

The generic commands below assume the intended virtual environment is active.

Install normal runtime dependencies:

``` bash
python -m pip install -r requirements.txt
```

For development and the normal test suite:

``` bash
python -m pip install -r requirements-dev.txt
```

SciPy is an offline validation dependency, not a Physics runtime dependency:

``` bash
python -m pip install -r requirements-validation.txt
```

Run the complete test suite from the repository root:

``` bash
python -m pytest tests/ -v
```

The verification strategy is progressive:

1.  **Unit tests** --- verify each class or subsystem independently.
2.  **Integration smoke tests** --- verify interfaces and data flow
    between completed components.
3.  **Subsystem integration tests** --- validate Environment, Physics,
    Planner, and Controller interactions.
4.  **ROS 2 / Gazebo validation** --- future external integration and
    progressively higher-fidelity evaluation.

The repository includes deterministic Core, Environment, Physics, and Control
scenarios. The Control demo closes the loop through the real Controller and
Physics backends in no-current and constant-current cases.

## Development Roadmap

``` text
v0.1  Project Skeleton
  |
v0.2  Core Data Model                 COMPLETE
  |
v0.3  Environment / World Model      COMPLETE
  |
v0.4  Physics Engine                 COMPLETE (synthetic 3-DOF reference scope)
  |
v0.5  Controller                     COMPLETE (synthetic 3-DOF baseline scope)
  |
v0.6a Planner contracts and validation COMPLETE
  |
v0.6b A* baseline                     COMPLETE
  |
M1    A* acceptance and static figures IMPLEMENTED / evidence generated
  |
v0.7  Trajectory generation           COMPLETE (M2 baseline verified)
  |
v0.8  Navigation integration          BASELINE VERIFIED; hard-case failures recorded
  |
Next  Benchmark, then evidence-driven planning improvements and ROS 2
  |
Gated RRT/RRT*, MPC, 6-DOF and Gazebo extensions
  |
v1.0  Dissertation Release
```

### Completed Milestone: Physics Engine v0.4

Physics v0.4 now has a deterministic Python 3-DOF backend, validated
ENU/SNAME transformations, canonical synthetic parameters, and Euler/RK4
integrators. Its normative contract and approved vertical-plane reduction are
recorded in `docs/physics_v0.4_specification.md` and
`docs/physics_3dof_derivation.md`.

The full Simulation Manager remains deferred to a later orchestration phase;
`simulation/physics_smoke_demo.py` exercises the stable `step()` boundary.

### Future higher-DOF extension

The 3-DOF backend should remain available as a small reference model. A future
6-DOF backend should be introduced as a separate implementation behind
`PhysicsBackend`, with a new specification and parameter configuration. It
will require:

-   a full state mapping for `[x,y,z,roll,pitch,yaw,u,v,w,p,q,r]` and a stated
    Euler-angle singularity policy or quaternion-based internal attitude;
-   audited ENU/SNAME 6-DOF kinematics and world/body transforms;
-   full symmetric 6-by-6 rigid-body and added-mass matrices, including body
    origin, CG/CB offsets, products of inertia, and permitted coupling terms;
-   independently derived rigid-body and added-mass Coriolis matrices with
    skew-symmetry and energy tests;
-   physical weight, buoyancy, CG, and CB parameters replacing the reduced
    pitch-restoring coefficient;
-   three-dimensional world current, angular-current assumptions, and the
    corresponding body-frame current derivative;
-   6-DOF damping and cross-flow/lift models with explicit sign and parameter
    provenance;
-   a separate actuator/thruster/fin model and allocation matrix instead of
    expanding the current selector into hidden allocation logic;
-   3-D Environment boundaries, obstacle geometry, vehicle footprint, and
    collision queries if the higher-DOF simulation uses spatial interaction;
-   dimension-aware derivative/integrator abstractions or a parallel 6-DOF
    derivative type without weakening the current 3-DOF shape guarantees;
-   versioned real or synthetic vehicle configurations with units, frames,
    source commits, conversions, and confidence recorded;
-   analytical limiting cases, conservation/passivity checks, convergence and
    sensitivity studies, and comparison against an independent model or the
    future Gazebo Harmonic backend.

This is a coordinated Core/Environment/Physics/Simulation migration, not a
change that should be made by simply allowing larger arrays in the current
3-DOF classes.

### Completed Milestone: Control v0.5

Control v0.5 implements the approved underactuated surge/pitch cascaded PID
baseline. It consumes current and reference `VehicleState` objects and emits a
bounded, Physics-admissible `ControlInput`. Trajectory sampling remains a
Simulation responsibility, while actuator allocation and thruster/fin
dynamics remain deferred.

### Completed Milestone: Planner v0.6a

Planner v0.6a provides validated PlanningConstraints, PlanningRequest,
PlanningResult/PlanningStatus, GlobalPlanner, PlanningSpace and a WorldModel
adapter. Shared validation checks complete segments, footprint clearance,
optional request endpoints and path length in the ENU x-z vertical plane.
Its output is the existing geometric Core Path. A straight-line planner exists
only in tests to verify the integration boundary.

See [Planner v0.6a specification](docs/planner_v0.6a_specification.md).
Run `pytest tests/planner -q` for its contract and integration checks.

### Completed Milestone: Planner v0.6b

The A* baseline adds a bounded ENU x-z occupancy lattice, world/index
conversion, deterministic 4/8-connected search, open/closed sets, predecessor
reconstruction and endpoint connectors. Search accepts valid lattice nodes
inside goal_tolerance, returns `[start]` if already arrived, and uses distance
to the goal region for its heuristic and path-length pruning. Exact-goal
connectors remain available for off-grid goals. Every edge and returned Path
undergoes continuous collision validation. Occupancy applies vehicle radius
and safety margin; diagonal corner cutting is rejected. Failure results
distinguish invalid endpoints, no route in the configured graph and exhausted
resource budgets.

See [Planner v0.6b specification](docs/planner_v0.6b_specification.md).
Run `python -m simulation.planner_smoke_demo` for detour and unreachable cases.
This version adds no dependencies. Verified on 2026-09-09: 522 tests passed,
1 optional SciPy validation test skipped.

The delivery sequence now prioritizes M1 A* acceptance and static figures,
Trajectory v0.7, Navigation v0.8 and baseline benchmark evidence. Planning
improvements follow the benchmark; RRT/RRT* remains a conditional extension.

### M1 acceptance runner and static figures

Install the optional plotting dependencies and create a new run directory:

```bash
python -m pip install -r requirements-visualization.txt
python -m validation.planner_acceptance --output artifacts/planner/m1_run_001
```

The runner reads `config/scenarios/planner_acceptance_v1.json`, runs detour,
narrow-passage and unreachable cases twice, revalidates complete paths and
runs the full test suite. It saves PNG/SVG figures, JSON results/configuration,
test output, source snapshots and SHA-256 manifests. Existing run directories
are never overwritten. `--skip-tests` is a preview, not acceptance evidence.

The 2026-09-12 run reports **548 passed, 1 skipped** (optional SciPy), and all
three expected outcomes. All three PNGs were visually inspected. See
[M1 acceptance guide](docs/planner_m1_acceptance.md) for artifacts, limitations
and reproduction. That historical run used pytest 9.1.1, outside the declared
<9 range. A later isolated pytest 8.4.2 full-suite verification now passes;
see [dependency verification](docs/dependency_environment_validation.md).
That M1 evidence alone does not certify Navigation or a release-ready checkout.

### Trajectory v0.7 baseline

`trajectory.generate_trajectory` validates an accepted Path, removes consecutive
duplicate positions, performs collision-safe line-of-sight shortcutting,
resamples by bounded distance spacing and assigns constant-speed timestamps.
Every geometry stage is revalidated. The result includes intermediate paths
and the existing Core Trajectory. `sample_reference` provides the explicit
piecewise-linear reference policy for Simulation, including terminal holding.

This baseline has discontinuous pitch at corners and no acceleration or
finite-turn-rate guarantee. It does not establish underactuated dynamic
feasibility. Such failures must be measured in Navigation v0.8 before deciding
which smoothing or speed-policy extension to adopt.

Both v1 and user-added v2 planning scenarios are covered. Verified on 2026-09-12:
the dedicated trajectory log records 14 passed; M2's full log records
566 passed, one optional SciPy test skipped. Six successful scenarios
produce validated timed references; the unreachable scenario produces none.

M2 evidence is archived in `artifacts/trajectory/m2_acceptance_20260912`, with
config, per-case invariant checks, summary, separate pytest logs, dedicated
test source and a verified manifest. Generate a new evidence package using
`python -m validation.trajectory_acceptance --output artifacts/trajectory/m2_run_002`.
This runner records source hashes without copying another full source tree.

```bash
python -m simulation.trajectory_demo --output artifacts/trajectory/v07_run_001
```

See [Trajectory specification](docs/trajectory_v0.7_specification.md).
The demo writes raw/shortcut/sample comparisons, pitch/speed curves and JSON
states. Install requirements-visualization.txt for plotting.

### Navigation v0.8 / M3 closed-loop baseline

`Navigator` composes A*, trajectory generation/sampling, the existing PID and
Python 3-DOF Physics. It uses a fixed clock, records commanded/applied controls,
checks actual motion segments against obstacles and boundaries, and requires
position, speed and pitch-rate qualification for a settling interval.

```bash
python -m validation.navigation_acceptance --output artifacts/navigation/m3_run_001 --animate
```

Install `requirements-visualization.txt` for this runner. Optional `--stage`
values are `minimum`, `static`, `current` and `all` (default). Each case is run
twice; histories, parameters, metrics, PNG/SVG, pytest log and SHA-256 manifest
are saved without copying another source tree. `artifacts/` remains Git-ignored.

Re-audited on 2026-09-13: **588 passed, 1 skipped** (optional SciPy missing).
The 16 experiments produced **7 SUCCESS, 2 TIMEOUT, 6 COLLISION and 1
OUT_OF_BOUNDS**. Horizontal and static narrow-passage baselines passed, including
axial constant-current comparisons. Diagonal settling, detours and vertical
current still expose limitations. Evidence acceptance is not mission success.
That recorded M3 runner used global pytest 9.1.1. A subsequent isolated
pytest 8.4.2 full-suite run passes all 589 tests, including SciPy validation.
The old M3 artifact package is retained unchanged; this dependency check does
not replace or rerun its 16 scenario experiments.

See [Navigation contract](docs/navigation_v0.8_specification.md) and
[M3 audit and results](docs/navigation_v0.8_validation.md). The next research
step is diagnosing diagonal terminal holding and detour tracking failures,
not treating all obstacle scenarios as solved.

The [tracking diagnosis](docs/navigation_tracking_diagnosis.md) now records
30 repeated diagnostic cases, including the 24-case isolated corner sweep.
It identifies non-goal terminal equilibria, limited pitch integral authority
against restoring moment, and detour collision before the first reference
corner. Slower corner runs reduce measured moving-phase deviation but do not
establish reliable completion. No PID tuning or corner-speed policy has been
applied. Reproduce with the project virtual environment:

```powershell
.\.venv\Scripts\python.exe -m validation.tracking_diagnostics --output artifacts/navigation/tracking_diagnosis_run_002
```

Follow-up [restoring/terminal ablations](docs/terminal_restoring_experiments.md)
compare 16 navigation cases and 18 attitude-hold cases. Exact synthetic restoring
feedforward enables the two diagonal missions with unchanged tolerances, but
detour still collides. The tested terminal-guidance candidate degrades the
horizontal case and is rejected. These behaviors remain opt-in validation
experiments; production controller/trajectory defaults are unchanged.

The [optional feedforward and anti-windup design](docs/control_feedforward_antiwindup_design.md)
now specifies combined pre-limit requests and shared effective actuator caps,
with a validation-only executable prototype. The
[detour execution-margin study](docs/detour_execution_margin_study.md) separates
initial alignment, uniform speed and planning-only extra margin while keeping
actual-motion clearance fixed at 0.3 m. Several variants avoid collision but
still time out or leave the boundary; none is promoted to a successful default.

### Deferred Environment extensions

The following are intentional future options, **not currently implemented**:

-   `SinusoidalCurrent`, with separately configurable temporal periodicity
    (for tidal behaviour) and spatial periodicity (for wave-like or repeating
    flow structure), phase, amplitude, mean flow, and documented units;
-   analytical flow models such as a Lamb-Oseen vortex and, where justified by
    experiments, wave-spectrum-derived disturbance fields;
-   adapters for real ocean-current products such as HYCOM or Copernicus
    Marine data, including coordinate/time/depth conversion, interpolation,
    caching, dataset provenance, and reproducible offline experiment inputs;
-   replacement of the current vertical 2-D geometry backend with 3-D obstacle
    and boundary collision queries;
-   three-dimensional current vectors and environmental physics fields;
-   coordinated Core/Environment/Physics state and frame upgrades required for
    future 6-DOF simulation.

These features should be added behind the existing domain boundaries when an
experiment requires them. Dataset adapters and analytical current fields
should implement `CurrentField`; a 3-D migration should introduce an explicit
new coordinate contract rather than silently changing the meaning of `[x,z]`.

## Planned Research Capabilities

The architecture is intended to support later investigation of nonlinear
UUV dynamics, underactuated motion, obstacle-rich environments,
water-current disturbances, global/local path planning, trajectory
generation and tracking, baseline PID control, possible MPC extensions,
comparative experiments, ROS 2/Gazebo integration, and optional
extension toward higher-dimensional dynamics.

These are **development directions**, not completed simulator
functionality unless explicitly marked complete above.

## Software Architecture Principles

Lower-level modules must not depend on higher-level orchestration or
research algorithms.

``` text
simulation
    |
    +--> navigation --> planner / trajectory / controller / physics / environment
    +--> visualization
    +--> physics
    +--> environment
    +--> core
```

-   `core` remains independent of Environment, Physics, Planner,
    Controller, ROS 2, and Gazebo.
-   `environment` may depend on Core but does not own vehicle dynamics
    or control.
-   `physics` depends only on Core and NumPy; Simulation supplies numerical
    current values and loads configuration without Physics importing
    Environment, Planner, Controller, ROS 2, or Gazebo.
-   `planner` may use Core and Environment representations.
-   `controller` consumes Core state/reference objects and produces
    `ControlInput`.
-   `simulation` orchestrates modules rather than implementing their
    algorithms.
-   `navigation` owns mission scheduling, actual-motion safety, termination
    and recording; lower-level modules do not import it.
-   ROS 2 and Gazebo are integration infrastructure rather than the
    location of core research logic.

## Technology Stack

### Optional terminal capture (verified candidate)

Navigation now supports an opt-in `TerminalCapturePolicy` with an explicit controller
guidance interface, bounded braking/correction and unchanged safety/arrival checks.
The full suite passes 637 tests; 13 paired fixtures (26 enabled/disabled cases) have
reproducible acceptance evidence. Low-speed detours succeed with capture, but retain
only approximately 27 mm / 8 mm obstacle margin. Some previously successful tasks
take longer, so terminal hold remains the default.
See [contract, results and reproduction command](docs/terminal_capture_policy.md).

An additional opt-in controlled-startup candidate physically establishes initial
heading while pausing only the path clock. Separate experiments compare local
corner speed scheduling, without smoothing or PID tuning. These are not new defaults;
see [startup and corner-timing evidence](docs/controlled_startup_and_corner_timing.md).

Current development:

-   Python 3.10+
-   NumPy
-   PyYAML
-   Shapely 2.x
-   pytest (development)
-   SciPy (offline validation only)
-   Git / GitHub

Planned integration environment:

-   Ubuntu 24.04
-   ROS 2 Jazzy
-   Gazebo

ROS 2 and Gazebo integration are not yet completed simulator features.

## Dissertation Context

**Project:** *Simulator Development for UUV Path Planning with Obstacle
Avoidance and Trajectory Tracking*\
**Institution:** Nanyang Technological University, Singapore\
**Programme:** MSc Computer Control and Automation\
**Project ID:** ISM-DISS-05863

The simulator is being developed as a reusable experimental platform
rather than an implementation tied to one planning or control algorithm.

## License

See the repository `License` / `LICENSE` file for licensing information.
