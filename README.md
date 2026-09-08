# UUV Path Planning Simulator

A modular, research-oriented simulator for **Unmanned Underwater Vehicle
(UUV)** navigation, developed as part of an MSc dissertation at Nanyang
Technological University.

The project is intended as an extensible platform for studying **UUV
dynamics, configurable underwater environments, path planning, obstacle
avoidance, trajectory generation, and trajectory tracking** within a
common software architecture.

> **Current status:** Core v0.2, Environment v0.3, synthetic 3-DOF Physics
> v0.4, the underactuated Control baseline v0.5, and Planner v0.6a are implemented and
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
v0.6b A* baseline
  |
v0.6c Constraint/current-aware planning improvements
  |
v0.6d RRT/RRT* comparison
  |
v0.7  Trajectory generation
  |
v0.8  Navigation integration
  |
Later Visualization, benchmarks, ROS 2 / Gazebo integration
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

The next milestone is v0.6b standard A*. Subsequent milestones are v0.6c
constraint/current-aware improvements, v0.6d RRT/RRT*, Trajectory v0.7
(simplification, smoothing and time parameterization), and Navigation v0.8
(Planner–Trajectory–Controller–Physics integration). Search and trajectory
generation are not part of the completed v0.6a scope. Visualization, algorithm
benchmarks and ROS/Gazebo integration remain future work.

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
    +--> planner / controller / visualization
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
-   ROS 2 and Gazebo are integration infrastructure rather than the
    location of core research logic.

## Technology Stack

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
