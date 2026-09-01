# UUV Path Planning Simulator

A modular, research-oriented simulator for **Unmanned Underwater Vehicle
(UUV)** navigation, developed as part of an MSc dissertation at Nanyang
Technological University.

The project is intended as an extensible platform for studying **UUV
dynamics, configurable underwater environments, path planning, obstacle
avoidance, trajectory generation, and trajectory tracking** within a
common software architecture.

> **Current status:** Core Data Model v0.2 is implemented and
> unit-tested. The immediate next task is a Core integration smoke
> validation; the next formal subsystem is Environment / World Model
> v0.3.

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

  Core integration smoke  Cross-object            Next validation step
  scenario                simulation-style        
                          validation              
  -----------------------------------------------------------------------

The Core interface is intentionally minimal. Interpolation, path
smoothing, resampling, actuator allocation, and simulation scheduling
are deferred to the modules that own those policies.

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
├── controller/
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
├── physics/
├── planner/
├── simulation/
├── tests/
│   └── core/
│       ├── test_pose.py
│       ├── test_twist.py
│       ├── test_control_input.py
│       ├── test_vehicle_state.py
│       ├── test_path.py
│       └── test_trajectory.py
├── visualization/
├── ros2/
└── main.py
```

Some upper-layer packages are currently architectural placeholders and
will be implemented incrementally.

## Testing

Run the complete Core test suite from the repository root:

``` bash
python -m pytest tests/core/ -v
```

The verification strategy is progressive:

1.  **Unit tests** --- verify each class or subsystem independently.
2.  **Integration smoke tests** --- verify interfaces and data flow
    between completed components.
3.  **Subsystem integration tests** --- validate Environment, Physics,
    Planner, and Controller interactions.
4.  **ROS 2 / Gazebo validation** --- future external integration and
    progressively higher-fidelity evaluation.

The immediate next validation task is a deterministic Core integration
scenario using mock/stub Planner-, Controller-, and Physics-like
operations. It validates Core interfaces rather than hydrodynamic or
control performance.

## Development Roadmap

``` text
v0.1  Project Skeleton
  |
v0.2  Core Data Model                 COMPLETE
  |
  +-- Core integration smoke validation
  |
v0.3  Environment / World Model      NEXT
  |
v0.4  Physics Engine
  |
v0.5  Visualization
  |
v0.6  Planner
  |
v0.7  Controller
  |
v0.8  ROS 2 Integration
  |
v0.9  Gazebo Integration
  |
v1.0  Dissertation Release
```

### Next Formal Milestone: Environment / World Model v0.3

Initial scope:

-   world/environment boundaries;
-   geometric obstacle representations and basic queries;
-   a minimal ocean-current interface, beginning with a constant-current
    model;
-   an Environment/World container;
-   unit tests and an environment-level integration scenario.

The full Simulation Manager is intentionally deferred until Environment
and Physics expose stable interfaces.

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
-   `physics` may depend on Core and environmental/configuration
    interfaces, but not Planner or Controller.
-   `planner` may use Core and Environment representations.
-   `controller` consumes Core state/reference objects and produces
    `ControlInput`.
-   `simulation` orchestrates modules rather than implementing their
    algorithms.
-   ROS 2 and Gazebo are integration infrastructure rather than the
    location of core research logic.

## Technology Stack

Current development:

-   Python
-   NumPy
-   pytest
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
