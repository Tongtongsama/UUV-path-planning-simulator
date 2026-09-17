# Trajectory v0.7 baseline

Status: M2 completed and verified within baseline scope. This milestone does
not certify vehicle-dynamic feasibility or completion of Navigation v0.8.

Verification on 2026-09-12: 14 new tests; full suite 562 passed, 1 skipped
(optional SciPy unavailable). Six successful v1/v2 scenario figures inspected
in artifacts/trajectory/v07_20260912; the seventh scenario is unreachable.

M2 evidence verification on 2026-09-12 supersedes the earlier unarchived suite
claim: the dedicated original test file reports 14 passed, and the full suite
reports 566 passed, 1 skipped after adding four acceptance-checker regressions.
The archived files are in artifacts/trajectory/m2_acceptance_20260912:
config.json, results.json, summary.txt, pytest_trajectory.log, pytest_full.log,
test_trajectory_generation.py and manifest.json. The manifest was verified.
All six generated cases pass every machine-readable invariant; the unreachable
case has no trajectory. Historical Planner logs do not constitute M2 evidence.

Reproduce in a new directory:

```bash
python -m validation.trajectory_acceptance --output artifacts/trajectory/m2_run_002
```

The runner archives actual interpreter/package versions, Git state and source
hashes and exits nonzero on scenario/check/test failure. It does not copy a
full source tree. Source hashes identify the tested files but do not replace
a committed checkout. No repeated source snapshots are needed for this runner.
Spacing and timing use explicit 1e-10 tolerances (recorded in config.json).
Singleton spacing/time monotonicity checks are vacuous by definition; its
duration is zero and its terminal Twist must still be entirely zero.

## Input and ownership

generate_trajectory(Path, PlanningSpace, PlanningConstraints, TrajectoryConfig,
start_time=0, request=None) returns TrajectoryGenerationResult. It includes
raw, deduplicated, shortcut and sampled paths and a Core Trajectory. Core,
Planner and Controller interfaces are unchanged. This package consumes Planner
validation/space contracts and Core values; it does not import Physics,
Controller, YAML, ROS or plotting. Simulation owns scheduling.

Input and every output geometry stage are checked with validate_path. Supplying
request enables original start and goal-region validation. The actual accepted
Path endpoint is retained geometrically; no exact-goal segment is invented.
Malformed data and invalid geometry raise TypeError/ValueError. No partial
trajectory is returned. Configuration is frozen: positive finite nominal_speed
(default 0.5 m/s), positive finite sample_spacing (default 0.25 m), boolean
shortcut (default True), positive integer max_samples (default 100000).

## Geometry

Only exact consecutive duplicate x-z positions are removed; the first pose
is retained. Attitude-only changes are not motion commands in this geometric
baseline. Greedy shortcut selects the farthest visible subsequent waypoint
using complete swept-footprint collision checks and original clearance.
Each remaining segment is split into ceil(length/sample_spacing) equal pieces.
All corners remain exact samples: resampling never joins points across a corner
and thereby cuts through obstacles. This is bounded spacing, not a globally
uniform spacing guarantee. Output growth and nonrepresentable tiny steps are
rejected before creating invalid output.

## Coordinate and time contract

World is ENU x-z. On each outgoing straight segment:

```
pitch = atan2(delta_z, delta_x)
u = nominal_speed
w = q = 0
x_dot = u*cos(pitch)
z_dot = u*sin(pitch)
delta_t = segment_length/nominal_speed
```

Inactive pose/twist fields are zero. Reference u is desired body-frame surge
speed. No current compensation is performed by the trajectory generator.
Here body-frame specifies the coordinate basis, not velocity relative to water:
Physics v0.4 stores absolute translational velocity in body coordinates and
computes water-relative velocity separately as nu_relative = nu - current_body.
With tangent pitch and w=0, nominal_speed also equals the reference position
curve's speed through world coordinates. Actual ground speed and tracking error
under current are evaluated by Controller/Navigation. No current is added to
or subtracted from the generated reference.
Pure upward/downward or negative-x geometric motion follows the same atan2
rule. Feasibility of those attitudes for the controlled vehicle is not assumed.
Input active velocities and endpoint pitch are not preserved as constraints.

At an internal corner the outgoing segment sets pitch and velocity. Pitch may
jump; q=0 describes each open straight segment and is not the derivative at
the corner. No finite pitch-rate/acceleration guarantee is claimed. Smoothing,
turn slowdown, bounded angular-rate transitions and optimal timing remain
extensions motivated by closed-loop evidence. These limitations must remain
visible in plots and later Navigation evaluation.

Timestamps are finite and strictly increasing. Unrepresentable positive time
increments are rejected. The final reference has zero Twist and holds the last
segment attitude. A singleton (including a path of duplicate positions) returns
one stationary VehicleState at start_time, preserving its active pitch.

## Sampling outside Core

sample_reference(trajectory, time) is the baseline sampling policy for generated
trajectories. Simulation chooses the call time. It linearly interpolates x-z
inside one segment and holds that segment's outgoing pitch and Twist. At a knot
the outgoing state is selected. Before the first timestamp it holds the initial
pose with zero Twist; at/after the final timestamp it holds the final pose with
zero Twist. Returned timestamp equals query time. It must not be used as a
general interpolator for independently supplied smooth/6-DOF trajectories.

## Validation and demonstration

Tests cover analytical timing and ENU signs in four directions, reference
kinematic consistency, sample spacing, singleton/duplicates, corner sampling,
terminal holding, malformed parameters, resource/time limits, and continuous
post-processing validation on both planner acceptance v1 and the user's v2.
Unreachable planning outcomes do not generate trajectories.

```bash
pytest tests/integration/test_trajectory_generation.py -q
python -m simulation.trajectory_demo --output artifacts/trajectory/v07_run_001
```

The demo requires requirements-visualization.txt and a new output directory.
It records inputs, intermediate paths and complete timed states in JSON, plus
raw/shortcut/sample and speed/pitch figures. No closed-loop navigation result
is implied. The next acceptance step is reference tracking through Simulation,
with collision, timeout and goal/terminal-motion policies in Navigation v0.8.
