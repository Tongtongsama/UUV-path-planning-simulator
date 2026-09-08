# Planner v0.6a: public architecture and geometric validation

Status: implemented. Output boundary: PlanningRequest + PlanningSpace ->
PlanningResult -> Core Path or None.

## Scope and coordinates

The planning plane is world ENU x-z, consistent with Physics v0.4's
surge/heave/pitch state. Pose.y/roll/yaw and start Twist.v/p/r must be zero
within 1e-12. Active values and timestamps must be finite real numbers.
Start velocity and pitch are retained for future planners, but this milestone
does not certify motion feasibility or enforce goal orientation.

Core Path is the existing non-generic sequence of Pose objects. Construct it
with Path.from_list or a tuple; no Core type changes are required. It contains
no timestamps. No search algorithm, trajectory generation, smoothing, current
cost, or closed-loop navigation is implemented here.

## Public contracts

- PlanningConstraints: nonnegative finite vehicle_radius, safety_margin,
  goal_tolerance, optional max_path_length, all in metres. Required clearance
  is vehicle_radius + safety_margin and must also be finite. Zero is valid.
- PlanningRequest: frozen start VehicleState, goal Pose and constraints.
  Environment feasibility is checked separately from construction.
- PlanningResult: SUCCESS requires a nonempty Path; every failure requires
  path=None. Time is seconds; cost is algorithm-defined, finite and
  nonnegative. Expanded nodes is a nonnegative integer, not a boolean.
- GlobalPlanner: abstract name and plan(request, space). Planner is an alias
  for the same abstraction. Algorithms own their execution flow.
- PlanningSpace: contains, clearance, is_pose_valid, is_segment_valid.
  EnvironmentPlanningSpace adapts the actual WorldModel public interface.

Invalid API data raises TypeError or ValueError. Normal planning failure uses
PlanningStatus. validate_request returns INVALID_START before INVALID_GOAL
if both are infeasible; otherwise it returns None. It does not claim a route
exists. Result construction cannot establish map-dependent validity; planners
must validate their candidate path before reporting SUCCESS.

## Collision semantics

The footprint is an orientation-independent disk of required_clearance.
Obstacle contact is collision, including distance exactly equal to clearance.
Boundary contact is allowed when the complete footprint remains covered.
clearance(pose) means distance to the nearest obstacle only; an obstacle-free
world returns infinity. Boundary clearance is included in validity checks.

Environment supplies Obstacle.segment_collides, Boundary.contains_segment,
and WorldModel.is_segment_valid. These query complete line geometry, not
discrete samples. They detect thin obstacles and excursions from concave
boundaries. Zero-length segments use the same footprint interpretation.
Distances are relative to the existing Shapely geometry, including polygonal
approximations of circular obstacles; no new circle accuracy claim is made.
Planner code does not import Shapely or access Environment private geometry.

## Path validation

validate_path(path, space, constraints, *, request=None) returns a frozen
PathValidationResult with the first invalid waypoint or segment index and
a diagnostic message. Segment i joins waypoints i and i+1.

Validation checks nonempty paths, valid waypoints, every complete segment,
optional endpoint matching, then maximum length. Core normally rejects empty
paths during construction. Invalid plane/nonfinite waypoints produce an
invalid-path diagnostic. Bad argument types raise exceptions.

When request is supplied, its constraints must equal the supplied constraints.
The first x-z position must match start within 1e-9 m and the last must be
within goal_tolerance metres. Goal tolerance is positional only. Without a
request, endpoint matching is intentionally unavailable. A one-waypoint path
is valid if all applicable checks pass. Length is geometric translation length.

## Verification and progression

tests/planner/test_planner_contracts.py includes a test-only straight-line
GlobalPlanner. It demonstrates successful planning in free space, rejection
of a blocked line, INVALID_START and INVALID_GOAL. It is not a search baseline
and is not exported from the production package.

Run `pytest tests/planner -q` and `pytest tests -q` from the repository root.
Verified on 2026-09-07: 47 Planner tests passed; full suite 465 passed,
1 skipped (optional SciPy validation dependency unavailable).
Public contracts are covered by invalid numeric data, frozen constraints,
result invariants, continuous collision, footprint contact, concave boundary,
endpoint tolerance and path-length tests.

The agreed next sequence is v0.6b standard A*, v0.6c constraint/current-aware
improvements, v0.6d RRT/RRT*, Trajectory v0.7, then Navigation v0.8.
Algorithm-specific settings belong to their future configurations, not
PlanningConstraints. Empty algorithm packages are deferred until needed.
