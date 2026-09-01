# Environment / World Model v0.3 Specification

## 1. Purpose

Environment v0.3 is a lightweight planning world model. It provides stable
queries to planners, simulation tests, and the future Physics module while
delegating geometry operations to Shapely. It is not an ocean simulator,
renderer, collision-response engine, sensor model, or Gazebo world.

The geometry backend is an implementation detail. Callers use Environment
domain objects and `WorldModel` queries rather than Shapely objects directly.

## 2. Planning plane and frames

Core `Pose` uses the ENU inertial/world frame:

- `Pose.x`: East, metres
- `Pose.y`: North, metres
- `Pose.z`: Up, metres

Environment v0.3 plans in the vertical `x-z` plane. The mapping is:

```text
Core ENU (Pose.x, Pose.z) -> planning (x, z) -> Shapely (x, y)
```

Shapely's second coordinate therefore represents project `z`; it never means
`Pose.y`. All conversions must pass through `environment.coordinates`. Planner
and simulation code must not reproduce this mapping independently.

Configuration coordinates such as `center: [30.0, -12.0]` are explicitly
`[x, z]` planning coordinates. All coordinates and query values must be finite.

## 3. Backend and dependency boundary

- Environment may depend on Core, NumPy, Shapely, and PyYAML.
- Core must not depend on Environment.
- Environment must not depend on Planner, Controller, Physics, ROS 2, Gazebo,
  or visualization.
- Shapely geometry remains encapsulated inside Environment.
- A future backend may replace Shapely without changing the five primary
  `WorldModel` queries.

## 4. Domain objects

### Obstacle

A static obstacle has a unique non-empty `obstacle_id`, backend geometry, and
optional metadata. v0.3 supports circles, axis-aligned rectangles, and valid
polygons. Dimensions must be positive, bounds ordered, and polygons valid with
non-zero area.

Touching an obstacle boundary counts as collision.

### Boundary

The boundary is the polygonal region in which the UUV is allowed to operate.
A point on the boundary is valid when requested clearance is zero. With
positive clearance, the complete circular UUV footprint must lie within or on
the boundary. Boundary semantics remain distinct from obstacle semantics.

### CurrentField

`velocity_at(position, time) -> numpy.ndarray` returns `[v_x, v_z]` in the
planning plane. v0.3 provides `NoCurrent` and `ConstantCurrent`. Position,
time, and returned velocity must be finite; output shape is exactly `(2,)`.

### WorldModel

`WorldModel` composes one `Boundary`, an immutable ordered obstacle collection,
and one `CurrentField`. Its stable public queries are:

```python
contains(position, clearance=0.0) -> bool
collides(position, radius=0.0) -> bool
clearance(position) -> float
nearby_obstacles(position, radius) -> tuple[Obstacle, ...]
current_at(position, time) -> numpy.ndarray
```

`position` accepts either a Core `Pose` or a two-component `[x, z]` value.

Query semantics are frozen as follows:

- `contains`: boundary only; boundary contact is valid at zero clearance.
- `collides`: obstacles only; contact counts as collision.
- `clearance`: minimum distance to an obstacle surface; returns `0.0` inside
  or on an obstacle and positive infinity when no obstacles exist.
- `nearby_obstacles`: includes obstacles whose surface distance is less than
  or equal to the non-negative search radius; results preserve scenario order.
- `current_at`: delegates to the configured current field.
- Negative radius or clearance and non-finite inputs raise `ValueError`.

To determine whether a complete state is navigable, callers combine
`contains(position, vehicle_radius)` and `not collides(position,
vehicle_radius)`.

## 5. Scenario configuration

Scenario is the single source of planning-world data. It can be constructed
from a Python mapping or loaded from YAML. It validates schema values before
building the same deterministic `WorldModel` for the same configuration.

Required top-level declarations are:

```yaml
name: simple_static_world
planning_plane: xz
frame: world_enu
```

Only `planning_plane: xz` and `frame: world_enu` are accepted in v0.3.
Duplicate obstacle IDs, unknown types, malformed YAML, invalid dimensions,
non-finite values, and invalid polygons are rejected with explicit errors.

The same YAML will later feed an SDF adapter. Gazebo is not a dependency of the
Python world model.

## 6. Deferred features

Dynamic obstacles, prediction, occupancy/voxel maps, 3-D meshes, seabed
terrain, turbulent or stochastic current, sensors, rendering, collision
response, cost maps, path search, ROS 2, Gazebo plugins, and SDF generation are
outside v0.3.

## 7. Completion criteria

Environment v0.3 is complete when circle/rectangle/polygon obstacles, polygon
boundary queries, no/constant current, the five `WorldModel` queries, YAML
reconstruction, unit tests, and an Environment smoke demo all pass while the
Core suite remains green and no upper-layer dependency is introduced.

