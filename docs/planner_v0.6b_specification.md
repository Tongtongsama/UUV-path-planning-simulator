# Planner v0.6b: A* baseline

Status: implemented. The output remains a geometric Core Path in world ENU
x-z. This milestone adds no time parameterization, smoothing, current cost,
vehicle dynamics, or controller coupling.

## Interfaces

`AStarPlanner(AStarConfig(...)).plan(request, space)` implements GlobalPlanner.
The optional `BoundedPlanningSpace` protocol extends PlanningSpace with public
`bounds = (x_min, z_min, x_max, z_max)`. EnvironmentPlanningSpace exposes the
existing Boundary bounds through this protocol. The base protocol is unchanged.
Bounds describe a search rectangle; containment queries still determine actual
validity inside concave boundaries. A* does not inspect obstacles or Shapely.

AStarConfig is frozen and validated: resolution is positive finite metres;
connectivity is integer 4 or 8 (default 8); max_expanded_nodes defaults to
100000 and max_grid_nodes to 250000. Both budgets are positive integers.
No weighted heuristic is used. Malformed API data raises TypeError/ValueError.

## Occupancy and coordinates

OccupancyGrid samples a regular lattice anchored at `(x_min,z_min)`, with
`x=x_min+ix*resolution`, `z=z_min+iz*resolution`. Shape is
`(floor(width/resolution)+1, floor(height/resolution)+1)`. Storage is the
immutable tuple `occupied[ix][iz]`, where True means invalid. Increasing iz
increases world z (up), independent of image row conventions.

Occupancy describes node footprint validity, not a guarantee that an entire
cell is free. Each node uses vehicle_radius+safety_margin, including boundary
clearance. Every traversed edge also undergoes continuous collision checking.
The same existing Environment polygon geometry defines occupancy and edges.

index_to_pose rejects invalid indices. world_to_index requires an in-bounds
vertical-plane Pose and returns the nearest node, ties upward, clamped to the
last valid index. If the span is not divisible by resolution, the upper strip
has no extra nonuniform node. Endpoint connectors reach it through the last
lattice nodes. The grid must be rebuilt when space, resolution or clearance
changes; the space is required to remain static during a plan.

## Graph and search

Four-connected search permits axial steps. Eight-connected search adds
diagonals only if both adjacent axial nodes are free. Complete edge checks
also reject thin obstacles between safe nodes and concave boundary excursions.
Edge cost is Euclidean distance in metres. The goal is the closed region
`distance(p, goal) <= goal_tolerance`. The consistent heuristic is
`h(p) = max(0, distance(p, goal) - goal_tolerance)`.
After request feasibility checks, a start already in this region returns
the validated singleton Path `[start]`, at zero cost without building a grid.

Off-grid endpoints are connected to their up-to-four surrounding floor/ceil
nodes, filtered by occupancy and continuous connector collision checks. No
endpoint is silently snapped. These local connections are part of the search
graph. The exact start initializes multiple g-scores; the exact goal is a
virtual terminal node queued like other candidates, so the first discovered
goal connector does not prematurely terminate the search. Every valid lattice
node within the goal region is also a terminal. Search accepts terminals when
popped from the heap, not when first discovered. Missing exact-goal connectors
do not prevent search for a reachable region node. With zero tolerance the
exact-goal connector remains available for off-grid goals.

The open set is a heap with stable insertion-order tie-breaking. A g-score
table, predecessor table and closed set support relaxation, stale-entry
discarding and reconstruction. Output and expanded-node count are deterministic
for a fixed static input. planning_time is measured wall time and varies.
The returned path retains the original start Pose. It ends at a reached region
node or the exact goal; it does not append the exact goal after reaching a
region node. Lattice Poses have zero pitch; an exact-goal endpoint retains the
goal Pose. Goal acceptance is positional only, not orientation feasibility.

The shortest route is optimal on this finite connector/lattice graph, not in
continuous free space. Resolution, conservative corner rules and local endpoint
connections can exclude continuous-space routes. max_path_length prunes
candidates whose g+heuristic exceeds the allowed length, including initial
start connectors. This lower bound measures distance to the region. The graph
does not introduce arbitrary continuous endpoints on the tolerance circle or
terminate at intersections along edges; resolution limitations still apply.

## Outcomes and final validation

- INVALID_START / INVALID_GOAL: endpoint footprint is infeasible. The existing
  v0.6a requirement that the requested goal itself be valid remains in force,
  even when some portion of its tolerance region would be free.
- NO_PATH: no start connector or exhausted search under current graph and
  path-length constraints; this does not prove continuous-space impossibility.
- LIMIT_REACHED: occupancy would exceed max_grid_nodes, or expansion budget
  is exhausted. There is no wall-time timeout setting in this baseline.
- SUCCESS: reconstructed Path passes validate_path with the original request.
- INTERNAL_ERROR: a reconstructed candidate fails final validation.

Failures carry no Path. Success path_cost is geometric length in metres;
expanded_nodes counts expanded nonterminal lattice nodes, excluding terminals.
The wall clock covers validation, grid construction and search.

## Reproduction

Verified on 2026-09-09: 30 A* tests, 104 total Planner tests;
full suite 522 passed and 1 skipped (optional SciPy unavailable).

```bash
pytest tests/planner -q
pytest tests -q
python -m simulation.planner_smoke_demo
```

Tests cover coordinate round trips, non-multiple bounds, invalid settings,
4/8-connected free-space costs, independent Dijkstra cost comparison with an
obstacle, deterministic paths, continuous thin-wall detection, corner rules,
exact endpoints, blocked connectors, budgets, path-length constraints,
coincident endpoints and clearance-blocked corridors. Goal-region regressions
cover initial arrival (including equality), missing exact-goal connectors,
region-based length pruning, zero tolerance and Dijkstra region-cost comparison.

Smoke case: start `(1,-5)`, goal `(9,-5)`, bounds `[0,-10,10,0]`, radius 0.2 m,
resolution 1 m. A partial wall at x=[4.9,5.1], z=[-7,-3] produces a validated
10.485281374238571 m path with 32 expanded nodes. Extending the wall to z=[-10,0]
returns NO_PATH with 36 expanded nodes.
