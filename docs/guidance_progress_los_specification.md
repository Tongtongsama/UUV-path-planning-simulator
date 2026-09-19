# Finite-Segment Progress-Based LOS v1 — specification

Status: Phase A verified; opt-in Phase B adapter and six initial fixtures verified.
The matched sixteen-case comparison is recorded in [the results report](guidance_matched_16_results.md);
see also [initial Phase B evidence](guidance_phase_b_validation.md). Hard-case failures remain unresolved.
Principal reference: MSS LOSchi and
projectToPath; audited sources and differences are in [source audit](guidance_source_audit.md).
This milestone tests spatial reference advance against timed tracking with the same
planned path, controller inner loop, Physics, constraints and physical time budget.

## Frames and geometry

Authority: `physics/frames.py`, ENU world x-z; body u forward, w down; pitch nose-up.

```
xdot = u*cos(theta) + w*sin(theta)
zdot = u*sin(theta) - w*cos(theta)
theta_dot = q
```

The negative-zdot formula in the supplied proposal does not describe this backend.
For a target displacement (dx,dz), theta_LOS=atan2(dz,dx).
Horizontal forward => 0; upward forward => positive; downward forward => negative.
With tangent t=(cos(alpha),sin(alpha)) and normal n=(-sin(alpha),cos(alpha)),
signed cross-track e=n dot (P-Pproj). On a straight segment, a point Delta ahead
of the perpendicular projection gives theta_cmd=alpha-atan2(e,Delta).
Thus e>0 above a horizontal path produces a negative correction. No sign borrowed
from NED is applied without converting axes and the error definition.

For vertices P_i, d_i=P_(i+1)-P_i, L_i=norm(d_i), cumulative arc S_i:

```
lambda_raw = dot(P-P_i,d_i) / dot(d_i,d_i)
mu = clip(lambda_raw,0,1)
Pproj = P_i + mu*d_i
s_projected = S_i + mu*L_i
along_track_error = dot(P-Pproj,t_i)
cross_track_error = dot(P-Pproj,n_i)
```

Arc units are metres. Along-track error is residual relative to the finite projection,
not reference lead against a timestamp. Store lambda_raw for end-plane decisions.

## Modules and interfaces

Implemented Phase A files: `guidance/progress_projection.py`, `guidance/los_guidance.py`,
`guidance/guidance_command.py`; Core remains unchanged.

Phase A consumes Core Path/Pose, explicit previous progress and validated configuration.
There is no VehicleState, dt or clock input: progress advance uses actual displacement.
Return command plus next progress; no module-global persistent state. A mission reset
constructs ProgressState(geometry, actual_initial_pose), initializing segment=0,
committed_s=0 and total_switches=0. The initial pose is not inferred from the path start.
Any future clock/VehicleState conversion belongs to the Phase B adapter.

GuidanceCommand fields: pitch_ref, pitch_rate_ref, surge_speed_ref, progress_s,
projected_s, segment_index, projection_point, lookahead_point, cross_track_error,
along_track_error, distance_to_goal, terminal_ready and switch_count.
Phase A exposes projection diagnostics through command.projection and read-only
properties; valid calls have no status/message or success flag. Bad API inputs raise.
Phase is owned by Navigation: STARTUP, PATH_FOLLOWING, TERMINAL_CAPTURE, COMPLETE
or structured failure. Guidance reports eligibility; it cannot mark mission success.

The Phase B controller conversion uses existing BodyGuidance3DOF and step_guidance. This explicitly
bypasses position/depth outer loops while retaining PID inner feedback, restoring
feedforward, limits and anti-windup. Do not fabricate reference positions to bypass loops.

## Path and input validation

Reject empty path, nonfinite data, nonzero inactive coordinates, nonpositive lookahead,
incompatible progress state and malformed configuration. API type errors
raise TypeError; invalid numerical values raise ValueError.
Consecutive exact duplicate positions are removed once, with original index mapping.
Remaining numerically degenerate segments below the configured geometry epsilon are
rejected rather than divided by zero. Single unique waypoint is a terminal-only mission.
No smoothing or geometry shortcut is introduced in this preparation.
PathGeometry stores normalized position-only points, lengths, cumulative_s and the
first original waypoint index of each consecutive duplicate run. Public point_at_s
rejects out-of-range arcs; LOS explicitly clips its target arc to the endpoint.

## Projection, segment gates and monotonic progress

Search uses the previous active segment, at most one previous segment for diagnostic
projection, and a bounded forward arc window. Parameters are in metres and segment
counts explicitly; no implicit conversion from sample index to distance.

Forward segments are eligible only through sequential endpoint gates. Gate i opens if
distance(P,P_(i+1)) <= r_i, or lambda_raw >= 1 with |cross_track| <= r_i.
Use r_i=min(configured_switch_radius, 0.45*L_i, 0.45*L_(i+1)) at internal vertices.
The tube condition prevents switching just because a vehicle far off the path crosses
an infinite end plane. No global nearest-segment fallback or unconditional next-segment
distance rule. Latch committed forward switches; do not switch back on distance jitter.

Advance through sequential gates in one step only up to max_switches_per_step and the
forward arc window. At the cap, retain consistent state and report switch_limit_reached;
continue later, without silently skipping the rest. Distance ties choose lower progress.
No gate from a nearby but non-adjacent branch can open a remote segment.

Within the admitted region, choose the closest finite projection. Maintain raw projected_s
for diagnostics, but committed_s=max(previous committed_s, admissible projected_s), clamped
to [0,L]. A backward projection must not reset committed progress or the active segment.
Bound per-step progress advance by physical displacement plus a configured projection
slack; report a clamp. Initial state far along a path is not silently relocalized in v1.
Commit progress and segment transitions atomically: a candidate forward transition
is committed only when the bounded accepted arc reaches that segment's start.
Otherwise retain the preceding active segment and defer the transition; an endpoint
gate alone cannot move the committed segment ahead of the accepted arc.
Window size alone is not a guarantee against every self-near geometry; tests exercise gates.

ProjectionResult.segment_index is the committed active segment; projected_segment_index
identifies the segment to which mu, lambda_raw, projection_point and residual errors belong.
They can differ after backward motion or a progress clamp. switch_count counts committed
transitions in this call; next_state.total_switches is cumulative. switch_limit_reached
reports that candidate admission hit its switch cap while another local gate remained
open, even if the displacement bound subsequently prevents committing all candidates.
progress_clamped covers both forward-window and displacement-plus-slack clipping.
Singletons use index 0, zero arc/parameters/errors and their sole point as projection;
these zero errors do not imply zero distance to the goal (there is no tangent).

## Lookahead and speed

Phase A uses one explicitly configured fixed positive Delta; no variable-lookahead
policy or separate minimum/maximum tuning is introduced. Use
s_target=min(committed_s+Delta,L), then piecewise-linear interpolation at s_target.
pitch_ref=atan2(z_target-z_actual,x_target-x_actual). If the displacement vanishes,
retain the last finite command or final segment tangent; do not use atan2(0,0) as a turn.
For a singleton at zero displacement, use actual pitch when no previous command exists.
The previous command is an explicit optional argument, never remembered internally.

This polyline target can lie beyond a corner. It differs from infinite-line LOSchi,
and may command a chord across a corner. Baseline retains actual collision checks;
it is not collision-aware smoothing. Raw pitch_ref may jump. pitch_rate_ref=0 in v1
does not claim derivative consistency; log wrapped pitch reference steps explicitly.

Phase A implements CONSTANT only. EXISTING_CORNER_SLOWDOWN is deferred to a later,
independent configuration after base LOS geometry verification. For that future policy,
reuse exactly the frozen sampled-interval speed field as a function of spatial arc
progress, including its interval-midpoint rule and internal-knot semantics. Do not
quietly substitute a different continuous slowdown zone. No adaptive/integral LOS,
current compensation, variable lookahead, error-based slowdown or observer in v1.

## Startup and terminal ownership (Phase B adapter, separate from Phase A)

Use the same post-freeze startup configuration for every new comparison arm
(70-degree reference / 75-degree startup actual guard plus aligned-motion release).
The older frozen 60-degree configuration remains historical; it cannot be mixed with
new results as a controlled comparison. U-trap rejection remains a reported outcome.
Physical time and collision checks continue through all phases; no vehicle teleport.

After startup, projection commits only during PATH_FOLLOWING. On the final admitted
segment, terminal_ready requires remaining committed arc <= terminal entry distance
AND actual distance to goal <= terminal entry distance. Entry distance must be finite,
positive, at least position tolerance, and logged. Passing an endpoint plane while far
away is not arrival. Missing the gate eventually produces the original mission timeout.

Latch terminal entry for the next physical step and hand over a zero-surge terminal
reference with final tangent to the same TerminalCapturePolicy. The explicit
phase-entry capability is update_active; the ready step remains PATH_FOLLOWING;
do not forge a trajectory duration or timestamp to activate capture. Its finite budget
starts at handoff; total task budget, speed/pitch-rate/hold conditions remain unchanged.
Already-qualified goals use original settling. Failed capture never resumes progress
automatically. Single-waypoint missions go directly to terminal handling.
Phase A therefore reports terminal_ready=True for any singleton, including a distant
one; it requests terminal handling, not success. For other paths it uses all three
conditions above. LOS keeps nominal surge even when ready: the Phase B phase owner
must take over explicitly, rather than applying that command as a terminal stop.
LOSConfig validates a positive entry distance; the adapter must additionally enforce
entry distance >= mission position tolerance, which geometry does not own.

Near-vertical commands remain within the declared whole-task scope: startup guards
alone do not protect tracking. Define and share a full-task command/actual-attitude
policy across all comparison arms before claiming near-vertical robustness; do not
change this only for LOS. Until then retain observed excursions as explicit limitations.

## Logging and fair comparison

Record physical time, method, phase, projected and committed arc, active segment,
lookahead/projection positions, raw/effective guidance, errors, switches, clamps,
commanded/controller/applied controls, saturation, actual state and swept-chord safety.
For spatial guidance reference_duration and timed tracking RMSE are null/not applicable;
do not create a fictitious timed Trajectory merely to populate those fields.

Common metrics: outcome, actual obstacle/boundary margin, RMS/max finite-polyline
distance, total physical time, startup/capture duration, control effort proxy, saturation,
monotonicity violations, switch count, maximum wrapped reference step and actual |q|.
Report path-following and terminal phases separately. Equal dt makes sample RMS
comparable within phases; failed histories are censored and must be marked.

Freeze the same A* and postprocessed geometry for timed, timed+slowdown, LOS constant
and LOS+same slowdown arms. Same initial states, current, inner gains, actuators,
startup/capture parameters and safety/goal thresholds. LOS replaces the outer guidance;
therefore its result is not evidence solely about removing reference time.

## Acceptance order

1. Geometry tests: horizontal/ascent/descent signs, finite projection before/after,
   90-degree/acute turns, zigzag/U/self-near crossings, duplicates/zero lengths,
   singleton path, short-segment multi-switch cap, backward motion and monotonic progress.
2. Lifecycle tests: reset, startup freeze, no premature terminal entry, singleton terminal,
   capture budget and success delegation; unchanged actual collision/limit behavior.
3. Real closed loop: open straight, both diagonals, isolated corner, detour, constant
   current; then the original sixteen cases with the same frozen comparison conditions.
4. Report failures without tuning away individual cases. Observer/rate filter is a later
   independent ablation; no claim of continuous pitch for raw LOS.

Implementation numerical defaults (lookahead, search window, switch radius, progress
slack and terminal entry distance) must be frozen in the experiment config before the
first closed-loop run, with rationale and sensitivity ranges. They are not inferred
optimal from the successful detour cases.

## Phase A acceptance evidence

See [Phase A validation](guidance_phase_a_validation.md) for commands, test counts,
public API usage and evidence artifacts. Closed-loop lifecycle and initial mission
checks are reported separately in Phase B; the passing geometry tests do not establish collision safety,
dynamic feasibility or successful completion of the sixteen scenarios.
