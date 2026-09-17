# Navigation v0.8 static closed-loop baseline

Status: implemented; M3 baseline acceptance recorded separately in
navigation_v0.8_validation.md. Navigation is an orchestration package beside
Simulation. Simulation runners load configurations and choose experiments;
Navigator executes a mission using injected planner, generator, controller and
physics. No lower-level module imports Navigation.

## Frozen coordinate and reference contract

World ENU uses x East and z Up. Body uses surge u forward, heave w down and
pitch rate q positive nose-up. The implemented Physics v0.4 kinematics are
`x_dot=u*cos(theta)+w*sin(theta)` and
`z_dot=u*sin(theta)-w*cos(theta)`. Thus atan2(delta_z,delta_x) is the correct
M2 tangent pitch. The opposite z_dot formula in the M3 proposal is not adopted.
True-Physics ascending and descending tests verify this directly.

Reference u is desired absolute velocity expressed along the body surge axis,
not velocity relative to water. Physics computes relative velocity separately.
M2 supplies no current compensation. Its corner pitch jumps and q=0 on open
straight segments remain baseline limitations. Navigation samples that exact
policy and does not interpolate attitude or invent smoothing.

## Public interfaces

NavigationRequest contains initial_state, goal, WorldModel and
planning_constraints. Construction enforces valid Core 3-DOF numerical fields;
environmental infeasibility is reported as PLANNING_FAILED. Only NoCurrent and
ConstantCurrent are accepted. NavigationConfig contains dt, max_duration,
position/speed/pitch-rate tolerances, settle_time and surge/moment limits.
Defaults are synthetic experimental settings, not identified vehicle limits.
All numbers are finite, nonnegative and nonboolean; dt, duration and control
limits are positive. Duration is an integer number of dt steps, at most one
million. Planning goal tolerance must not exceed navigation position tolerance.

Navigator accepts GlobalPlanner, Controller3DOF, PhysicsBackend and a generator
callable matching generate_trajectory. run(request) returns NavigationResult.
Physics.step is required to be stateless with respect to the supplied state;
Navigator does not invent a reset method absent from PhysicsBackend. Controller
reset is called per mission. Initial velocity and pitch belong to the actual
state, while the reference starts on M2's outgoing tangent and nominal speed.

## Initialization and scheduler

1. Plan once and verify the returned Path against the original request.
2. Generate once with start_time equal to initial_state.timestamp; validate
   every reference state's reduced numerical fields and the complete geometry.
3. Reset controller and initialize synchronized state/reference histories.
4. At t_k sample reference, call controller, validate active generalized forces,
   clip surge/moment to Navigation limits, query current at actual pose and t_k,
   then call Physics.step with fixed dt.
5. Reject nonfinite/inactive state or incorrect/non-increasing timestamp before
   adding it to valid history. Check the complete actual endpoint chord against
   obstacles and boundary using vehicle_radius+safety_margin.
6. Record the transition and x_(k+1), with reference sampled at the same new
   timestamp. Safety failures precede goal success, which precedes timeout.

The existing PID already clips its returned command internally. Commanded
means the Controller API output, not its hidden pre-clipping signal. Navigation
records commanded and applied values separately. Extra clipping can produce
windup because the PID does not receive actuator feedback; the deliberate
tighter-limit saturation experiment measures this without changing PID.

Control v0.5.1 integration update: Navigation now negotiates the optional
LimitAwareController3DOF capability and supplies its caps before integration.
For that mode defensive clipping must not change the returned command; a
violation becomes CONTROLLER_FAILURE. Legacy backends still use the behavior
above and are recorded as limits_aware=false. NavigationStep additionally stores
optional immutable controller diagnostics containing combined pre-clip requests,
effective limits and the public output. Saturation metrics use diagnostic
effective limits when available. See control_v0.5.1_validation.md.

## Safety and arrival

Obstacle contact or safety-margin violation returns COLLISION. Leaving the
clearance-eroded permitted boundary returns OUT_OF_BOUNDS. Collision takes
precedence when both occur; both diagnostic booleans are retained. There is
no option to disable safety or continue a collided mission in this baseline.

Continuous checking applies to the segment between successive numerical
positions, preventing endpoint-only tunnelling. It is not proof that the true
curved continuous-time solution or all internal RK stages are collision-free.
The fixed dt and this approximation must be disclosed in experiments; later
timestep-sensitivity or substep checks can assess that limitation.

Success requires position error <= goal_position_tolerance, sqrt(u²+w²) <=
goal_speed_tolerance, abs(q) <= goal_pitch_rate_tolerance, continuously at the
sampled checks for settle_time. The hold counter accumulates dt only when both
ends of a step qualify, and resets otherwise. No final pitch-angle tolerance
is required. Initial qualification succeeds immediately only if settle_time=0.
After the reference ends, M2 zero-speed terminal holding continues until
success or the original max_duration horizon; no hidden extra timeout exists.

## Outcomes and history

SUCCESS, PLANNING_FAILED, TRAJECTORY_FAILED, COLLISION, TIMEOUT,
OUT_OF_BOUNDS, NUMERICAL_FAILURE, CONTROLLER_FAILURE and ENVIRONMENT_FAILURE
are terminal statuses. Expected backend exceptions become a classified result
with diagnostic type/message; invalid constructor/request API data raises.

For N completed finite transitions, state_history and reference_history each
have N+1 matching timestamps; control_history has N records. Each record stores
the command, applied control, current and pre-step errors at t_k, plus complete
segment safety at t_(k+1). Failed initialization has only the initial state and
possibly no references. A failed numerical transition is excluded from valid
history; failed_command/applied/current retain available attempted inputs.
Collision endpoints remain in history. The final status is in NavigationResult;
preceding transitions are interpreted as RUNNING.

## Metrics and interpretation

Position and wrapped pitch RMSE/max use aligned state/reference samples,
including the terminal sample. Velocity error per transition uses body u/w.
Terminal position is measured to the requested goal centre. Completion time
is elapsed simulated time; reference duration is separate. Settling time after
reference means max(0, completion_timestamp-reference.end_time) on success;
goal_held_seconds reports the sampled qualification interval.

Minimum reference/executed clearance are centre-to-obstacle distances over
complete line segments (Environment.segment_clearance), not waypoint minima.
Safety margin subtracts vehicle radius plus configured margin. These metrics
exclude boundary distance, which is checked separately. Obstacle-free infinity
is represented as null in JSON, not zero clearance.

Per-axis maximum, RMS and total variation use applied controls. Saturation
count/duration/fraction means either applied axis touches its configured limit;
navigation_clip_count counts commands modified by Navigation. The summed
square-input integral is named control_effort_proxy. It mixes surge/moment
components with implicit unit weights; it is not joules or true vehicle energy.
Planner time/nodes remain in the planning result and are not controller metrics.

## Acceptance scope

Required success baselines: horizontal line and static narrow passage.
Initial-condition variants, diagonal settling, deliberately tighter input
limits, obstacle detours and vertical current are informative experiments.
Their collision/timeout/out-of-bounds outcomes are structured failures, not
successful missions. Current parallel/antiparallel to horizontal travel uses
the same reference, controller and Physics; no current-aware tuning occurs.

This milestone excludes replanning, dynamic obstacles, MPC, RRT*, RL, 6-DOF,
ROS/Gazebo, variable current and higher-order reference smoothing.

## Optional terminal-capture extension

The default remains terminal hold. An explicitly supplied `TerminalCapturePolicy`
adds bounded brake/align/move/settle guidance and `CAPTURE_FAILED` termination.
Its explicit controller capability bypasses position/depth outer loops, not actuator
limits or inner-loop anti-windup. Original timed references remain separately recorded.
See [terminal capture contract and verification](terminal_capture_policy.md) for the
authoritative candidate bounds, evidence and limitations. Low-speed detour success
does not establish robust obstacle margin or justify changing the global default.

## Optional controlled startup

An explicit `ControlledStartupPolicy` pauses only the path-reference clock while
physical time, actuator limits and actual-motion safety continue. Release requires
measured attitude/speed/rate qualification; startup drift and time budgets can return
STARTUP_FAILED. No initial-pose replacement or controller reset occurs at release.
`NavigationStep.reference_time` records the trajectory query time; reference states
retain physical timestamps. Terminal capture and settling account for startup delay.
The default None retains the original clock. See the
[candidate contract and separate timing comparison](controlled_startup_and_corner_timing.md).
