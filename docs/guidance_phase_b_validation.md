# Guidance v1.0 Phase B — initial adapter verification

Status: adapter implemented; six ordered small fixtures verified. The frozen sixteen
scenarios and controlled timed-versus-LOS comparison have **not** been run here.

## Contract and unchanged behavior

`navigation/los_adapter.py` maps GuidanceCommand directly to BodyGuidance3DOF.
Navigator accepts an optional `guidance_adapter`; omission retains timed tracking.
The existing generator supplies validated processed geometry and an archived timed
reference. LOS uses `generated.shortcut_path`, not the advancing reference clock.

- Every run initializes fresh explicit progress and resets existing policies/controller.
- STARTUP freezes progress while physical time, control and actual safety checks run.
- Only PATH_FOLLOWING calls compute_los. `step_guidance` explicitly bypasses the
  position/depth outer loops; inner PID, restoring feedforward, effective limits and
  anti-windup are unchanged. No fabricated actual-position reference is used.
- terminal_ready latches TERMINAL_CAPTURE for the **next physical step**, one dt later.
  The ready step remains PATH_FOLLOWING and applies its recorded LOS command.
- `TerminalCapturePolicy.update_active` takes actual time and an endpoint/final-tangent,
  zero-surge terminal reference, without inventing a reference-end timestamp. It shares
  the original timed capture implementation: braking, guards, local safety and budgets.
- Capture freezes progress and cannot resume LOS. Initially qualified arrivals retain
  settling with explicit zero surge/final tangent. Singletons bypass startup.
- LOS requires body-guidance capability, a capture policy, and entry distance at least
  the Navigation position tolerance.

Original swept-segment collision and eroded-boundary checks remain active. No Core,
Physics, PID, startup configuration, capture guard, actuator or goal threshold changed.

## History and metric semantics

New NavigationStep fields: guidance_phase, guidance_command, path_progress_s and
path_segment_index. Commands retain projection/committed progress, diagnostic segment,
cross-track residual, lookahead point and switch/clamp evidence. Outside following,
the command is None and committed progress is retained. Existing effective body targets,
PID diagnostics, applied inputs, current, swept clearance and safety flags remain logged.

For LOS, trajectory/reference_history are archived timed-baseline data, not the active
control target. reference_time is None. Timed reference_duration, tracking RMSE and
timed settling metrics are None; the runner labels old per-step timed errors explicitly.
Common metrics instead use finite-polyline deviation, phase-separated following error,
physical/capture time, control effort/saturation and actual-motion safety.

## Configuration fixed before experiments

See `config/guidance_phase_b.json`. dt=0.05 s, horizon=120 s; existing position/speed/q
tolerances 0.25 m / 0.10 m/s / 0.10 rad/s, hold 0.5 s, actuator caps 20 N / 8 N m.
The existing restoring-candidate YAML (coefficient 6.0) and default startup/capture
settings were used unchanged. No settings were adjusted after observing results.

Lookahead=0.5 m; switch radius=0.2 m; forward window=2 m; progress slack=0.05 m;
max switches/update=3; terminal entry=0.25 m. Constant surge=0.3 m/s, except the
existing low-speed detour fixture at 0.2 m/s. Current fixture: world ENU [0,+0.05] m/s.
These are diagnostic settings, not an optimized envelope: lookahead spans two nominal
0.25 m samples, window/count cap bounds admission, slack permits local catchup, and
terminal entry matches the existing positional tolerance. Future sensitivity sweeps
must be separate from this recorded configuration.

Open lines: 3 m x displacement and z change 0 / +1 / -1 m, initially at rest/zero pitch.
Isolated corner: two 5 m legs, 30-degree turn, initially aligned cruising surge to
isolate the turn. Detour reuses the existing A*-derived processed path and original
initial pose via PrescribedPathPlanner. This measures execution, not planner speed.

## Results

Each case ran twice on the same Navigator. All six were deterministic, finite, had
no secondary clipping, no progress regression, no collision and no boundary departure.
Original arrival speed, position, pitch-rate and hold conditions were checked independently.

| Case | Status | Duration (s) | Final position error (m) | Capture (s) |
| --- | --- | ---: | ---: | ---: |
| Horizontal | SUCCESS | 12.95 | 0.0142 | 1.45 |
| Ascent | SUCCESS | 20.30 | 0.0193 | 1.45 |
| Descent | SUCCESS | 20.30 | 0.0193 | 1.45 |
| Isolated 30-degree corner | SUCCESS | 36.70 | 0.0479 | 1.60 |
| Detour, 0.2 m/s | SUCCESS | 57.50 | 0.1088 | 1.15 |
| Vertical current +0.05 m/s | SUCCESS | 14.30 | 0.2005 | 2.85 |

Important limitations:

- Detour minimum obstacle clearance minus required clearance is **0.07117 m**;
  this single-fixture margin is not proof of robust collision avoidance.
- Maximum following deviation: corner **0.2478 m**, detour **0.2964 m**.
- Maximum actual |q|: corner **0.3519 rad/s**, detour **0.5549 rad/s**. The existing
  0.35 rad/s startup/capture guard is phase-local, not a full-task constraint. It was
  not relaxed or extended. SUCCESS does not imply whole-task rate boundedness.
- Largest wrapped LOS pitch step: corner **0.01501 rad**, detour **0.02427 rad**.
  No filter was added; other geometries may still produce larger jumps.
- All runs had zero actuator-limit saturation fraction. This does not establish
  robustness to saturation, model mismatch or near-vertical paths.
- No superiority over timed tracking is claimed without a matched comparison.

## Reproduction and evidence

```powershell
.\.venv\Scripts\python.exe -m validation.guidance_phase_b --stage straight --output artifacts/guidance_phase_b_straight_v1
.\.venv\Scripts\python.exe -m validation.guidance_phase_b --stage remaining --output artifacts/guidance_phase_b_remaining_v1
.\.venv\Scripts\python.exe -m pytest -q -p no:cacheprovider --junitxml=artifacts/guidance_phase_b_tests_v1.xml
```

Use a new output directory to repeat; the runner refuses to overwrite existing evidence.
Each run saves config, per-case full history, checks, summary, source SHA-256 and
artifact manifest. Artifacts stay ignored by Git.

**11 new adapter/integration tests; 751 full-repository tests passed.** Tests cover
direct mapping, tighter limits, progress freeze, explicit capture before timed end,
singleton settling, bounded failure, swept collision, configuration rejection, real
straight-line signs and reset repeatability. Existing startup/capture regressions pass.

Next: explicitly controlled comparison and sixteen-case regression. No smoothing,
adaptive LOS, new slowdown, observer or PID tuning was introduced in this phase.
