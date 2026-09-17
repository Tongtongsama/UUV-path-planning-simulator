# Optional TerminalCapturePolicy — contract and verification

Status: opt-in experimental Navigation policy; verified on the specified synthetic,
no-current fixtures on 2026-09-16. Not a new global default or a robustness claim.

## Ownership and interface

`Navigator(..., capture_policy=TerminalCapturePolicy())` explicitly enables capture.
Omitting the policy preserves terminal hold. Navigation owns phase selection;
Core and trajectory generation are unchanged. No smoothing or corner slowdown is added.

The controller capability `GuidanceController3DOF.step_guidance` consumes an explicit
`BodyGuidance3DOF(u, pitch, q)`. It bypasses position/depth outer loops, while retaining
the inner surge/pitch feedback, actual-pitch restoring feedforward, effective actuator
limits and conditional integration. Integrator memory is retained across transitions.
No reference position is replaced with actual position. Controllers lacking the
capability are rejected when capture is requested.

`reference_history` always contains the original timed reference. Per-step
`effective_guidance`, `capture_phase` and controller diagnostics separately record
the capture command, raw request, controller output and applied control.

## State machine and bounds

Before reference end the policy does nothing. If the original goal conditions are
already satisfied, the original settling process continues without capture. Otherwise
BRAKE precedes ALIGN, then a short MOVE, followed by BRAKE and reassessment.
Qualification during capture enters SETTLE; loss of qualification returns to BRAKE.

- BRAKE requires translational speed <= 0.03 m/s and |q| <= 0.03 rad/s before
  choosing a correction. It holds the previous commanded attitude and requests
  surge opposite actual surge, capped at 0.15 m/s. This is active braking on the
  synthetic bidirectional actuator model, not a guarantee for all AUV actuators.
- ALIGN slews the pitch reference at at most 0.15 rad/s; MOVE begins once heading
  error <= 0.08 rad and |q| <= 0.03 rad/s. Corrections request +/-0.08 m/s for at
  most 3 s before reassessment.
- Forward and reverse candidate headings are limited to +/-60 degrees. Selection
  requires positive progress toward the goal and a valid short candidate segment.
  A side/rear goal does not cause a pi-radian heading jump. Actual pitch beyond
  70 degrees or |q| beyond 0.35 rad/s yields CAPTURE_FAILED.
- Capture budget is 60 s with at most 6 correction attempts. A step that would
  exceed the budget is rejected. The original 120 s mission horizon still applies.
- Original acceptance thresholds remain: position 0.25 m, speed 0.10 m/s,
  pitch rate 0.10 rad/s and continuous sampled qualification for 0.5 s.

These are candidate configuration values, not identified physical feasibility limits.
Bounded reference rates do not imply identical actual rates; actual guards remain active.

## Safety and failure semantics

Candidate straight segments (0.05–0.30 m) are only a preliminary geometric filter.
Every actual fixed-step movement still undergoes the existing obstacle-clearance and
eroded-boundary checks. No safe candidate, exceeded budget/attempts, or attitude guard
failure produces CAPTURE_FAILED; collision and out-of-bounds retain their own statuses.

Safety uses the existing circular-footprint, endpoint-chord model. It is not a proof
of the continuous nonlinear intra-step curve or an elongated body's rotational swept
volume. No-current tests do not establish current robustness or station keeping.

## Reproducible evidence

Final evidence directory: `artifacts/navigation/terminal_capture_verified_v2_20260916/`.
It contains per-case configuration/history JSON, summary, comparison PNG/SVG,
full pytest log, installed package versions, source hashes and artifact manifest.
The interrupted `terminal_capture_verified_20260916` directory is not acceptance evidence.

Re-run into a new directory (the runner refuses to overwrite an existing directory):

```powershell
.venv\Scripts\python.exe -m validation.terminal_capture_acceptance --output artifacts/navigation/terminal_capture_new_run
```

Full suite: **637 passed**, including 15 terminal-capture tests. All 26 enabled/disabled
cases met their declared acceptance expectations; each was repeated deterministically.
All 13 enabled cases succeeded. Disabled residual/detour timeouts are expected diagnostic
controls, not successful missions. Tests include blocked correction, actual segment
collision, budget/attempt exhaustion, attitude guards, explicit outer-loop bypass and
already-arrived real-Physics behavior. NumPy-backed arrival comparisons are explicitly
converted to Python bool at the Navigation boundary.

Five isolated fixtures cover both lateral residual signs, overshoot, mixed residual
and already-arrived state. Their test-only trajectory has one stationary tick followed
by a terminal target; it is not a cruising time-parameterization or tracking benchmark.
Eight complete fixtures cover horizontal/ascending/descending at 0.2 and 0.5 m/s,
plus the original detour at 0.2 and 0.3 m/s. Initial attitude, tolerances, clearance
and actuator limits were not relaxed.

| Case | Hold result | Capture result | Capture elapsed | Final error |
| --- | --- | --- | ---: | ---: |
| Lateral up/down | TIMEOUT | SUCCESS | 26.65 s | 0.2444 m |
| Mixed residual | TIMEOUT | SUCCESS | 13.30 s | 0.2291 m |
| Detour 0.2 m/s | TIMEOUT | SUCCESS | 19.30 s | 0.2291 m |
| Detour 0.3 m/s | TIMEOUT | SUCCESS | 24.65 s | 0.2331 m |

Capture elapsed includes braking, alignment, movement and settling. Total detour
mission times are 66.50 s and 56.10 s respectively. Success means the unchanged
goal region and settling conditions, not convergence to exactly zero position error.

Original pre-terminal state sequences were identical with capture enabled/disabled.
Original-reference moving-phase position RMSE remains 0.33046 m / 0.46613 m for
the two detours. Whole-history RMSE still measures the original reference and must
not be presented as improved original trajectory tracking by substituting guidance.

## Limitations and next decision

Success and safety regression passed, but performance is not uniformly improved:
at 0.5 m/s horizontal completion increases from 23.35 to 28.75 s, and slopes from
23.85 to 29.55 s. The already-arrived fixture requires no capture attempts.

Detour minimum obstacle clearance is 0.32702 / 0.30799 m against 0.30 m required:
only **27.02 / 7.99 mm** net margin. Capture resolves these specified terminal failures;
it does not resolve startup/corner execution margin, model mismatch or robust safety.
Keep the policy optional. Next isolate startup and turning errors before considering
speed scheduling or smoothing; do not describe the full complex-scene suite as closed.

Follow-up: [startup/corner execution-margin diagnosis](startup_corner_execution_margin.md)
separates pre-corner clearance loss from the observed turning envelope, without
changing this capture policy or the production controller.
