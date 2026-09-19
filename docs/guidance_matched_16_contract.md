# Preregistered matched 16-case comparison

Freeze this contract before examining sixteen-case outcomes. No outcome-based tuning.

## Candidate and experimental arms

Candidate: `guidance-v1.0-phase-b-candidate`, NOT a final release. Git HEAD plus dirty
working-tree SHA-256/ZIP identifies its actual code; HEAD alone does not include Phase B.
Original six-case evidence and 751-test JUnit are verified and retained in the archive.
Historical v0.8 evidence is read-only and is not reused as the matched timed arm.

Order: all sixteen LOS cases, two runs each, then all sixteen timed cases, two runs each.
Scene source: unmodified build_cases('all', config/navigation_v08.json), including
six minimum cases, six static scenes and four current cases. The zero-current case
duplicates narrow passage; these sixteen fixtures are not independent random trials.

Both arms use the original scene nominal speed **0.5 m/s**, NOT the 0.3/0.2 m/s used
for initial Phase B fixtures. LOS geometric parameters remain from guidance_phase_b.json:
lookahead .5 m, switch radius .2 m, window 2 m, slack .05 m, max switches 3, entry .25 m.
Only LOS nominal_speed is set to the common original scenario speed.

No corner_timing/local slowdown, new speed scheduling, filter, observer, smoothing,
adaptive/integral LOS, current compensation, replanning or PID tuning.

## Shared conditions and deliberate difference

Plan once with the scene's A* configuration and run the original generate_trajectory
postprocessing once. Cache that same PlanningResult and TrajectoryGenerationResult
for both arms; record raw/processed path and common-input fingerprints. Planner wall
time is neither part of controller performance nor repeated-run determinism checks.

Same initial state (no hidden pitch alignment), static world/current, synthetic Physics,
restoring-candidate PID YAML, actuator caps/anti-windup, dt/horizon, goal tolerances,
hold time, swept safety checks, TerminalCapturePolicy and post-freeze startup defaults
(70-degree command / 75-degree actual guard and aligned-motion release). The saturation
scene keeps its original tighter surge cap. Persist all effective configs.

Timed uses time-sampled references and the existing position/depth outer loops.
LOS uses spatial finite-segment guidance and step_guidance, bypassing those outer loops.
The inner controller is identical, but this is a guidance-system comparison: it does
not isolate ONLY the clock variable. Terminal handoff triggers intentionally differ.

## Common diagnostic definitions

- Full-history finite-polyline RMS/max distance, plus following-phase RMS/max distance.
  Include both endpoints of following transitions; diagnostics may use global nearest
  distance, never to steer LOS. Failures are censored, not fast completion successes.
- Minimum swept obstacle clearance and net clearance after required footprint margin.
- Minimum signed eroded rectangular-boundary margin over all states; for these convex
  rectangles/chord checks this also bounds each chord's minimum margin. Not a general
  polygon clearance formula. Null obstacle clearance means no obstacle, not zero.
- Physical duration, capture duration, control-effort proxy, saturation fraction.
- Max actual |q| over **all** states. rate_envelope_violation = max|q| > .35 rad/s.
  Diagnostic ONLY; does not terminate or change policy guards. Save first exceedance.
- Max wrapped effective pitch-target step across all transitions (shared definition),
  plus raw reference step within following and within-phase effective steps. Phase
  handoff discontinuities are included in the all-transition metric, not hidden.
- LOS progress clamp count, switch-limit count, committed switch count, monotonicity
  violations. Timed equivalents are null, NOT zero (no such progress mechanism).
- Stalled LOS TIMEOUT: committed progress advance <= .01 m over the final five seconds
  of PATH_FOLLOWING AND at least five seconds of samples; this is a descriptive flag.

Safety/termination taxonomy: STARTUP_FAILED, COLLISION, OUT_OF_BOUNDS, CAPTURE_FAILED,
TIMEOUT (with progress-stall flag), SUCCESS. Rate violation is orthogonal to outcome.
At first safety violation record physical phase, raw/effective reference, actual state,
processed-path segment, residual/lead, saturation, swept clearance and boundary margin.
Never classify all non-null effective_guidance as capture: LOS also uses this interface.

Interpretations (time lead, corner geometry, current margin, capture, attitude envelope)
must be labelled hypotheses from observed evidence, not proven causal attribution.
Do not immediately fix failures. Primary output is per-case paired results, category
counts, diagnostic evidence and determinism/shared-input checks, not a global average
of successful and failed mission duration.
