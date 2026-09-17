# Control v0.5.1 — optional restoring feedforward acceptance

Date: 2026-09-15. Status: implemented and verified within the tested control scope.
The zero-coefficient baseline remains the default. Detour mission completion is
not solved, and no terminal policy, corner slowdown or smoothing is added.

## Delivered interface and behavior

- `CascadedPID3DOFParameters.restoring_pitch_coefficient=0.0`: finite,
  nonboolean, nonnegative; missing field in old YAML loads as zero.
- `step(current, reference, dt)` and `reset()` remain available.
- `ControlLimits3DOF` holds positive finite symmetric surge/moment caps.
- Runtime-checkable `LimitAwareController3DOF.step_with_limits` accepts this
  step's available caps. Effective limits are the minimum of configuration and
  supplied limits. They may change at each call; larger supplied limits cannot
  override the controller's configured maximum.
- Feedforward is `coefficient*sin(actual_pitch)`, with the sign cancelling the
  positive restoring term subtracted in Physics. Physics code is unchanged.
- Pitch PD + feedforward + accepted integral are summed before final clipping.
  The same non-integral sum and effective cap are used for conditional
  integration. Surge uses its own effective cap identically.
- Navigation supplies its caps before integration for capability-aware backends.
  Its defensive clip must leave their output unchanged; exceeding the supplied
  limit returns CONTROLLER_FAILURE. Legacy backends retain their old step/clip
  flow and are explicitly marked `limits_aware=false`.

The implementation does not query Environment or Physics for limits or model
coefficients. Dynamic actuator derating must be supplied before step; unknown
post-controller rate limits, allocation or saturation are not covered.

## Three distinct control signals

`ControllerDiagnostics3DOF.requested` is the combined pre-clip force/moment.
`ControllerDiagnostics3DOF.output` equals `NavigationStep.commanded`, the public
controller output. `NavigationStep.applied` is the final Physics input.

Immutable diagnostics also record timestamp, effective caps, actual target
construction (forward/depth errors, surge target, travel direction, depth
correction, pitch target/error), PD, feedforward and integral memory before/after.
`last_diagnostics` clears on reset or a failed attempt. Navigation saves the
diagnostic snapshot per step and rejects stale timestamp/output mismatches.
For built-in control, saturation metrics use diagnostic effective caps, including
when configured controller caps are tighter than Navigation's caps.

This does not claim that commanded is an unlimited internal PID signal. Histories
remain N controls with N+1 actual/reference states. At the final post-step state
there is no newly executed control; reports retain the last control's timestamp
rather than fabricating a command at final time.

## Fixed opt-in candidate

Candidate file: `config/controllers/cascaded_pid_3dof_restoring_candidate.yaml`.
All gains and configured limits match the old baseline; coefficient is explicitly
6 N m, matching synthetic v1. Existing default YAML is unchanged.

```python
from pathlib import Path
from controller import CascadedPID3DOFController
from simulation.controller_config import load_baseline_controller_parameters

parameters = load_baseline_controller_parameters(
    Path("config/controllers/cascaded_pid_3dof_restoring_candidate.yaml")
)
controller = CascadedPID3DOFController(parameters)
```

Use a new/reset controller per mission; no live coefficient switching or bumpless
gain transfer is introduced. This is a fixed candidate for subsequent diagnosis,
not a global-default replacement or identified real-vehicle model.

## Regression and control acceptance

Full suite: **622 passed, no skips**, pytest 8.4.2 / project Python 3.10.9 venv.
18 new formal tests cover old YAML, disabled-feedforward step-by-step equality
against the independent baseline-equation prototype, angle wrap/saturation,
changing effective limits, positive/negative saturation freezing/unwinding,
integral recovery, actual-pitch FF, no integral accumulation at zero-error static
equilibrium, pre-clip composition, immutable/aligned diagnostics, invalid inputs,
tighter Navigation limits, legacy fallback and false capability violations.

The independent prototype is kept as an experiment fixture. Its explicit
limits-aware method preserves its own equations when Navigation negotiates
capabilities; it is not the implementation used for the formal acceptance run.

The disabled/default regression guarantee applies to the same effective limits.
When Navigation supplies tighter caps, anti-windup intentionally differs from
the old unaware controller. For example the 2 N surge-limited experiment now
succeeds; reproducing its old windup-induced failure is not a compatibility goal.

## Real-module experiments

17 cases, each run twice with identical states, references, controls and metrics.
All diagnostic/output checks passed; no limits-aware Navigation clip changed a
controller output. Position/speed/pitch-rate/hold tolerances remain
0.25 m / 0.10 m/s / 0.10 rad/s / 0.5 s; maximum duration remains 120 s.

| Group | Outcome |
| --- | --- |
| FF off, 0.5 m/s horizontal | SUCCESS 23.35 s; original result retained |
| FF off, 0.5 m/s ascending/descending | TIMEOUT, error 0.2621 m; original result retained |
| Candidate, horizontal 0.2 / 0.5 m/s | SUCCESS 41.55 / 23.35 s; error 0.1925 / 0.2120 m |
| Candidate, ascending/descending 0.2 m/s | SUCCESS 42.75 s; error 0.1875 m |
| Candidate, ascending/descending 0.5 m/s | SUCCESS 23.85 s; error 0.2142 m |
| Coefficient 4.8, ascending/descending 0.5 m/s | SUCCESS 23.95 s; error 0.2137 m |
| Coefficient 7.2, ascending/descending 0.5 m/s | SUCCESS 24.05 s; error 0.2171 m |
| Candidate, surge cap 2 N, horizontal | SUCCESS 25.15 s; saturation fraction 0.771 |
| Candidate, pitch cap 0.5 N m, ascending | TIMEOUT; error 1.2147 m; recorded failure |
| Candidate, unchanged detour 0.2 / 0.3 m/s | TIMEOUT; error 0.2872 / 0.4150 m; no collision/boundary exit |

Physics restoring coefficient stays 6 in the 4.8 and 7.2 tests. These four
successful points provide limited ±20% coefficient-mismatch evidence only, not
a general robustness guarantee, worst-case bound or a validated real vehicle.
The severe pitch cap is expected to limit performance; its structured failure is
not silently promoted to mission success. Automated acceptance distinguishes
required-success cases, preserved baseline outcomes and exploratory cases.

The zero-error equilibrium test proves FF does not itself cause repeated integral
accumulation there. Low-speed terminal histories also show small residual pitch
integrals, rather than a second integral contribution equal to the restoring
moment. This is not a guarantee of zero integral under arbitrary disturbances.

## Evidence and reproduction

```powershell
.\.venv\Scripts\python.exe -m validation.control_v051_acceptance --output artifacts/controller/v051_run_002
```

Archive: `artifacts/controller/v051_20260915/`. Each case contains parameters,
full histories, diagnostics, deterministic comparison, acceptance checks and
terminal snapshots. Summary includes source hashes, installed versions and test
command; pytest.log records the complete suite. Manifest and recorded source
hashes verified. The terminal PNG was visually inspected. Artifacts remain
Git-ignored; no source-tree copies or historical artifact edits.

The original 16-scene M3 acceptance suite has not been re-run as the next
terminal/turning work is not yet implemented. See
[terminal residual classification](terminal_residual_classification_v051.md).
