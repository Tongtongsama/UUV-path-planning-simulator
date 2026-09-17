# Navigation v0.8 — M3 re-audit and baseline evidence

Audit date: 2026-09-13.

Status: minimum closed-loop baseline verified in the recorded local environment.
This is not general obstacle-avoidance robustness or real-vehicle validation.
The authoritative contract is [Navigation specification](navigation_v0.8_specification.md).

## Evidence and reproduction

Latest evidence: `artifacts/navigation/m3_audit_20260913/`.
Run from the repository root into a new directory:

```bash
python -m validation.navigation_acceptance --output artifacts/navigation/m3_run_001 --animate
```

The full suite records **588 passed, 1 skipped** in `pytest.log`.
The skipped test is optional SciPy reference validation because SciPy is absent.
Python is 3.10.9; the exact executable, package versions, git state and source
hashes are recorded in `summary.json`. pytest 9.1.1 is outside the current
`requirements-dev.txt` range (<9). This historical runner invocation is not
evidence of a declared-dependency environment. A subsequent isolated environment
verification on 2026-09-13 passed all 589 tests with pytest 8.4.2 and SciPy 1.15.3;
see [dependency verification](dependency_environment_validation.md).
That full-suite check closes the pytest mismatch, but does not rerun or replace
the 16-case M3 runner evidence below.

Each of 16 cases has a separate JSON, PNG and SVG. The narrow-passage animation
has 100 decodable frames. All artifact manifest hashes and recorded Python
source hashes matched at audit time. The PNGs for ascending, narrow passage and
detour were visually inspected. Full histories and exact case parameters are
stored, without a new copy of the source tree. Artifacts remain Git-ignored.

## Re-audit correction

The previous runner completed 587 tests, but plot naming used `with_suffix()`
on identifiers containing decimal current values. This truncated the final
decimal component and allowed the zero-current and vertical-current plots to
overwrite each other. Their JSON histories were not overwritten.

Plot extensions now append to the complete identifier. A regression test
verifies four distinct PNG/SVG outputs for those two cases. The new evidence
directory supersedes earlier M3 folders for figure delivery; historical
artifacts were not deleted or silently modified.

## Measured outcomes

| Case | Terminal status | Simulated time (s) | Final goal error (m) |
| --- | --- | ---: | ---: |
| horizontal | SUCCESS | 23.35 | 0.212 |
| ascending | TIMEOUT | 120.00 | 0.262 |
| descending | TIMEOUT | 120.00 | 0.262 |
| initial_pitch | SUCCESS | 23.35 | 0.214 |
| initial_speed | SUCCESS | 22.55 | 0.213 |
| saturation | OUT_OF_BOUNDS | 26.40 | 0.703 |
| narrow_passage | SUCCESS | 23.35 | 0.212 |
| detour | COLLISION | 8.20 | 4.630 |
| double_baffle_detour | COLLISION | 5.75 | 5.926 |
| edge_forced_detour | COLLISION | 8.35 | 5.350 |
| u_trap_detour | COLLISION | 6.10 | 6.765 |
| zigzag_maze_detour | COLLISION | 5.35 | 6.482 |
| current [0, 0] | SUCCESS | 23.35 | 0.212 |
| current [+0.1, 0] | SUCCESS | 23.35 | 0.215 |
| current [-0.1, 0] | SUCCESS | 23.35 | 0.213 |
| current [0, +0.05] | COLLISION | 6.35 | 5.138 |

All cases reproduced the same states, references, controls, metrics and terminal
status on a second execution. Planning wall-clock time is not compared for
determinism. The set contains repeated baseline conditions and is a designed
diagnostic suite, not an unbiased statistical success-rate benchmark.

`automated_acceptance=true` means the required-success baselines passed, the
recorded exploratory outcomes were consistent with independently checked
termination conditions, repeatability passed, and the tests passed. It does
**not** mean all 16 missions succeeded. The totals are 7 successes, 2 timeouts,
6 collisions and 1 out-of-bounds termination.

## What has been established

- The real Planner → Trajectory → Controller → Physics pipeline runs with
  fixed dt=0.05 s, including controller reset on repeated missions.
- True-Physics up/down tests confirm positive pitch is nose-up and
  `z_dot = u*sin(pitch) - w*cos(pitch)`. The opposite formula in the proposal
  was not adopted.
- Horizontal and narrow-passage missions meet the 0.25 m position, 0.10 m/s
  speed and 0.10 rad/s pitch-rate tolerances for the 0.5 s hold interval.
- Actual endpoint chords, not just reference paths or isolated states, are
  checked for footprint clearance and boundary containment. A tunnelling
  regression detects collision despite safe endpoints.
- Tests cover planning/trajectory failures, control validation and clipping,
  bad current/reset, invalid numerical states/clocks, timeout and goal holding.
- The runner independently checks history alignment, fixed time steps, input
  limits and success/failure termination evidence; falsely relabelled success
  is rejected by a regression test.

## Limits and next work

The ascending and descending references move in the correct direction, but
settle just outside the goal tolerance. This is not evidence of a sign reversal.
Inspect terminal-reference behavior and underactuated depth control next,
without relaxing the acceptance tolerance simply to turn the status green.

Detour execution deviates from a geometrically safe reference and violates
clearance. The current evidence does not isolate a unique cause: initial
attitude alignment, tracking transients, stopping behavior and corner policy
need controlled comparisons. Do not assume smoothing alone will fix every case.

The tighter-limit experiment clips outside the PID's own limits; no applied
actuator feedback is sent to its anti-windup mechanism. Commanded control means
the PID API output, not the hidden pre-saturation command. Vertical current
also causes collision with unchanged settings. These are recorded limitations,
not silently tuned-away failures.

Collision safety is continuous along each numerical endpoint chord, not the
true curved intra-step solution. Timestep sensitivity/substep safety remains
future validation. Clearance metrics exclude boundary distance. The squared
control integral is a mixed-unit control-effort proxy, not physical energy.

M3's minimum orchestration demonstration is established; successful multi-turn
execution and broader current robustness remain open research work. No new
planner, smoothing policy, controller tuning or 6-DOF implementation is claimed.
