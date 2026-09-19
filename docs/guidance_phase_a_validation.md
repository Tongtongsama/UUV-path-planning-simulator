# Guidance v1.0 Phase A — pure geometry acceptance

Status: completed within the pure-geometry scope. No closed-loop acceptance claimed.

## Implementation boundary

- `guidance/progress_projection.py`: normalized polyline, original index mapping,
  finite projection, sequential gates, local candidate search, explicit bounded
  monotonic progress, atomic segment commitment, arc interpolation.
- `guidance/los_guidance.py`: fixed arc lookahead, ENU pitch, constant nominal surge,
  zero q reference (not the command derivative), terminal eligibility only.
- `guidance/guidance_command.py`: immutable reference and geometric diagnostics.
- No changes to Core, Physics, Controller/PID, Navigator or frozen safety settings.
- No smoothing, observer, slowdown, obstacle queries, ROS or current compensation.

The independent ENU implementation follows the mathematical ideas and limits in the
[source audit](guidance_source_audit.md); no upstream source has been copied or ported.
Geometry tests alone cannot establish safe corner execution or physical feasibility.

## Verification

Interpreter: repository `.venv/Scripts/python.exe`; pytest **8.4.2** (inside declared `<9`).

```powershell
.\.venv\Scripts\python.exe -m pytest tests/guidance -q -p no:cacheprovider --junitxml=artifacts/guidance_phase_a/geometry.xml
.\.venv\Scripts\python.exe -m pytest -q -p no:cacheprovider --junitxml=artifacts/guidance_phase_a/regression.xml
```

Results: **77 guidance tests passed; 740 full-repository tests passed**.
The local JUnit evidence is generated under ignored `artifacts/guidance_phase_a/`;
these XML outputs are not source files or promised GitHub attachments.
The first projection-only run passed 56 tests but emitted a cache-directory permission
warning. Final runs disable pytest's cache provider; no test is excluded by that option.

| Test file | Evidence |
| --- | --- |
| `test_segment_projection.py` | Horizontal/ascent/descent, clipped mu vs raw lambda, signed errors, zero length and invalid input |
| `test_path_validation.py` | Exact duplicate runs, singleton, tiny nonzero segments, original indices, immutable geometry, arc/config/state validation |
| `test_path_progress.py` | Corners, U/zigzag/self-near paths, remote-branch exclusion, backward motion, end-plane tube gate, short-segment cap, advance/window clamp, lower-progress ties, deterministic reset |
| `test_los_guidance.py` | ENU correction signs, corner lookahead, endpoint clipping, finite zero-displacement fallback, raw angle changes, terminal eligibility and import boundary |

Topology walkthroughs assert committed progress monotonicity and sequential transitions
at every update. These are explicit fixtures, not a universal proof for every possible
self-intersecting path or arbitrary vehicle motion.

## Explicit-state usage

```python
from core import Path, Pose
from guidance import PathGeometry, ProgressState, ProjectionConfig, LOSConfig, compute_los

geometry = PathGeometry(Path((Pose(), Pose(x=2, z=1))))
actual_pose = Pose()
state = ProgressState(geometry, actual_pose)
projection_config = ProjectionConfig(
    switch_radius=0.2, forward_window=2.0,
    progress_slack=0.05, max_switches_per_step=3,
)
los_config = LOSConfig(lookahead=0.5, nominal_speed=0.3, terminal_entry_distance=0.2)
previous_pitch = None

command = compute_los(actual_pose, state, projection_config, los_config, previous_pitch)
state = command.projection.next_state
previous_pitch = command.pitch_ref
```

These numbers illustrate API usage, not frozen or tuned Phase B settings. Supply the
actual pose on every subsequent call. No simulation time, dynamics or controller is
required. Never overwrite raw projection evidence with committed progress: after
backward motion/clamping they legitimately differ, including their segment indices.

Singletons request terminal handling; at zero displacement their fallback is the last
finite pitch or actual pitch. They have no tangent, so projection residual errors are
zero by convention; distance_to_goal remains the meaningful distance. No output
declares success. The future phase adapter must own braking, capture and settling.

## Phase B boundary

Not implemented or tested here: phase ownership, startup/capture handoff, actuator
behavior, collision safety, physical tracking, currents or the sixteen-scenario comparison.
Next sequence: open horizontal, ascent, descent, isolated corner, detour, constant
current, then frozen sixteen scenarios. Keep PID and safety limits fixed and record
failures before considering any filter, slowdown or gain changes.
