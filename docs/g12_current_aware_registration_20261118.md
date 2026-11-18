# G12 current-aware independent revision — registration

Date: 2026-11-18; timezone Asia/Singapore.

This is a **registration**, not a navigation result. No new G12 trial was run in this step.

## Frozen base

- Controller: frozen G12/v6
- Base release: `ros2/twin_axial_g12_link_v6/release_01.json`
- Base release SHA-256: `f56d394e02818d5de00d3c4eb456365f8b471b6759feaf633a89269c7b53dadd`
- Frozen: G12, model, PID, allocator, scene, thresholds

## Independent revision

- Name: `g12_current_v1_20261118`
- New records only; existing five GUI records and 53/54 matrix are not reused as proof.
- No full real matrix.
- Acceptance uses the unchanged G12/v6 rules and thresholds; no relaxation.

## Constant F2 first

- Flow: F2, planning `current_xz = (0.0, 0.05)`
- Independent cases planned: S3/F2/P0 and S3/F2/P2
- Existing S3/F2 records are historical/frozen; they are not the new revision's proof.
- Run only after a new independent release and fresh joint probe/gate.

## Segmented current (definition only)

Proposed schedule, to be frozen before any run:

| time (s) | planning current (x,z) | note |
|---:|---|---|
| 0–60 | (0.0, 0.05) | F2 hold |
| 60–75 | (0.0, 0.05) | hold |
| 75–135 | (0.05, 0.05) | x component added |
| 135–150 | (0.05, 0.05) | hold |

Transition must be explicitly chosen (smooth or registered step) and frozen before implementation. Controller knowledge of the schedule and estimation error are separate questions; no claim yet.

## Gate and isolation

- Reuse frozen Python evidence only as prerequisite, not as a new-run result.
- Fresh joint probe/gate required before the new revision.
- Actual-trial preflight required.
- No threshold changes.
- New directories only; old records untouched.
- Flat seabed `y=-15 m` is accepted as the current-study engineering boundary, not chart bathymetry.

## Constant F2 result (new revision)

- release: `g12_current_v1_20261118`
- probes: `artifacts/gui_reproduction_gate_20261118_01` and `_02` (gates passed; P0 used `_01`, P2 used `_02` after 600 s freshness refresh)
- [P0](artifacts/gui_current_v1_20261118_01/S3/F2/P0): passed; arrival 129.35 s; clearance 0.91868 m; RMS 0.330607 m; max 0.723844 m
- [P2](artifacts/gui_current_v1_20261118_01/S3/F2/P2): passed; arrival 129.65 s; clearance 0.91617 m; RMS 0.329982 m; max 0.726253 m
- all 28 checks true; no threshold change; old records untouched.
- summary: `artifacts/gui_current_v1_20261118_01/constant_f2_summary.json`

## Segmented flow — first run failed

- Release: `g12_current_v2_20261118`, SHA-256 `ae0ffe03...5f8c9b`.
- Gate: `artifacts/gui_reproduction_gate_20261118_06/gate.json`, passed.
- S3/F2/P0 failed at **82.05 s** with `Swept safety clearance violated`.
- Last recorded point clearance is 0.9057 m; the swept segment clearance crossed 0.9 m.
- No arrival; P2 not run after the P0 failure.
- Failure record: `artifacts/gui_current_v2_20261118_03/S3/F2/P0`.
- Summary: `artifacts/twin_axial_g12_current_20261118_01/segmented_f2_failure.json`.

This is a real failed segmented run. It is preserved; the threshold was not lowered, and constant F2/old records were not changed.

Diagnosis: failure is transition-induced tracking deviation, not a controller/model current mismatch.

- At 60 s constant and segmented are nearly identical.
- After the transition, segmented path error grows faster: 70 s 0.045 vs 0.013; 80 s 0.821 vs 0.429; 82 s 0.992 vs 0.512.
- At 82 s constant clearance is 1.376 m; segmented is 0.906 m and the swept segment crosses 0.9 m.
- Controller `current_xz` follows the smooth schedule; the xdyn expression and Python schedule evaluate identically (max abs diff 0.0 at checked times).
- Therefore this is not a neighbor-geometry-only failure and not a current-schedule mismatch.

Transition re-frozen as the **smooth algebraic gate** (accepted). Freeze record: `artifacts/twin_axial_g12_current_20261118_01/freeze_v2.json`; diagnosis: `segmented_diagnosis.json`.

## Segmented flow — frozen definition

- Transition: **smoothstep** `s=3u^2-2u^3`.
- Schedule:
  - 0–60 s: `(0.0, 0.05)`;
  - 60–75 s: x smooth from `0.0` to `0.05`, z stays `0.05`;
  - 75–150 s: `(0.05, 0.05)`.
- `c_true(t)` is the current applied by the model.
- `c_ctrl(t) = c_true(t)`: controller uses the same schedule.
- `c_est(t) = c_ctrl(t)` in this phase: **no estimation error injected**; true/estimated are recorded separately by label, not by a different value.
- Status: **frozen but not implemented / not run**. No segmented record exists.
- Constant F2 and old records are frozen; do not alter them.

Freeze record: `artifacts/twin_axial_g12_current_20261118_01/freeze.json`.


## v3 slower-ramp result

New independent revision: `g12_current_v3_20261118`.

- Transition: slower smooth algebraic gate, 60–120 s, center 90 s, half-width 30 s.
- Release SHA-256: `eced8948...7598133`.
- Gate: `artifacts/gui_reproduction_gate_20261118_08/gate.json`, passed.
- S3/F2/P0: **passed** all 28 checks; arrival 128.55 s; clearance 0.90929 m (margin 0.00929 m); RMS 0.32222 m; max 0.73901 m.
- P2: **passed** with a fresh gate (`gui_reproduction_gate_20261118_09`), arrival 128.80 s; clearance 0.90636 m (margin 0.00636 m); RMS 0.32178 m; max 0.74239 m. The P0 proof was not reused.
- Both v3 P0/P2 pass, but the P2 margin is **smaller** than P0, not larger. The margin-target objective was not achieved; v3 is not robust evidence for a broader segmented-flow claim.
- Compared with v2 failure at 82 s: v3 at 82 s has clearance 1.370 m and path error 0.517 versus v2 0.917 m and 0.992. This supports the slower-ramp hypothesis, but v3 still reaches pitch limits at other times; it does not by itself prove all transition/attitude causes.
- Record: `artifacts/gui_current_v3_20261118_01/S3/F2/P0`; summary: `v3_slow_ramp_summary.json`.
- Constant F2 and v2 failure untouched.

## v4 margin result

- Revision `g12_current_v4_20261118`; release SHA-256 `307786a6...5dab6eb`.
- P0 gate `_10`, P2 gate `_11`; both passed, no proof reuse.
- P0: 129.35 s; clearance 0.91648 m; margin 0.01648 m; RMS 0.32834 m; max 0.72658 m.
- P2: 129.65 s; clearance 0.91395 m; margin 0.01395 m; RMS 0.32832 m; max 0.72902 m.
- Improvement over v3: P0 +0.00719 m, P2 +0.00758 m.
- P0 repeat: passed; clearance 0.91714 m; margin 0.01714 m; max position difference 0.00068 m; arrival/terminal time difference 0.0 s.
- Both P0/P2 still have `pitch_limited=true` at the minimum-clearance point (85.15/85.40 s).
- This is S3/F2 P0/P2 only; not a full segmented-flow verdict and not a full matrix.

## v4 frozen minimal candidate

- `freeze_v4.json` records release, four gates, P0/P2 and both repeats.
- P0 repeat: max position difference 0.000679 m; P2 repeat: 4.57e-06 m; arrival/terminal differences 0.0 s.
- Status: **frozen minimal S3/F2 candidate**, not full segmented-flow / deployment certification.
- Minimal entry: `scripts/current_v4_minimal.py`, `scripts/run_current_v4_minimal.sh`.
- P2 repeat not a full matrix; pitch saturation remains.

## v4 integration validation

- Entry `scripts/current_v4_minimal.py` ran one S3/F2/P2 case with fresh gate `_14` and expected release SHA-256.
- Result: passed all 28 checks; arrival 129.65 s; clearance 0.91393 m; margin 0.01393 m.
- Against frozen P2: max position difference 0.000319 m; arrival difference 0.0 s.
- Decision: **retain v4** as the minimal S3/F2 current-aware candidate; entry path/output validated.
- This is still one integration run, not a matrix. Pitch saturation, flat engineering seabed, synthetic true=estimated current remain caveats.
- Record: `v4_integration_validation.json`.

## Next

1. Create the independent `g12_current_v1` release and source snapshot.
2. Add a fresh joint probe/gate for that release.
3. Run constant F2 independently.
4. Only if constant F2 passes, implement and register segmented current.
