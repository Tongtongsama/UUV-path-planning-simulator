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

## Next

1. Create the independent `g12_current_v1` release and source snapshot.
2. Add a fresh joint probe/gate for that release.
3. Run constant F2 independently.
4. Only if constant F2 passes, implement and register segmented current.
