# v4 minimal current-aware entry

This is the minimal project entry for the frozen v4 S3/F2 current-aware candidate. It runs one case at a time. It does not run a matrix, does not alter thresholds and does not bypass probe/gate freshness.

## Usage

```bash
bash scripts/run_current_v4_minimal.sh \
  --case P2 \
  --gate-dir artifacts/gui_reproduction_gate_20261118_13 \
  --output artifacts/gui_current_v4_20261118_01/S3/F2/P2_entry
```

For P0 use the P0 gate directory.

The gate directory must contain `gate.json`, `normal/`, `repeat/` and `abort/`. The wrapper constructs the same `PilotProof` and calls the frozen v4 trial. It does not reuse an old proof silently; gate expiry still applies.

## Scope

- revision: `g12_current_v4_20261118`
- flow: F2
- cases: S3/F2/P0 or S3/F2/P2
- release: `ros2/twin_axial_g12_current_v4/release_current_04.json`
- frozen candidate record: `artifacts/twin_axial_g12_current_20261118_01/freeze_v4.json`

## Current data

The current is the frozen v4 schedule (75–135 s smooth algebraic ramp, F2 z=0.05). True and estimated current are recorded as the same schedule; there is no measured-current or estimator error in this candidate.

## Caveats

- not a full segmented-flow matrix;
- P0/P2 repeats exist, but no other initials;
- `pitch_limited=true` remains at the minimum-clearance point;
- flat engineering seabed, not chart bathymetry;
- no measured current, chart datum, tide or operational certification.
