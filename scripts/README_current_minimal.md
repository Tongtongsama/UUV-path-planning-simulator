# Unified minimal current-aware entry

`scripts/current_minimal.py` is the single project entry for the frozen S3/F2 candidates.

## Candidates

- `v4`: synthetic schedule, frozen baseline.
- `v5`: measured NOAA CO-OPS `pm0101` 0.033 m/s sample with a delayed 75–135 s onset.
- `v6`: measured two-sample NOAA time series (05:15–05:21 UTC) resampled to 0–150 s, delayed onset.

Registry: `artifacts/twin_axial_g12_current_20261118_01/candidates.json`.

## Usage

```bash
bash scripts/run_current_minimal.sh \
  --candidate v5 \
  --case P2 \
  --gate-dir artifacts/gui_reproduction_gate_20261118_24 \
  --output artifacts/gui_current_v5_ramp_20261118_01/S3/F2/P2_entry2 \
  --expected-release-sha256 78d987cc869d5459f29b8045719c7d3dd04dea8cac0171e64fd066d55401d686
```

The entry requires a fresh `gate.json` and probe directories. It does not bypass freshness, does not change thresholds and runs one case only.

## Scope

- S3/F2 P0 or P2 only.
- v4 is synthetic; v5 uses one measured uniform sample and a delayed onset.
- No full matrix and no operational claim.
