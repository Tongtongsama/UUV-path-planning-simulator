# Current-data interface: evaluation and next tasks

## Baseline

v4 is frozen as `v4_minimal_current_aware_baseline`:
- baseline lock: `artifacts/twin_axial_g12_current_20261118_01/baseline_v4.json`
- release SHA-256: `307786a6...5dab6eb`
- entry: `scripts/current_v4_minimal.py`
- scope: S3/F2 P0/P2 and repeats only.

## What was added

`ros2/twin_axial_current_data_v1/` defines a labelled current-data contract:

- `source_type`: `synthetic_schedule`, `measured`, or `estimated`;
- provenance, license, datum, time reference required;
- synthetic provenance must contain `synthetic`;
- measured provenance cannot contain `synthetic`;
- measured CSV requires strictly increasing time, finite values, no extrapolation;
- `EstimatedProvider` cannot be used as measured truth;
- `PairedCurrent` keeps `true_at()` and `estimated_at()` separate.

This module does **not** modify v4, G12/v6, thresholds or existing records. It is an interface, not a new navigation result.

## Definition of done for the minimal current-data goal

A minimal current-data function exists when all of these are true:

1. one measured uniform-current file with license, provenance, datum and time reference;
2. true current and estimated current are separate labelled values;
3. a new v5 trial consumes the interface without changing acceptance thresholds;
4. at least S3/F2/P0 and P2 pass with the new interface, each with fresh gate;
5. diagnostics record true/estimated difference and do not call synthetic data measured;
6. no full matrix is required for this minimal milestone.

## Next tasks, priority order

### P0 — integrate the interface into a new v5 diagnostic

- New `g12_current_v5` or `current_data_v1` trial, not a modification of v4.
- Truth provider feeds the model/current forcing.
- Estimate provider feeds the controller.
- Record both values and their difference in `navigation.jsonl`.
- Keep the v4 release and records untouched.

### P1 — measured uniform-current ingest

- Choose a source (ADCP/profile/Copernicus subset) with clear license.
- Store raw data, metadata, datum/time reference and checksum.
- Implement interpolation over time and reject extrapolation.
- Do not call a synthetic profile measured.

### P2 — clock and time-base alignment

- Decide simulation time vs UTC and the mapping at start.
- Handle sample latency and interpolation; record source timestamp and query time.
- Ensure no future sample is used.

### P3 — estimator/error model

- Define estimate provider and uncertainty.
- Add bias/noise/lag diagnostics separately from truth.
- Do not silently replace truth with estimate, or vice versa.

### P4 — spatial current

- Only after uniform measured current works, add position-dependent samples and interpolation.
- Address shear, vertical structure and extrapolation.

### P5 — validation and acceptance

- Keep original thresholds.
- Add current-mismatch diagnostics, not looser pass criteria.
- Re-run fresh gate/probe for the new release.
- No full matrix until the uniform measured case is stable.

### P6 — packaging

- Portable entry, offline data bundle, checksums and provenance manifest.
- Document that the flat `-15 m` seabed is an engineering boundary, not chart bathymetry.

## Current gaps

- No measured current data yet.
- No estimator/error model yet.
- No v5 trial yet.
- v4 remains synthetic and true=estimated.
- Real seabed/collision and pitch saturation remain lower-priority known gaps.

## Risk notes

- A measured file without license/provenance must be rejected.
- Simulation/UTC timezone mistakes can create silent phase errors.
- Interpolation/extrapolation must be explicit.
- A synthetic profile must never appear as measured in a report.
