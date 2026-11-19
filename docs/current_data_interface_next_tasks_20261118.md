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

## Completed in this step

- Interface separation smoke: `artifacts/current_data_interface_20261118_01/` — synthetic truth vs biased/lagged estimate; 301 samples; labels separate; max delta 0.03381 m/s.
- P1 measured uniform ingest: `artifacts/current_data_p1_20261118_01/` — NOAA CO-OPS pm0101 bin 1, public domain, 238 samples, cm/s -> m/s, measured provider smoke with 144 paired samples.
- These are **interface smokes only**; no v5 navigation and no G12 trial used the measured data.

## v5 measured-current navigation result

- Interface smoke passed (`current_v5_interface_20261118_01`): measured provider `source_type=measured`, estimate `source_type=estimated`; v5 uses the measured east/up sample.
- v5 navigation S3/F2/P0: **failed** at 29.90 s with `Swept safety clearance violated`; last recorded point clearance 0.90479 m, path error 1.67532 m, current `[0.089986, 0.0]`.
- P2 was **not run** after the P0 failure.
- The v5 gate passed; no threshold was changed.
- This is a real measured-current navigation failure, not an interface-labelling failure. The interface labels and measured sample are consistent.
- Record: `artifacts/gui_current_v5_20261118_01/S3/F2/P0`; summary: `v5_measured_navigation_failure.json`.

## v5 onset diagnosis

- Constant measured 0.090 m/s: failed at 29.9 s.
- Weak constant 0.033 m/s: failed at 85.1 s.
- Clamped 0.050 m/s: failed at 79.85 s.
- Same weak 0.033 m/s with a delayed 75–135 s ramp: **passed** P0; arrival 135.9 s; clearance 1.12759 m; margin 0.22759 m; RMS 0.27424 m; max 0.67281 m.
- Conclusion: amplitude alone is not the cause. Constant x-current from mission start is the dominant failure mechanism; a delayed/conditioned onset fixes P0.
- Do not inject measured current as a constant offset at t=0. Align the measured window/onset or ramp/condition it explicitly.
- One P0 ramp run only; P2 not run; no threshold change.
- Record: `v5_onset_diagnosis.json`.

## v5 ramp follow-up

- Ramped P0 repeat: passed; arrival 135.90 s; clearance 1.127589581714 m; max position difference **4.06e-10 m**; arrival/terminal time difference 0.0 s.
- Ramped P2 with fresh gate `_21`: passed; arrival 136.30 s; clearance 1.131469571 m; margin 0.23147 m; RMS 0.27159 m; max 0.66654 m.
- Summary: `v5_ramp_p0_p2_summary.json`; P0 repeat: `v5_ramp_repeat_summary.json`.
- P2 repeat not run; no full matrix.
- This is a **measured 0.033 m/s sample with delayed onset**, not a constant measured-current pass.

## v5 ramp P2 repeat and entry

- P2 repeat: passed; arrival 136.30 s; clearance 1.131474 m; max position difference 2.54e-05 m; arrival/terminal difference 0.0 s.
- Entry `scripts/current_v5_ramp_minimal.py` validated on S3/F2/P2 with fresh gate `_24`: passed; clearance 1.131497 m; entry-vs-frozen-P2 max position difference 0.000163 m.
- Decision: retain v5 measured-ramp candidate and entry; no full matrix.

## Unified implementation entry

- `scripts/current_minimal.py` / `run_current_minimal.sh` selects v4 or v5 by name.
- Registry: `candidates.json`.
- Both candidates remain single-case S3/F2 entries with fresh gate requirements.
- v4 is synthetic; v5 is a measured uniform sample with a delayed onset. Neither is a full current field or operational release.

## Unified entry validation

- `scripts/current_minimal.py --candidate v5 --case P0` passed with fresh gate `_25`.
- Arrival 135.90 s; clearance 1.1275895817125 m; margin 0.2275895817125 m.
- Entry vs frozen P0 max position difference 3.34e-10 m; arrival difference 0.0 s.
- A second unified P2 attempt was refused because gate `_25` expired; the earlier v5 entry P2 validation with gate `_24` remains valid.
- Record: `v5_unified_entry_validation.json`.

## v6 measured time-series P0

- New v6 uses two measured NOAA samples (05:15–05:21 UTC), linearly resampled to 0–150 s, with the same 75–135 s onset.
- Release: `ros2/twin_axial_g12_current_v6/release_current_09.json`; SHA-256 `1c9f8b39...0fa80956`.
- S3/F2/P0: **passed**; arrival 136.80 s; clearance 1.127573 m; margin 0.227573 m; RMS 0.283426 m; max 0.672856 m.
- P2: **not run**; gate `_27` expired before the second run.
- This is a real measured time-series P0, not a full matrix and not a time-series P2.
- Record: `v6_time_series_p0_summary.json`.

## v6 measured time-series P2

- P2 with fresh gate `_28`: **passed**; arrival 137.30 s; clearance 1.131451 m; margin 0.231451 m; RMS 0.281438 m; max 0.666594 m.
- v6 now has P0 and P2 passed on a measured two-sample time series; no repeats yet.
- Summary: `v6_time_series_summary.json`.
- This is still a two-sample window, not a full measured field or full matrix.

## v6 repeats and frozen candidate

- P0 repeat: passed; max position difference 1.86e-09 m; arrival/terminal difference 0.0 s.
- P2 repeat: passed; max position difference 0.0 m; arrival/terminal difference 0.0 s.
- v6 P0/P0_repeat/P2/P2_repeat all pass; frozen record `freeze_v6_time_series.json`.
- Unified entry now accepts `--candidate v6` and post-checks `source_type=measured` and onset.
- Still two measured samples, not a full field; no matrix; not operational.

## v7 three-sample measured profile

- Window 05:15–05:27 UTC, three measured samples, piecewise linear resample to 0/75/150 s, same 75–135 s onset.
- P0: **passed**; arrival 136.85 s; clearance 1.127556 m; margin 0.227556 m; RMS 0.289416 m; max 0.672905 m.
- P2: **not run**; gate `_33` expired before the second run.
- No repeats; no matrix.
- Record: `v7_three_sample_p0_summary.json`.

## v7 P2 and integration

- v7 P2 with fresh gate `_34`: **passed**; arrival 137.35 s; clearance 1.131425 m; margin 0.231425 m; RMS 0.286886 m; max 0.666656 m.
- v7 P0/P2 pass on a measured 3-sample profile; no repeats.
- Unified entry now supports `--candidate v7` and validates measured/onset metadata.
- Summary: `v7_three_sample_summary.json`.

## v7 P0 repeat

- P0 repeat passed; arrival 136.85 s; clearance 1.127555 m; margin 0.227555 m.
- Max position difference 4.41e-05 m; arrival/terminal difference 0.0 s.
- P2 repeat was not run; gate `_35` expired before it.
- v7 is not frozen; P2 repeat is still missing.

## v8 invalid attempt

- The v8 P0 record is **invalid for a 4-sample claim**: v8 trial still imported the v7 mission/gateway, and `current_schedule.json` contains the 3-sample v7 profile.
- Record preserved but marked invalid: `v8_four_sample_p0_summary.json`.
- v7 remains the frozen 3-sample candidate.
- Next: fix v8 imports to v8, reseal, fresh gate, rerun.

## v8 P0 repeat

- P0 repeat passed; arrival 136.85 s; clearance 1.127542 m; margin 0.227542 m.
- Max position difference 9.92e-05 m; arrival/terminal difference 0.0 s.
- P2 repeat not run; gate `_41` expired.
- v8 not frozen; P2 repeat is missing.

## v8 unified entry

- `--candidate v8` P0 passed with fresh gate `_43`; entry_validation confirms measured source and 05:15–05:33 window.
- v8 P2 entry not run; gate expired; v8 P2 trial/repeat passed separately.
- v8 is frozen with P0/P2 repeats and entry P0.

## v9 repeatability failure

- v9 P0/P2 pass, but P0 repeat fails the position threshold: max position difference **0.02451 m** > 0.02 m; arrival/terminal difference 0.05 s (within 0.1).
- P2 repeat not run; gate expired.
- v9 is **not** frozen and cannot be called repeatable.
- Summary: `v9_six_sample_summary.json`.

## Next tasks, priority order

### P0 — integrate the interface into a new v5 diagnostic

- Interface smoke is done; the **v5 navigation trial is not**.
- New `g12_current_v5` or `current_data_v1` trial, not a modification of v4.
- Truth provider feeds the model/current forcing.
- Estimate provider feeds the controller.
- Record both values and their difference in `navigation.jsonl`.
- Keep the v4 release and records untouched.

### P1 — measured uniform-current ingest

- Source chosen and ingested: NOAA CO-OPS pm0101 bin 1, public domain.
- Raw JSON, metadata, license, provenance and UTC time preserved.
- CSV conversion cm/s -> m/s and direction -> ENU is documented.
- Time interpolation and extrapolation rejection are implemented.
- Not yet: chart datum/tide, spatial shear, vertical current, calibrated uncertainty, v5 navigation use.

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
