# v4 margin schedule and current deployment gaps

Date: 2026-11-18; timezone Asia/Singapore.

## v4 margin schedule (registered, not run)

- Revision: `g12_current_v4_20261118`
- Release: `ros2/twin_axial_g12_current_v4/release_current_04.json`
- Release SHA-256: `307786a6...5dab6eb`
- Transition: smooth algebraic gate
  - start 75 s, center 105 s, half-width 30 s, end 135 s
  - `x_current = 0.05 * 0.5 * (1 + u/sqrt(u^2+0.01))`, `u=(t-105)/30`
  - `z_current = 0.05`
- Rationale: at the v3 minimum-clearance time (~85 s), x current is only ~0.00028 m/s instead of v3 ~0.00374 m/s; this should move clearance toward the constant-F2 value.
- Status: **P0/P2/P0_repeat passed with fresh gates**. P0 repeat max position difference 0.00068 m, arrival/terminal time difference 0.0 s; P2 repeat not run. The margin effect is now measured: P0 margin 0.01648 m, P2 margin 0.01395 m; both improve over v3 by about 7.2/7.6 mm. This is still only two cases, not a full segmented-flow verdict.
- v4 does not modify constant F2, v2 failure, v3 records, G12/v6, thresholds, old five or the 54-case matrix.

## Deployment gaps: what remains before real application

### 1. Seabed / collision

- Dartsim in this build cannot create SDF heightmap collision.
- Current study accepts a flat engineering seabed at `-15 m`; it is not chart bathymetry.
- Exact shoreline is a thin-box wall along polygon edges, not a filled land collision volume.
- No grounding, tide, chart-datum or real bathymetry collision validation.
- Start/terminal points still need a land/no-go preflight.

### 2. Chart data

- NOAA ENC US5NH01M is **obsolete**; depth is sampled from DEPARE midpoints, not raw SOUNDG.
- No chart datum/tide correction, no licensed real-area chart pipeline at runtime.
- S-57 conversion is an offline GDAL step, not part of the adapter.
- Real chart licensing/provenance/versioning and a portable release package are incomplete.
- French Levant data was not used or modified.

### 3. Current model

- Current is a synthetic, spatially uniform, time-varying xdyn correction.
- True and estimated current are identical by construction; no estimator or sensor error is exercised.
- No measured current, spatial shear, waves, tide, or Copernicus/Gauss-Markov plugin validation.
- Current model is not validated against field data.
- v2 showed pitch saturation and narrow clearance under a fast ramp; v3 P0/P2 pass with only 0.0093/0.0064 m margin.

### 4. Control and verification

- v3 P0/P2 pass only two cases, not a full segmented matrix.
- Minimum clearance occurs near 85 s with `pitch_limited=true`; attitude authority remains a limiting factor.
- No full 54-case real matrix, no complete repeatability suite for current-aware revision.
- Historical 53/54, five GUI and constant-F2 results remain the frozen evidence; current-aware records are separate.

### 5. Runtime / reproducibility

- Clean-machine/empty-Nix-cache build and project migration are unverified.
- Offline dependency/Python distribution archive is not packaged.
- Historical scripts have fixed paths; portable entry and release bundle are incomplete.
- Gate/probe evidence is time-limited; every run needs fresh proof.
- No certification, sea trial, or operational safety case exists.

## Next

1. v4 probe/gate/P0/P2 are complete; P0 gate `_10`, P2 gate `_11`.
2. v4 margin improved; it is a candidate for the S3/F2 pair, but `pitch_limited=true` remains at the minimum-clearance point and no full matrix exists.
3. Do not enter the full matrix or claim operational readiness.
4. Fix seabed/current measurement/data provenance before any real-application claim.
