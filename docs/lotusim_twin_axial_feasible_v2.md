# S2 startup coordination: feasible_v2

This is a separate guidance revision in `ros2/twin_axial_feasible_v2/`.
The vehicle remains `uuv_twin_axial_v2`. The original feasible_v1, model,
PID, allocator, current model and prior records are unchanged.

## Completed evidence (2026-11-05)

The campaign retains its `20261104_01` directory IDs; F1/F2 real runs were
completed on 2026-11-05. Every flow has P0--P4 plus a separate P0 repeat.
All 18 Python runs and all 18 LOTUSim/xdyn runs passed. All real runs launched
with GUI enabled; none were substituted by offline replay. No failed real trial
was replaced by a retry in this campaign.

Values below are worst-case over all six runs in each row, not averages.

| Flow | Backend | Passed | Max RMS (m) | Max path error (m) | Min clearance (m) |
|---|---|---:|---:|---:|---:|
| F0 no current | Python | 6/6 | 0.113047 | 0.231804 | 1.506690 |
| F0 no current | LOTUSim/xdyn | 6/6 | 0.113021 | 0.231801 | 1.506696 |
| F1 horizontal -0.05 m/s | Python | 6/6 | 0.100417 | 0.221462 | 1.504710 |
| F1 horizontal -0.05 m/s | LOTUSim/xdyn | 6/6 | 0.100486 | 0.221482 | 1.505031 |
| F2 upward +0.05 m/s | Python | 6/6 | 0.114416 | 0.222998 | 1.481583 |
| F2 upward +0.05 m/s | LOTUSim/xdyn | 6/6 | 0.114300 | 0.222028 | 1.483859 |

Original thresholds remain RMS <= 0.35 m, max error <= 0.8 m,
safety clearance >= 0.9 m, arrival within 300 s and saturation fraction <= 20%.
Real arrival ranges were F0 143.4--143.9 s, F1 142.15--142.45 s and F2
136.15--137.0 s. Maximum allocation saturation fraction was about 0.1101%;
this is distinct from pitch-reference limiting. All runs had 100% recorded
Gazebo pose correspondence coverage and passed the original final-zero,
feedback, allocation/anti-windup reproduction and clean-shutdown checks.

All three Python repeats passed. All three real P0 repeats had zero maximum
position difference, zero arrival-time difference and zero terminal-time
difference against their first run, with 2876 / 2848 / 2735 matched steps for
F0 / F1 / F2 respectively. This does not mean log files are byte-identical.

Cross-backend trajectories are not bit-identical. Over matched simulation
times, the maximum Python/xdyn position differences were 0.01997 m (F0),
0.02097 m (F1) and 0.04074 m (F2). One observed timing contribution is the
inherited `t < 2` arming comparison: xdyn reports 1.9999999999999998 at the
nominal 2 s sample, while Python records 2.0. Actual F0/P0 acceleration release
was 12.45 s versus Python 12.40 s, and F2/P0 was 8.10 s versus 8.05 s.
These are descriptive differences, not a newly relaxed acceptance threshold.
The original time-continuity checks passed. Do not treat this navigation
comparison as a new bitwise numerical-equivalence certification.

F0/P0 Python max error fell from feasible_v1's 0.953937 m to 0.231024 m.
Its RMS after acceleration release was 0.116482 m, so the improvement is not
merely dilution of full-history RMS by waiting. There is a travel-time cost:
the F0/P0 Python arrival moved from 131.95 s (failed tracking) to 143.60 s
(passed); no energy-optimality claim is made.

The full regression suite passed 1021 tests. Final preservation checks found
all 65 protected source/configuration files and all 18,875 prior artifact
files unchanged. Real run sources are identical across all flows, all shared
Python/real source files match, and all configuration dictionaries match.
The acceptance implementation is unchanged apart from newline representation
in the copied independent package.

Evidence in `artifacts/twin_axial_feasible_v2/`:

- `summary_20261104.json`: combined metrics, repeatability, source checks and evidence hashes.
- `python_20261104_01/matrix.json`: complete Python matrix and deterministic replay checks.
- `xdyn_f0_gui_20261104_01/suite.json`, corresponding `f1` and `f2`: real suites.
- `preservation_final_20261105.json`: protected-source and prior-record hashes.
- `regression_20261104.xml`: test results.
- `startup_comparison.png`: F0/P0 startup comparison, Python evidence only.

`python scripts/summarize_feasible_v2.py` regenerates the combined summary from
these fixed campaign directories and checks raw real-recording hashes. Use
the analysis commands below for independent metric/control reconstruction.

## Cause and change

In feasible_v1/F0/P0, surge acceleration started while the pitch reference
was still slewing: at 10 s actual surge was 0.268 m/s and pitch was 54.8 degrees.
Cross-track error peaked at 0.954 m at 17.4 s. The vehicle could not recover
that initial displacement immediately under the unchanged 70-degree reference
limit. This was primarily startup coordination, not a missing collision-free
geometric path or proof that the thrusters must be enlarged.

feasible_v2 keeps the exact same path generator, geometric clearance,
preview guidance, slip compensation, pitch/surge slew limits and cruise speed.
After the existing two-second arming period, it commands zero surge reference
while establishing the current compensated initial pitch. Acceleration is
released only when measured pitch is within 5 degrees of the clipped desired
pitch and absolute pitch rate is at most 0.05 rad/s continuously for 0.5 s.
It then uses the original 0.04 m/s^2 reference slew and original guidance.
This is a once-per-mission startup state, not a switch to a different PID.
Zero surge reference is not a state clamp: current-induced drift remains in
both solvers and in all safety and path-error metrics.

This is physically available in the selected synthetic twin-axial model:
opposite upper/lower thrust can create a pitch moment with zero net axial force.
Zero speed reference therefore does not mean zero actuator input. This startup
policy should not be transferred unchanged to a rudder-only vehicle that needs
forward speed for attitude authority.

All three flows use exactly the same configuration. Flow-specific startup
times arise from measured feedback, not separately tuned timers. The original
full-history metrics include startup and waiting, without discarding samples.
Additional post-launch RMS diagnostics check that improvement is not merely
RMS dilution by waiting. The original safety threshold remains 0.9 m, RMS
threshold 0.35 m, maximum path error 0.8 m and timeout 300 s.

## Run Python first (PowerShell)

```powershell
cd C:\Users\Songyuan\uuv_simulator
$run = Get-Date -Format yyyyMMdd_HHmmss
.\.venv\Scripts\python.exe -B -m ros2.twin_axial_feasible_v2.cli python-matrix --output "artifacts/twin_axial_feasible_v2/python_$run"
```

This runs F0/F1/F2, P0--P4 plus P0 repeat (18 runs), then independently
reintegrates dynamics and replays guidance/PID for verification. `matrix.json`
must report `passed`, `all_18_passed` and `cross_flow_source_identical` as true.
The all-18 requirement is an additional real-run prerequisite; it does not
relax or replace the original per-run or suite acceptance checks.

## Run LOTUSim/xdyn with GUI (NixOS)

Open PowerShell and enter `wsl -d NixOS`, then:

```bash
cd /mnt/c/Users/Songyuan/uuv_simulator
python_matrix=artifacts/twin_axial_feasible_v2/python_20261104_01
stamp=$(date +%Y%m%d_%H%M%S)
for flow in F0 F1 F2; do
  bash ros2/twin_axial_feasible_v2/run-nix.sh suite \
    --flow "$flow" --python-matrix "$python_matrix" \
    --output "artifacts/twin_axial_feasible_v2/xdyn_${flow}_${stamp}" || break
done
```

For a newly generated Python matrix, substitute its path. All 18 Python runs
and their source snapshots must match before any real trial starts. Each real
suite runs six trials sequentially. GUI is on by default; `--headless` is
available explicitly. Do not pause Gazebo or close its window during a trial:
this is a closed-loop dynamics test, not offline replay. Each trial closes its
own GUI on completion. Keep all three suites on the same source/configuration.
Existing output directories are rejected; interrupted or failed runs remain
evidence and must not be overwritten or silently replaced.

## Read-only review

```powershell
.\.venv\Scripts\python.exe -B -m ros2.twin_axial_feasible_v2.cli python-analyze artifacts/twin_axial_feasible_v2/python_20261104_01
.\.venv\Scripts\python.exe -B -m ros2.twin_axial_feasible_v2.cli analyze artifacts/twin_axial_feasible_v2/xdyn_f0_gui_20261104_01
```

Change `f0` to `f1` or `f2` for the other real suites. Review each run's
`acceptance.json`, the suite result, repeatability and recorded commands/state
and Gazebo pose correspondence. A moving GUI alone is not acceptance evidence.

`artifacts/twin_axial_feasible_v2/preservation.json` records protected source
and prior-record hash checks. `startup_diagnosis.json` contains release times
and post-launch RMS. This revision is scoped to the existing S2 geometry and
F0/F1/F2 currents; it is not validation of S1/S3 or stronger/time-varying currents.
