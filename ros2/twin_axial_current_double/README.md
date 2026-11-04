# S2 double-baffle constant-current navigation

Independent extension of the S1 constant-current package. The v2 model, PID,
LOS, twin-thruster allocation and `constant_current_correction` are reused
unchanged; only the two-obstacle scene and its run/acceptance wrapper are new.
The existing S1 records and frozen data are not modified.

```bash
# one-time preflight (shared with S1; current model/configuration)
bash ros2/twin_axial_current/run-nix.sh numerics --output artifacts/twin_axial_current/numerics_$(date +%Y%m%d_%H%M%S)

# 13 runs: F0/P0, F1/F2 x P0-P4, F1/P0 and F2/P0 repeats
bash ros2/twin_axial_current_double/run-nix.sh suite \
  --preflight artifacts/twin_axial_current/numerics_20261027_final \
  --output artifacts/twin_axial_current_double/navigation_$(date +%Y%m%d_%H%M%S)

# one run
bash ros2/twin_axial_current_double/run-nix.sh run --flow F2 --case P0 \
  --preflight artifacts/twin_axial_current/numerics_20261027_final \
  --output artifacts/twin_axial_current_double/F2_P0_$(date +%Y%m%d_%H%M%S)
```

Analyze without rerunning dynamics:

```bash
bash ros2/twin_axial_current_double/run-nix.sh analyze \
  artifacts/twin_axial_current_double/navigation_20261028_final
```

The scene uses two finite 3 m-tall offset baffles rather than the full-height
S2 walls from the dissertation scope. The full-height S2 geometry requires a
~10 m vertical reversal, which is outside the fixed 0.4 m/s / 60-degree / 15 N
configuration's velocity envelope; this variant keeps a two-obstacle
above/below slalom with a ~5 m reversal so the existing algorithms and gains
can be evaluated unchanged. See [the full instructions and acceptance
record](../../docs/lotusim_twin_axial_current_double.md).
