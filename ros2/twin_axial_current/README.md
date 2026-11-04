# Constant-current validation

Independent v2 extension evaluated inside xdyn. Existing model/PID/LOS files are unchanged.

```bash
bash ros2/twin_axial_current/run-nix.sh numerics --output artifacts/current_preflight_01
bash ros2/twin_axial_current/run-nix.sh suite \
  --preflight artifacts/current_preflight_01 --output artifacts/current_navigation_01
```

Use new output directories. See [instructions, equations and acceptance](../../docs/lotusim_twin_axial_current.md).
