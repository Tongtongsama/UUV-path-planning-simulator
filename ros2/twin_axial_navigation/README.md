# S1/F0 closed-loop navigation

Independent integration of existing A*, safe geometric shortcut, LOS, allocated PID and `uuv_twin_axial_v2`. Executes actual LOTUSim/xdyn dynamics; no trajectory playback.

From the repository root in NixOS:

```bash
bash ros2/twin_axial_navigation/run-nix.sh suite \
  --output "artifacts/twin_axial_navigation/suite_$(date +%Y%m%d_%H%M%S)"
```

See [step-by-step instructions and acceptance criteria](../../docs/lotusim_twin_axial_navigation.md).
