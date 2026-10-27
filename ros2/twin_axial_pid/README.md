# Twin axial PID step bench

Uses the unchanged `uuv_twin_axial_v2` model and gateway. Independent adapter of the existing Python inner PID with joint-allocation back-calculation. No current, LOS or path planning.

See [启动和验收说明](../../docs/lotusim_twin_axial_pid.md).

```bash
bash ros2/twin_axial_pid/run-nix.sh suite --output artifacts/twin_axial_pid/new_suite
bash ros2/twin_axial_pid/run-nix.sh analyze artifacts/twin_axial_pid/new_suite
```
