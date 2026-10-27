# No-current motion and closed-loop fault bench

Independent depth outer loop and fault harness, reusing the unchanged v2 model, PID adapter and allocation. No LOS or planning paths.

See [启动、验收和注入边界](../../docs/lotusim_twin_axial_motion.md).

```bash
bash ros2/twin_axial_motion/run-nix.sh suite --output artifacts/twin_axial_motion/new_suite
bash ros2/twin_axial_motion/run-nix.sh analyze artifacts/twin_axial_motion/new_suite
```
