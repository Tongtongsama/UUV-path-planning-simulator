# uuv_twin_axial_v1

Independent LOTUSim/xdyn open-loop two-thruster bench. No PID or trajectory playback.

See [运行与验收说明](../../docs/lotusim_twin_axial_open_loop.md).

```bash
bash ros2/twin_axial/run-nix.sh suite --output artifacts/twin_axial/my_new_suite
```

This reuses the pinned dependency files in `ros2/minimal_xdyn/nix/` without modifying them.

No-current numerical checks (no GUI or PID):

```bash
bash ros2/twin_axial/run-nix.sh numerics --output artifacts/twin_axial_numerics/my_new_check
```

See [数值验收与运行说明](../../docs/lotusim_twin_axial_numerical_validation.md) and [实测结果及模型差异](../../docs/lotusim_twin_axial_numerical_results_20261026.md). Passing response tolerances does not establish equation equivalence.
