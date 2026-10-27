# Recorded pose replay

Read-only experiment data → verified pose interpolation → isolated Gazebo display.
No solver, controller or actuator trial is launched.

Single recording:

```bash
bash ros2/recorded_replay/run-nix.sh artifacts/twin_axial/pitch_20261026_070605
```

Recorded validation suite (`suite.json` plus one completed run per case):

```bash
bash ros2/recorded_replay/run-nix.sh artifacts/twin_axial_pid/validation_20261027_final
```

Starts paused at 0.25x. In the launching terminal, enter `p`, `pause`, `play`,
`r`, `l`, `speed 0.5` or `q`, followed by Enter. In a suite, `next`, `prev`,
`case NAME` and `list` switch the active run; the display model is shared, so
switching only changes the recorded pose source.

`--validate-only` verifies without starting Gazebo; `--prepare-only --output DIR`
writes the sanitized display world and source record without launching Gazebo.

See [中文启动及控制说明](../../docs/lotusim_recorded_pose_replay.md) and
[PID 套件可视化说明](../../docs/lotusim_recorded_pid_suite_replay.md).
