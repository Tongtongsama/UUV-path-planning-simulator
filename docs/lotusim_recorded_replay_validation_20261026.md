# 离线回放验证 — 2026-10-26

原始记录：`artifacts/twin_axial/pitch_20261026_070605`，160个位姿，0.05–8.00秒。

- 原速无界面：verify_pitch_01，160条显示指令全部在Gazebo反馈中匹配。
- GUI交互：verify_gui_01，实际终端发送p/pause/speed 2/l/r/q，记录时间暂停于约4.06秒，后续完成7次循环。用户确认窗口中可见UUV。测试窗口由助手主动退出。
- 最终版0.5倍无界面：verify_pitch_final，318条指令、331条观测，318条匹配，最大匹配姿态误差0.02017110度，位置误差0米。
- 最终显示时钟改为0.05秒/实时因子1，避免默认展示世界高频发布。
- 独立进程检查仅有回放和Gazebo，没有xdyn-for-cs；生成world不含physics_interface，模型不含lotus_param或传感器。
- 所有显示输出在原目录之外。退出校验source_unchanged=true；本次复核原manifest全部通过。

这不是新的动力学实验，不能作为PID或模型一致性验收。GUI截图未自动采集，模型可见性由用户直接确认。运行见 [说明](lotusim_recorded_pose_replay.md)。
