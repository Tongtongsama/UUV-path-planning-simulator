# UUV ↔ LOTUSim interaction demo v2.1 / 给陆博士的演示

## Windows + NixOS：逐步启动

1. 将压缩包解压到 `\\wsl.localhost\NixOS\home\nixos`，得到 `uuv_lotusim_demo_v2_1` 文件夹。不要覆盖旧版目录。
2. 打开 PowerShell，输入 `wsl -d NixOS`。
3. 在 NixOS 终端依次输入：

```bash
cd ~/uuv_lotusim_demo_v2_1
bash run.sh
```

4. 等待终端显示 `All UUVs created and state feedback received`，然后 Gazebo 自动打开。不要另开 LOTUSim 服务端或网页 Demo。
5. 画面中应有橙色 UUV、灰色挡板、绿色路径。Entity Tree 中 `replay_..._scene` 和 `replay_..._approach` 是两个同级模型，UUV 不在场景的 base_link 下。
6. 默认预留 15 秒加载界面，之后开始约 98 秒回放，结束停留。回到启动终端按 Ctrl+C 退出并保存记录。

v2.1 将独立演示改为“启动服务端 → 创建全部模型 → 确认状态反馈 → 启动 GUI → 回放”，避免 GUI 首次同步期间插入模型造成缺失。GUI 辅助插件的说明面板已收起。
`--start-delay 30` 可为较慢电脑多预留加载时间。若界面仍卡顿，可以使用后文的无界面记录模式；本修复不保证 GPU/远程桌面性能。
连接已有服务端模式不会重启对方的 GUI；此次启动顺序修复针对默认的独立演示模式。

面向已有 LOTUSim 的电脑。第一演示是 **在 LOTUSim 中创建 UUV 和挡板、驱动历史位姿、接收真实位姿反馈**；
第二演示是 **真实 xdyn 开环执行器脉冲及完整状态反馈**。尚未实现在线规划/PID避障闭环，不能混称。

## 推荐：使用已有安装环境

进入已 source 的 LOTUSim/ROS 环境（须能 import rclpy、lotusim_msgs），解压后 cd 到本目录：

```bash
bash run-local.sh
```

启动独立场景：橙色鱼雷状 UUV（白色艇首、尾翼、尾部螺旋桨示意），灰色挡板，绿色参考路径。
UUV 外形经过放大，仅用于展示，不是已辨识 LRAUV，也不是碰撞尺寸。
默认相机对准 x-z 场景，15 秒后开始运动。终端每两秒显示 LOTUSim 实际反馈位置和仿真时间。
全程约98.3秒，结束保持，Ctrl+C清理。`--auto-close` 自动关闭，`--compare` 同时显示蓝色 P1。

## 已经打开 LOTUSim/Gazebo：只接入，不再启动服务器

保持已有世界运行，关闭正在执行的网页 Demo。在相同 ROS_DOMAIN_ID 和 ROS 环境运行：

```bash
bash run-local.sh --connect-existing
# 若世界的 ROS 命名空间不是 /lotusim：追加 --namespace /实际命名空间
```

此模式要求已有服务 `/lotusim/mas_cmd_array`、话题 `/lotusim/poses`，以及 GUI 所需 SceneBroadcaster。
本包只创建随机前缀的实体，退出只删除这些实体，不关闭已有服务器。
此模式不改变已有相机；在 Entity Tree 选 replay_..._scene → Move To，或选 replay_..._approach 跟随艇体。
已有 LOTUSim 但 ROS Python 环境不可用时，用下面的 Nix 入口提供客户端依赖：

```bash
bash run.sh --connect-existing
```

需客户端与服务端兼容的 lotusim_msgs，不能保证任意旧版 LOTUSim 可互通。

## Nix 安装用户的一键固定环境

若平时使用 `nix run github:naval-group/LOTUSim`，可直接用：

```bash
bash run.sh
```

它复用本机 Nix store 中已有的匹配依赖；缺少的固定版本会下载，不修改已有 LOTUSim 安装。
要求 x86_64 Linux / NixOS WSL2、Nix和GUI显示支持。Windows原生/ARM未支持。
不需要复制完整项目或原始实验库，不需要运行网页 UI。

## 第二演示：证明控制输入能经动力学生成运动

独立新世界，不接入已有服务器，不发送 MOVE，不读冻结轨迹：

```bash
# 推荐先无界面验证，8秒仿真后自动验收
bash run.sh --dynamics --headless --auto-close --case surge_positive
# 可选带GUI俯仰脉冲（本阶段载具仍为原始合成测试模型）
bash run.sh --dynamics --case pitch_positive
```

已有原生环境须另外提供 xdyn-for-cs 和 Python websockets 才能用 run-local.sh 的动力学模式。
理想推力单位为N，不是RPM。状态包括位置、四元数、三轴线速度/角速度、仿真时间和实际输入。
`acceptance.json` 的 passed=true 表示方向、时间和反馈验收通过；不表示真实艇型辨识或PID已经完成。

## 验收与演示口径

所有输出在 `runs/<唯一时间编号>/`，不会覆盖旧结果。
轨迹回放的 `recording/summary.json` 应有 completed、gazebo_observed=true 和非零命令/观测数量。
`commands.csv` 是发送的位姿，`gazebo_poses.csv` 是 LOTUSim 返回的位姿；历史控制曲线不是实时控制输出。
终端反馈必须随艇体运动变化。P1 TIMEOUT / APPROACH SUCCESS 是冻结实验结论。
`OPEN_DEMO.html` 只是无需软件的历史记录备选展示，不是 LOTUSim 联动证明。

若GUI卡顿，用 `--headless --auto-close` 验证通信和记录，勿把画面流畅度当作动力学正确性。
自带 manifest 校验、原证据来源哈希和许可证。此包不发送消息、不上传实验结果。
