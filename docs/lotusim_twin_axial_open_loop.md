# uuv_twin_axial_v1：双推进器开环试验

本模块 `ros2/twin_axial` 独立于旧 `ros2/minimal_xdyn`。不读冻结轨迹、不接 PID/LOS、不调用 MOVE、不在 Python 中积分。LOTUSim → 独立网关 → xdyn RK4 → LOTUSim 是实际状态链路。

对应 [设计与验收基线](dissertation_model_acceptance_v1.md) 的首阶段：执行器分配、坐标、仿真时间、运动方向、命令失效归零。**通过本套件不表示全部 M1–M7、恒流、控制器或导航已经验收。**

## 1. 一步一步运行

打开 PowerShell：

```powershell
wsl -d NixOS
```

进入 NixOS 后：

```bash
cd /mnt/c/Users/刘松原/uuv_simulator
bash ros2/twin_axial/run-nix.sh suite \
  --output "artifacts/twin_axial/suite_$(date +%Y%m%d_%H%M%S)"
```

默认无界面，自行启动并清理本次试验的三个进程。不要另外启动网页 Demo 或动力学服务器。沿用旧模块已固定的 Nix flake/lock，仅复用依赖，不修改旧文件；第一次缺少依赖时需要下载。

套件共八个工况，再重复一次前进脉冲。零输入30秒仿真，其余每次8秒；进程启动和依赖加载另需时间。每个输出目录必须是新的，已有目录会明确拒绝覆盖。

## 2. 单项运行与离线复核

```bash
bash ros2/twin_axial/run-nix.sh run --case pitch_positive \
  --output "artifacts/twin_axial/pitch_$(date +%Y%m%d_%H%M%S)"

bash ros2/twin_axial/run-nix.sh analyze artifacts/twin_axial/你的套件目录/pitch_positive
```

可选展示：在单项 run 后加 `--gui --keep-open`。回到终端 Ctrl+C 退出并验收；未完成脉冲提前中断会标 interrupted。验收推荐 headless，避免GUI卡顿影响墙钟watchdog。

## 3. 输入和模型

- 名称 `uuv_twin_axial_v1`；两台理想可逆轴向推力器，FRD z=-0.25/+0.25 m；无直接升沉通道。
- ROS请求 JSON 为 `{"X":2.0,"M":0.0}`，单位N/Nm；网关将请求分配为 `upper(T)`、`lower(T)` 两个xdyn命令。此接口拒绝额外Z或其他输入。
- Tu=(X-4M)/2，Tl=(X+4M)/2；超过单台15 N时统一缩放两路，保持广义力方向。不是分别裁剪X/M。
- 合力 X=Tu+Tl，Z=0，俯仰力矩 M=0.25(Tl-Tu)。等效力矩施加在质心，不能重复叠加力臂。
- 模型沿用基线合成质量、附加质量、阻尼及等效恢复项，6DOF积分，不强制出平面状态归零，无流。
- 初始规划位置(2,-10)，ENU(2,0,-30)，朝东；NED位置(0,2,30)，航向pi/2。规划z=ENU z+20。FRD俯仰抬头为正，FLU的欧拉pitch符号相反。

隔离配置：ROS_DOMAIN_ID=74；GZ_PARTITION为本次试验唯一目录名；网关12356、xdyn12357。端口占用时拒绝启动，不清理其他人的进程。状态命名空间 `/uuv_twin_axial`。

## 4. 工况与预期

除零输入外，普通脉冲均为2–4 s，其余时间归零。

| case | 请求 | 预期 |
|---|---|---|
| zero | X=M=0，30 s | 水平静止，位置/姿态漂移满足基线 |
| surge_positive | X=+2 N | Tu=Tl=+1 N，ENU x增加 |
| surge_negative | X=-2 N | Tu=Tl=-1 N，ENU x减少 |
| pitch_positive | M=+0.5 Nm | Tu=-1、Tl=+1 N，抬头 |
| pitch_negative | M=-0.5 Nm | Tu=+1、Tl=-1 N，低头 |
| saturation | X=20 N，M=5 Nm | alpha=0.75，Tu=0、Tl=15 N，实际X=15 N、M=3.75 Nm |
| watchdog | 2–3 s发送X=2 N，3 s后停止发送直到结束归零 | 通信失效后自动归零 |
| stale_stamp | 3 s后仍发送X=2 N，但冻结最后有效状态时间戳 | 有流量但状态时间已过期，同样自动归零 |

失效界限：0.25 s仿真时间或1 s墙钟；下一积分请求使用零输入。仿真暂停期间没有新的积分，恢复后不会继续施加墙钟已过期命令。单元测试独立覆盖墙钟失效；实机两项故障试验验证仿真时标失效。不声称本次测试已经覆盖任意网络故障或长暂停恢复。

## 5. 如何验收

套件进程退出码0，`suite.json` 的 passed=true，各工况 `acceptance.json` passed=true。任何检查失败会返回非零，套件停在该工况并保留证据。失败不能通过改输入/删日志隐藏，重新运行用新目录。

自动检查包含：

- 输入分配证据与实际施加输入一致，推力限幅、Z=0、联合饱和指定工况。
- dt=0.05 s、状态连续、无非法求解状态、初始坐标与运动方向。
- 正/负前进首个有力积分步平均加速度接近±0.08 m/s²；俯仰接近±0.05 rad/s²（1%容差，离散首步检查，不冒称直接读取瞬时导数）。
- 30 s零输入静止；普通脉冲释放后对应轴机械能衰减。
- 独立Gazebo位姿回写匹配位置0.01 m/姿态1°，允许一个dt时间偏差，求解记录覆盖>=95%。
- 命令失效后的实际推力为零；俯仰及出平面包络。
- 完整日志哈希、进程退出、前进脉冲重复性。

**尚未完成的基线项目：**正式总机械能M4测试、步长收敛M5、Python/xdyn含水流配对M6、恒流、PID、45次导航矩阵。当前轴向能量衰减只是开环检查，不替代完整机械能验证。

## 6. 记录与复现

每个目录保存 model.yml、worlds/minimal.world、源码快照、flake.lock、进程日志、metadata.json、run_status.json、manifest.json，以及：

- commands.jsonl：发布的X/M、时间戳、序号。
- transactions.jsonl：LOTUSim输入状态、原始命令、请求广义力、未限幅推力、alpha、分配推力、实际推力/广义力、watchdog、完整求解器状态和坐标转换。
- gazebo_poses.jsonl：独立LOTUSim位姿订阅。
- acceptance.json：自动验收；suite.json另保存套件状态与重复性。

`/uuv_twin_axial/state` 为 std_msgs/String JSON 的求解器真值事务，不是含噪声传感器估计。日志采用仿真时间做分析，墙钟仅用于监控和超时。

新模块预留完整记录基础，但本阶段不添加播放器或PID。旧模型、原算法、旧配置和冻结实验保持不变。
