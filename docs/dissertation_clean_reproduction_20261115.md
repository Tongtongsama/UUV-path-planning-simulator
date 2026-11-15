# 干净环境复现清单与命令

2026-11-15，文档阶段。以下环境创建、测试、重放和仿真命令均为下一阶段操作清单，本轮未执行。已执行的仅是现有证据读取/哈希/状态代数核验。不得把本清单称作已完成干净复现。

## 0. 输入包与停止条件

- 使用独立工作目录/新环境；不要覆盖当前 `.venv`、旧 artifacts、release、模型或配置，不修改冻结代码以适配新依赖。
- 输入包必须包含 E07 release 递归引用的全部源码、锁文件和历史 Python/探针证据；v6 门禁硬绑定 `artifacts/twin_axial_g12_matrix/python_20261110_01`，不能只复制 src，也不能把新跑 Python 目录冒充这个旧目录。
- 边界 v3 还绑定边界 v2 的五条 Python 记录、其发布文件及复制来源。三个强流失败也是必要证据，不删除。
- 全矩阵旧 `run.sh`/`resume01.sh` 写死旧路径，`run_matrix.py` 也读取其同目录登记文件。禁止直接再次运行这些历史脚本。可复现入口与“重跑旧记录”不是一回事。
- 记录操作系统、WSL版本、Nix版本、GPU/WSLg、Python与包版本、全部release/lock哈希；归档实际安装日志。当前环境清单是观察值，不是带wheel哈希的跨平台锁。
- 任一源码/证据哈希、Python完整验收、运行文件哈希、端口或新鲜联合探针失败：停止真实放行，保留失败日志。不升级依赖、延长watchdog或换控制参数来取得通过。
- 12356/12357用于导航，12358用于数值检查；不得与已有实验争用，不杀端口占用进程、不关闭用户 Gazebo。

## 1. Windows 独立 Python 环境（未来执行）

历史正式 Python矩阵记录为 Python 3.12.14、NumPy 2.5.4、Shapely 2.2.0、PyYAML 6.0.3。本轮当前解释器同为3.12.14，pytest8.4.2、SciPy1.18.1、Matplotlib3.11.2；9月环境文档的3.10.9不是最终矩阵环境。尚未验证这些包在干净机器上的下载/安装可用性。

在**独立项目副本根目录**，先确认所选 `py -3.12` 的精确补丁版本。以下不安装或修改现有环境：

```powershell
py -3.12 --version
py -3.12 -m venv .venv-repro
.\.venv-repro\Scripts\python.exe -m pip install -r requirements-validation.txt -r requirements-visualization.txt "numpy==2.5.4" "shapely==2.2.0" "PyYAML==6.0.3" "pytest==8.4.2" "scipy==1.18.1" "matplotlib==3.11.2"
.\.venv-repro\Scripts\python.exe -m pip check
.\.venv-repro\Scripts\python.exe -m pip freeze
```

若指定版本不可取得或Python补丁不同，记录环境偏差，不称“精确环境复现”；不得静默用最新版替代。后续发布需整理下载可用的依赖闭包及哈希，不在本轮自动下载。

## 2. 只读证据及离线代码检查（未来执行）

```powershell
$env:PYTHONDONTWRITEBYTECODE='1'
.\.venv-repro\Scripts\python.exe -B -c "from ros2.twin_axial_g12_boundary_v3.gate import contract; print(len(contract()['files']))"
$tag = Get-Date -Format 'yyyyMMdd_HHmmss'
$out = "artifacts/reproduction_$tag"
New-Item -ItemType Directory -Path $out -ErrorAction Stop
.\.venv-repro\Scripts\python.exe -B -m pytest tests -q -p no:cacheprovider --basetemp "$out/pytest_tmp" --junitxml="$out/tests.xml"
.\.venv-repro\Scripts\python.exe -B -m ros2.twin_axial_g12_matrix.cli analyze --source artifacts/twin_axial_g12_matrix/python_20261110_01 --output "$out/python_reanalysis"
```

`analyze` 会逐步重放Python方程/PID但不启动真实后端；它不等于只读取JSON。单独保存输出，期望54/54、9/9。全量pytest包含离线模拟/合成记录测试，故也留到复现阶段。本轮没有执行这些命令。

只读复核真实组（仍会执行原验收器，不产生新轨迹）：

```powershell
.\.venv-repro\Scripts\python.exe -B -c "from pathlib import Path; from ros2.twin_axial_g12_link_v6.acceptance import analyze_suite; print(analyze_suite(Path('artifacts/twin_axial_g12_real_matrix_20261113_01/S1/F1')))"
```

预期该组 `passed=true`、`successful_initials=4`、`all_runs_passed=false`。其他八组5/5。不得把组布尔值当作逐条全部通过。

## 3. 新 Python 轨迹（未来执行，必须另行登记）

```powershell
.\.venv-repro\Scripts\python.exe -B -m ros2.twin_axial_g12_matrix.cli run --output "$out/python_fresh_54"
```

这是新实验，预计54条；成功标准按原验收和重复性，不要求字节级完全相同。保存其独立source snapshot、环境及全部失败。旧门禁硬绑定原正式Python证据，新复现证据应作为附加对照，不覆盖绑定目录或重新seal旧release。

## 4. Nix/WSL 干净构建与只读运行检查（未来执行）

选择独立 NixOS/用户环境并导入项目副本。冻结flake在 `ros2/twin_axial_navigation/nix`，LOTUSim引用固定提交 `2034754b2140006fd615bcb1c420d6a7c83f0642`，传递依赖由flake.lock决定。仅有锁不保证包/构建缓存可下载；本轮未做空缓存构建。

```bash
cd /mnt/c/Users/Songyuan/uuv_simulator_repro  # 独立副本；用其实际绝对路径
export PYTHONDONTWRITEBYTECODE=1
export UUV_XDYN_LOCK="$PWD/ros2/twin_axial_navigation/nix/flake.lock"
nix --version
sha256sum "$UUV_XDYN_LOCK"
nix develop --no-write-lock-file ./ros2/twin_axial_navigation/nix --command python3 -m ros2.twin_axial_g12_link_v6.gate runtime
```

最后一条只检查导入、显示、端口、运行文件和锁；不会启动仿真。第一次构建需要网络/缓存时不能使用 `--offline` 假装空缓存复现；缓存齐备后可追加 `--offline` 检查离线能力。不得执行 `nix flake update`。运行文件路径和SHA必须满足原release；不一致先记录与定位，不能改release使检查通过。

## 5. 后续 GUI 复现的调用契约（本轮不运行）

正式运行前另行确定复现范围与登记目录；这份清单不自动授权再次跑54条。先新Python验收，再使用同一Nix环境调用以下探针入口，每次独立目录：

```bash
# 每条命令均须置于上面的 nix develop --command 环境内。
python3 -m ros2.twin_axial_g12_link_v6.probe --transport UDPv4 --output "$REPRO/probe_normal"
python3 -m ros2.twin_axial_g12_link_v6.probe --transport UDPv4 --inject-abort --output "$REPRO/probe_abort"
python3 -m ros2.twin_axial_g12_link_v6.probe --transport UDPv4 --output "$REPRO/probe_repeat"
```

`REPRO`须预先设为新的绝对目录；故意abort预期运行失败，但必须通过 `abort_evidence` 的清理验收。不能用退出码成功代替该验收。repeat要最后刷新，启动门禁时须≤600s。探针本身会启动GUI真实链路，不能归类为只读。

实际导航使用冻结API（示例仅说明 S2/F0/P0 的准确参数，执行前需登记，不是本轮执行）：

```python
from pathlib import Path
from types import SimpleNamespace
from ros2.twin_axial_g12_link_v6.gate import pilot_evaluate
from ros2.twin_axial_g12_link_v6.trial import run_trial
base = Path('/absolute/new/reproduction/run')
proof = pilot_evaluate(base/'probe_normal', base/'probe_repeat', base/'probe_abort')
assert proof.report['ready_to_run']
args = SimpleNamespace(gate_proof=proof, gui=True, output=base/'S2/F0/P0',
                       scene='S2', flow='F0', case='P0', real_started=False)
run_trial(args)
```

运行器再次guard、检查端口且拒绝已有输出目录。它只关停自己启动的进程。正式矩阵如果需要重现，必须新登记全部45+9，不直接调用写死旧路径的历史运行脚本。重复目录名 P0_repeat，传入初态仍为P0。

边界真实入口 `python3 -m ros2.twin_axial_g12_boundary_v3.run <独立边界目录>` **不能用于空目录**：先建立与现有登记格式一致的新登记、五条对应Python记录、来源/源码绑定。旧脚本不是通用干净复现CLI，这属于发布包入口待整理项。强流Python失败时，适用门禁仍拒绝真实；不得为补表格绕过。

## 6. 复现验收与发布清单

- 记录文件齐全、哈希、原始轨迹、计划、控制器、源码、环境、关停和失败均保留。
- 统计分开：Python54/54；历史真实53/54；九组原验收/九组重复；新复现另表；强流三项参考失败/真实拒绝；两阻尼真实通过。禁止合并分母。
- M1–M7历史证据的可重分析性与新环境实际再运行是不同任务。若要重跑数值/开环，按其原入口独立登记；旧suite默认非GUI且不是v6导航入口，不要混用或当作GUI完整复现。
- 报告差异及限制，特别是历史单次反馈/清理故障、S3约16mm净距裕量、D120约5.69mm RMS裕量；不承诺复现一定零失败。
- 最后才生成最终图表、发布包文件清单和正文交叉引用。发布包应说明未包含/不可访问的WSL模型符号链接；现有Windows校验清单明确列了不可读取项，不能称完整跨平台归档已完成。

本清单中命令已对照本地CLI/API源码核对语法和作用，但未执行新环境安装、完整构建或新轨迹。可移植矩阵/边界编排入口、依赖闭包可取得性和整包恢复测试仍是C6待验收项。
