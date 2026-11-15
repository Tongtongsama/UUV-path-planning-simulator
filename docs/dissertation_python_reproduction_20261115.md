# 独立 Python 环境复现结果

日期：2026-11-15。范围：在既有 Windows 主机和未修改项目工作区上创建全新 venv，重新下载固定版本依赖，核对证据、运行离线测试、重放历史矩阵并生成54条新Python轨迹。不是全新操作系统、项目迁移或空缓存Nix构建；本轮没有启动xdyn、LOTUSim、Gazebo、ROS节点或联合探针。

## 结果

- 新环境：`C:/Users/Songyuan/uuv_repro_env_20261115_01`，Python 3.12.14，`include-system-site-packages=false`。原 `.venv` 未改动。复用同一基础Python解释器，不声称重新安装了Python发行版。
- 使用 `--no-cache-dir` 安装20个固定版本包，安装成功，pip check通过。与先前环境清单相比，Python及全部已安装包版本无差异。
- 原冻结release及170个绑定文件核对通过；历史54条Python证据在新环境中完整重放通过。
- 离线测试：1217 passed，49.11 s；未失败、未跳过。
- 新Python矩阵：54/54通过；45名义+9重复，九组重复自身验收及重复性全部通过。全部九组重复位置、到达时间和终止时间差均为0。
- 与历史 `artifacts/twin_axial_g12_matrix/python_20261110_01` 比较：54条 `states.jsonl` 逐字节一致，54条指标及验收检查一致，源码文件清单与聚合哈希一致。输出路径/运行时间/来源登记属于新记录，不能宣称所有元数据字节相同。
- 65,501个受保护旧文件哈希前后检查均通过，无修改、无缺失。全部新记录独立保存，没有覆盖旧结果。

## 原结论保持

本轮是Python复现，不改变历史GUI真实矩阵53/54、九组N6/R2通过的结果。S1/F1/P4真实失败仍保留；S3仍是名义场景。边界评价三档强流仍是Python失败、真实门禁拒绝；两档阻尼真实通过。本轮新Python54/54不能被用来替换真实失败。

## GUI条件

现有Nix环境只读 `runtime_check` 通过：冻结运行文件哈希、锁、ROS类型支持、GUI显示/驱动桥、端口12356/12357均满足检查时条件。检查没有启动仿真。

具备下一阶段采集GUI联合探针的前提；**尚不具备直接正式导航放行证据**。必须另行登记GUI复现范围，再采集normal/repeat/abort实际证据，由原门禁核验，repeat须在600s时限内。旧探针不能充当新鲜证据，端口和环境状态也须在启动时重检。新Python复现为附加对照，原门禁硬绑定的历史Python目录不可替换。

## 依赖与入口缺项

1. 20个下载wheel的来源和SHA-256已记录于 `install_report.json`，并导出 `requirements-wheel-hashes.txt`。文件针对此次Windows/Python3.12 wheel，不是跨平台完整锁；尚未构建离线wheelhouse、归档基础Python发行版或测试完全断网恢复。
2. 当前Nix检查复用了现有store/cache；未验证空缓存构建、外部依赖未来可取得性或新机器WSLg/GPU兼容性。
3. 本轮使用同一项目checkout的冻结代码，没有测试搬迁后的完整证据路径解析。旧矩阵shell脚本写死路径，边界入口要求事先存在登记/五条Python记录；可移植统一编排入口和发布包仍未完成。不得直接重跑历史shell脚本覆盖旧目录。
4. 新环境创建命令依赖当前Codex runtime的Python绝对路径；发布说明需提供可取得且版本匹配的独立Python安装来源，不能把该用户路径当通用前置条件。
5. 历史单次反馈/清理故障尚未充分归因；Python一致性和只读runtime检查不能消除这项真实链路限制。

## 记录与复现命令

- `preregistration.json`、`requirements-observed.txt`：本轮范围和固定包版本。
- `install.log`、`install_report.json`、`environment.json`、`dependency_comparison.json`：安装和环境证据。
- `tests.xml`、`offline_tests.log`：1217项结果。
- `python_reanalysis/`：历史轨迹重放，不是新实验。原汇总器的静态scope文字不改变此来源。
- `python_fresh_54/`：本轮独立54条新轨迹与验收。
- `reproduction_comparison.json`、`preservation_final.json`：逐条比较和保全。
- `runtime_readonly.json`：现有Nix只读检查，非新鲜真实门禁。

实际入口为新目录内 `run_offline.py`，使用新环境解释器执行；脚本以独占创建方式保存日志，不应再次指向本目录。下一次复现必须建立新登记/输出目录，不将失败重跑替换为通过。

## 收尾建议

C6 的独立Python依赖安装、离线测试、54条数值复现已获得证据。下一阶段可限定为GUI复现前的新鲜联合探针及事先登记的代表性GUI复现，或先完善可移植入口与发布依赖闭包；不需要调整G12/PID/场景/阈值。最终图表、发布包、完整论文及实际GUI复现仍未完成，不能宣布最低交付全部完成。

辅助审计脚本曾因默认GBK读取安装JSON失败，随后以UTF-8读取成功，发生在任何比较输出创建之前；没有重跑安装、测试或轨迹，也没有丢弃实验失败。本轮安装、测试、两次矩阵操作本身均成功。
