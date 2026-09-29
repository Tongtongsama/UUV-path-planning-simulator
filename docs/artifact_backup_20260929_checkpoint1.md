# Checkpoint 1 完整证据备份回执

2026-09-29。状态 **VERIFIED**。本地 D 盘副本，不是异地备份；不宣称 C/D 属于不同物理设备。

## 原始证据

- 来源：`C:\Users\刘松原\uuv_simulator\artifacts`
- 目标：`D:\UUV_Backups\uuv_artifacts_20260929_checkpoint1`
- **2,486 个普通文件，2,252,684,979 字节**（不含后附源码包）。
- 源复制前后、目标逐文件 SHA-256 及文件清单全部一致；补充打包时再次校验整个目标副本。
- `backup_manifest.json` SHA-256：
  `3b54a5fa6ec5f73a3cdb7955a4e77e2492e469c82c4d1c8a36727bb296015bbc`。
- 包含 terminal 先导/完整对照 history、逐候选诊断、freeze/config/source hashes、图表、
  测试记录，以及原 P1、P2、P2b、LOS 等历史证据；没有覆盖任何原文件或旧备份。
- 唯一外部依赖目录连接 `presentations_build/node_modules` 仅记录元数据，不跟随复制外部缓存。

## 源码与反向核验

有效源码恢复包：`checkpoint_sources_verified/`。

- `repository.bundle`：全部本地 Git refs；`git bundle verify` 通过。
- `checkpoint.zip`：提交 `9169dffa16ebeba009aee120b8f96134f51d6c5f` 的 tracked 源码，
  算法仍为 `f6b6d38`，之后仅增加交付文档和备份工具。
- `p2b.zip`：独立 `c9d809e` 快照，不与 approach 混合；两个 ZIP 的 CRC 检查通过。
- 复制四组 dataset indices 及其配置，核验它们引用的 manifest、JUnit 与配置哈希。
- 进一步反向验证 terminal 先导/完整数据与图表，以及 P2b 数据/图表，共 **131 个证据文件**；
  P2b 图表 manifest 的源数据及报告脚本哈希也核对通过。
- `environment_observed.txt` 保存打包时实际 pip freeze，`verification.json` 保存 Python 身份；
  这是打包环境记录，不伪称所有历史实验使用相同锁定依赖。
- `verification.json` SHA-256：
  `7f4b9d1f7ddc2d163f40cf5f966c04d111015df4e0bc8c4ccb7d0a00a03ce2ed`。

初次附加源码打包在处理 P2b 嵌套 manifest 格式时停止；原始 artifacts 备份未受影响。
不删除或覆盖该次 `checkpoint_sources/` 中间产物。修正校验器后使用全新的
`checkpoint_sources_verified/` 完整通过；恢复时只使用后者。

## 恢复方法

不要覆盖现有工作树；选择一个尚不存在的新目录，例如：

```powershell
git clone D:\UUV_Backups\uuv_artifacts_20260929_checkpoint1\checkpoint_sources_verified\repository.bundle C:\UUV_Checkpoint1_Restore
git -C C:\UUV_Checkpoint1_Restore checkout 9169dffa16ebeba009aee120b8f96134f51d6c5f
Copy-Item -LiteralPath D:\UUV_Backups\uuv_artifacts_20260929_checkpoint1\artifacts -Destination C:\UUV_Checkpoint1_Restore\artifacts -Recurse
```

然后按 `docs/3dof_checkpoint_1.md` 安装依赖、校验并复现。上述恢复命令是操作说明，
本轮完成 bundle 验证、ZIP 检查及文件反向校验，没有宣称已执行恢复后的全部仿真。
没有上传 GitHub，也没有远程/offsite 副本。本次未重跑算法；847 passed 为上一轮已冻结证据。
