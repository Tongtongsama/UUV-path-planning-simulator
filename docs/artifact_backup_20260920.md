# Artifacts backup — 2026-09-20

状态：**已完成并核验本地备份**。

- 源：`C:\Users\刘松原\uuv_simulator\artifacts`
- 备份：`D:\UUV_Backups\uuv_artifacts_20260920_v1`
- 普通文件：1,942 个；总字节数：1,079,107,756（约 1.08 GB）。
- 完整文件清单和逐文件 SHA-256：备份根目录的 `backup_manifest.json`。
- 该 manifest SHA-256：`139474ea08b6478db3c8d512a682cbb25a1520c0f3e0697ca38cd94531c26040`。
- 额外保留 `envelope_dataset_v1.json` 冻结索引的副本。

执行工具：`scripts/backup_artifacts.ps1`。目标目录必须不存在，未覆盖或删除源文件。
核验了源文件复制前/后的 SHA-256、目标文件 SHA-256 与源文件清单不变。
另外，使用备份内冻结索引重新校验四个正式 envelope stage 的 manifest、
逐步历史/ZIP/summary 等全部索引工件，以及最终 774 passed JUnit，均通过。
备份包括旧实验、部分完成的 pilot、日志和展示文件，不仅是本批 envelope。

## 唯一连接项及适用范围

`artifacts/presentations_build/node_modules` 是指向项目外运行时缓存的 Junction。
其路径、连接类型和目标已记录在 manifest 的 `junctions` 中；不递归复制外部缓存，
备份内不创建一个仍然指向原缓存的连接冒充独立依赖副本。
所有 artifacts 普通文件均已复制；重新构建该展示项目可能仍需恢复 npm/运行时依赖。

本备份是另一盘符的本地副本，不宣称 D 与 C 属于不同物理磁盘，也不等于异地备份。
没有上传 GitHub 或云端，原 `/artifacts/` 忽略规则不变。
今后有新数据时建立新的带日期目录，不修改本次备份；必要时再将该目录复制到外置
存储并重新核对 manifest。校验和索引不能替代实际文件。

## Step 2b 后新增备份（不覆盖 v1 副本）

- 目录：`D:\UUV_Backups\uuv_artifacts_20260920_v2`。
- 2,151 个普通文件，共 1,233,484,537 字节；源前后及目标逐文件 SHA-256 一致。
- backup_manifest.json SHA-256：`61ff6ea2036115df85c991f4c45cd58e017999f62ce3172c284754468ce5fc64`。
- 清单完成后额外复制 `zero_current_recovery_v2_dataset.json`，该元数据副本独立 SHA-256：
  `bf2024f099a683bd712076e60d48ce74a36f5c986f71a84fb9aa5a7c2eb037aa`。
- 已从备份内 v2 索引反向校验 pilot/full 的全部工件与 789 passed JUnit，均通过。

同样只记录外部 node_modules Junction 元数据，不复制其目标缓存；仍是本地而非异地备份。
