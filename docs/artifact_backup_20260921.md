# P2 evidence backup — 2026-09-21

状态：VERIFIED，本地副本，不是异地备份。

- 来源：`C:\Users\刘松原\uuv_simulator\artifacts`
- 目标：`D:\UUV_Backups\uuv_artifacts_20260921_p2`
- 2,339 个普通文件，1,695,751,025 字节。
- 源复制前后及目标逐文件 SHA-256、文件清单一致。
- `backup_manifest.json` SHA-256：
  `c906931da38df75d65fdc3b0b20888773cc13c4fc927c684508da2f29df5f588`。
- 额外复制 P0/P1 与 P2 的 dataset index、JSON 配置，保留 docs/config 相对位置；
  逐一与源核对，并从备份索引反向校验引用的配置、manifest 与 JUnit。
- 保留 P2 几何拒绝、独立重复 history、阴性结果、诊断报告及测试证据。

使用既有 `scripts/backup_artifacts.ps1`，未覆盖旧备份或源文件。
外部 node_modules Junction 仅记录元数据，不递归复制外部缓存。
上述文件数与总字节数针对 artifacts；额外四个配置/索引副本不包含在该计数中。
没有远程上传，也不宣称 C/D 为不同物理磁盘。
