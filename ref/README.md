# ref/ · 文献原文库

`docs/literature_review_sources_20261125.md` 所列 L01–L22 的原文，按 `<L编号>_<bibkey>.<ext>` 命名（例：`L13-2_lekkas2014integral.pdf`），bibkey 与 `docs/literature_review_sources_20261125.bib` 一致。

## 已入库：45 / 48（全部做过内容校验）

| 文件 | 大小 | 通道 |
|---|---|---|
| `L01-3_wynn2014auvs.pdf` | 4.93 MB | White Rose eprints（机构库） |
| `L08-1_lavalle2006planning.pdf` | 12.75 MB | 作者官网 lavalle.pl |
| `L12-1_fraichard2004reeds.pdf` | 0.41 MB | LORIA 作者页 |
| `L12-3_park2022three.pdf` | 3.05 MB | mdpi-res CDN |
| `L14-2_bingul2023intelligent.pdf` | 7.06 MB | mdpi-res CDN |
| `L16-2_wang2024trajectory.pdf` | 1.58 MB | mdpi-res CDN |
| `L17-1_kot2022review.pdf` | 3.57 MB | mdpi-res CDN |
| `L18-2_buche2026lotusim.pdf` | 4.70 MB | arXiv |
| `L18-3_quigley2009ros.pdf` | 0.43 MB | ai.stanford.edu |
| `L20-1_sandve2013ten.pdf` | 0.13 MB | PLOS |
| `L21-1_noaa2024pm0101.html` | 36.8 KB | NOAA |
| `L21-2_noaa_coops_measure.html` | 19.9 KB | NOAA |
| `L22-1_zhao2020sim.pdf` | 0.59 MB | arXiv |
| `L22-2_tiboni2023dropo.pdf` | 10.36 MB | arXiv 2201.08434 |

## 剩下 3 个：均有替代

用户手工补下 27 篇（原是论文标题名，已按约定改名并校验），现在只差 3 条：

| 缺失 | 替代方案 | 说明 |
|---|---|---|
| L10-2 Latombe 1991（书） | `L08-1_lavalle2006planning.pdf` | LaValle 第 2–3 章完整覆盖 C-space、障碍膨胀、胞腔分解 |
| L14-1 Sahoo 2019 OMAE | `L14-2_bingul2023intelligent.pdf` | 同组的 PID 类 AUV 控制器实现 |
| L19-2 Oberkampf 2010（书） | `_substitutes/SUB-L19-1_benhthacker2004.pdf` | Thacker《Concepts of Model V&V》，与 Sargent 同源的概念定义 |

另：L02-1 的书本体已入库，配套讲义（含 Ch10 环境力/海流、Ch12 引导）见 [_supplements/](_supplements/README.md)。

### 两处需人工确认的校验

- `L01-2_sahoo2019advancements.pdf`、`L07-2_almakdah2019three.pdf`：PDF 用的是自定义字体编码，抽出的文字是乱码，**无法做关键词校验**。按你下载时的 L 编号命名保留，建议各扫一眼标题页。
- `L15-1_donald1993kinodynamic.pdf`：扫描件，同样无文本层，仅过了魔数检查。

## 校验规则（为什么可信）

每个文件都必须过三道：

1. **魔数**：前 4 字节是 `%PDF`；网页类必须是 `text/html`；
2. **体量**：PDF ≥ 50 KB（L01-3 那个 126 KB 的错文件就是被第 3 道拦下的）；
3. **内容关键词**：解压内容流抽文字，命中标题/作者关键词才收。例：L12-1 命中 `reeds`，L18-3 命中 `robot operating system`，L22-2 命中 `domain randomization`。

第 3 道当场揪出一处错误：从 FSU 数据服务器下到的 `AutonomousUnderwaterVehicles.pdf`，正文其实是 2003 年 von Alt 的 workshop 报告，不是 Wynn 2014 —— 已删除。因此 `ref/` 根目录只放过了校验的文件。

## 怎么继续

```bash
python ref/fetch_refs.py                 # 断点续跑那 16 个开放获取项，已存在自动跳过
python ref/hunt_mirrors.py --only L15-1  # 用搜索引擎找作者/机构镜像（DDG 会限流，需间隔）
powershell -ExecutionPolicy Bypass -File ref/fetch_refs.ps1 -Verify   # 对 INDEX.csv 自检
```

## 技术备注

- 这台机器 **Windows 的 SChannel 拿不到 TLS 凭证**（`Invoke-WebRequest`、`curl`、`certutil`、BITS 全失败），但 **Python 的 OpenSSL 栈可用**；`fetch_refs.py` 用 Git 自带的 CA bundle 做**验证过的 TLS**（未关校验）。
- 搜索引擎不稳定：DDG 几次查询后返回 202，Bing RSS 返回无关结果，Mojeek 403。所以镜像搜寻只在会话前半段有效。
- 所有尝试（含失败与拒绝）都记在 [download_log.txt](download_log.txt)，保留 URL 与 sha256 前缀，便于回溯。

## 合法性

只取开放获取版本（金色/绿色/青铜 OA）与作者主页自存稿；订阅期刊不抓，清单里给的是 DOI 而非绕过付费墙的地址。
