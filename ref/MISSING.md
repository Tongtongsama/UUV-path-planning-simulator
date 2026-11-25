# 拿不到的文献（30 条 + 2 条待确认）

**结论先说（2026-11-25 21:00 更新）**：`ref/` 现为 **45 / 48**。用户手工补下 27 篇后，只剩 **3 条**没有原文，且**全部有替代**：

| 缺失 | 替代 |
|---|---|
| L10-2 Latombe 1991（书） | `L08-1_lavalle2006planning.pdf` |
| L14-1 Sahoo 2019 OMAE | `L14-2_bingul2023intelligent.pdf` |
| L19-2 Oberkampf 2010（书） | `_substitutes/SUB-L19-1_benhthacker2004.pdf` |

下面保留的 §1、§5 是**历史记录**：§1 是当初 30 条订阅墙的完整清单（现已基本补齐，可作为复查用），§5 是反爬拦截的记录（7 条均已解决）。

---

## 1. 需机构订阅（30 条）

### IEEE（15 条）——占比最大

| 编号 | 文献 | DOI |
|---|---|---|
| L01-1 | Paull 2014 AUV 导航综述 | 10.1109/JOE.2013.2278891 |
| L03-1 | Healey & Lienard 1993 | 10.1109/JOE.1993.236372 |
| L04-1 | Aguiar & Hespanha 2007 | 10.1109/TAC.2007.902731 |
| L04-3 | Gao 2016 | 10.1109/CCDC.2016.7531944 |
| L05-2 | Fossen & Johansen 2006 | 10.1109/MED.2006.328749 |
| L06-2 | Kim 2013 | 10.23919/OCEANS.2013.6741024 |
| L07-3 | Kim, Fan & Bose 2018 | 10.1109/AUV.2018.8729741 |
| L08-2 | Wang 2004 | 10.1109/ICIMA.2004.1384193 |
| L09-1 | Hart 1968 | 10.1109/TSSC.1968.300136 |
| L10-1 | Elfes 1989 | 10.1109/2.30720 |
| L12-2 | Yang & Sukkarieh 2010 | 10.1109/TRO.2010.2042990 |
| L13-2 | Lekkas & Fossen 2014 | 10.1109/TCST.2014.2306774 |
| L15-2 | Khorrami & Krishnamurthy 2009 | 10.1109/ACC.2009.5160300 |
| L17-2 | Banik 2018 | 10.1109/AUV.2018.8729714 |
| L18-1 | Koenig & Howard 2004 | 10.1109/IROS.2004.1389727 |

**获取途径**：学校/机构的 IEEE Xplore 订阅（VPN 或校园网 IP）。另可先查作者主页：Fossen 组（NTNU）长期公开预印本，L13-2 可能在其个人页或 GitHub 仓库有 accepted version——这条值得先试，因为它是本项目最核心的引文源之一。

### Elsevier（6 条）

| 编号 | 文献 | DOI |
|---|---|---|
| L01-2 | Sahoo 2019 AUV 进展 | 10.1016/j.oceaneng.2019.04.011 |
| L03-2 | Lapierre & Soetanto 2007 | 10.1016/j.oceaneng.2006.10.019 |
| L05-1 | Johansen & Fossen 2013 分配综述 | 10.1016/j.automatica.2013.01.035 |
| L07-2 | Al Makdah 2019 | 10.1016/j.oceaneng.2019.05.030 |
| L13-1 | Fossen 2003 LOS | 10.1016/S1474-6670(17)37809-6 |
| L16-1 | Shen & Shi 2020 | 10.1016/j.automatica.2020.108863 |

**获取途径**：ScienceDirect 订阅。L13-1 是 IFAC Proceedings，年代较早，部分高校的会议论文集有纸本。

### Springer（3 条，均为专著）

| 编号 | 文献 | DOI |
|---|---|---|
| L02-2 | Antonelli 2018《Underwater Robots》4th ed. | 10.1007/978-3-319-77899-0 |
| L04-2 | Do & Pan 2009《Control of Ships and Underwater Vehicles》 | 10.1007/978-1-84882-730-1 |
| L10-2 | Latombe 1991《Robot Motion Planning》 | 无 DOI |

**获取途径**：SpringerLink 电子书（多数高校订阅）；或图书馆纸本。L10-2 出版于 1991 年，纸本比电子本更常见。

### 其他出版社（6 条）

| 编号 | 文献 | 出版社 | DOI |
|---|---|---|---|
| L02-1 | Fossen 2011 手册 | Wiley | 10.1002/9781119994138 |
| L13-3 | Fossen & Lekkas 2015 | Wiley | 10.1002/acs.2550 |
| L06-1 | Tarbouriech & Turner 2009 | IET | 10.1049/iet-cta:20070435 |
| L19-1 | Sargent 2013 | Taylor & Francis | 10.1057/jos.2012.20 |
| L14-1 | Sahoo 2019 OMAE | ASME | 10.1115/OMAE2019-95345 |
| L19-2 | Oberkampf & Roy 2010（书） | Cambridge UP | 10.1017/CBO9780511760396 |

**获取途径**：各出版社平台订阅；专著走图书馆。

---

## 2. 公开流传但链接未验证（2 条）

| 编号 | 文献 | 情况 | 怎么办 |
|---|---|---|---|
| L08-1 | LaValle 2006《Planning Algorithms》 | 作者在 lavalle.pl 免费公开全书 PDF，但我这边无法联网验证具体直链 | 打开 `lavalle.pl/planning/`，页面上有 "Download the book" 链接；下载后改名 `L08-1_lavalle2006planning.pdf` |
| L18-3 | Quigley 2009 ROS（ICRA workshop） | 无 DOI，无正式出版渠道；社区多处托管该 PDF | 用 Google/Bing 搜 `"ROS: an open-source Robot Operating System" pdf`，取 robotics.stanford.edu 或 ros.org 的版本；改名 `L18-3_quigley2009ros.pdf` |

这两条不是"找不到"，是"我无法在沙箱里确认链接"。你在浏览器里点一下就能确定。

---

## 3. 三条兜底渠道（按成本从低到高）

1. **馆际互借 / 文献传递**：大多数高校图书馆免费或低价，3–7 天到。适合 30 条里的期刊论文。
2. **作者索取**：给通讯作者发一封 3 行邮件（"I'm a master's student working on underactuated UUV path following, could you share a copy of your paper …"），命中率在学术界相当高，尤其对 1990 年代会议论文（L03-1、L08-2、L09-1）与老旧 IEEE 论文。
3. **机构存储库**：先用 OpenAlex 的 `any_repository_has_fulltext` 已判定为 false 的才列进上面清单；如果后续有新的绿色 OA 版本出现，重跑 `fetch_refs.ps1 -Refresh` 会重新查询。

---

## 4. 优先补哪几条

按"论文里非引不可"排序，只有 5 条：

| 顺序 | 编号 | 为什么优先 |
|---|---|---|
| 1 | **L13-2** Lekkas 2014 | 积分 LOS 是控制章的直接对照物，且上一轮已发现笔记引用出错，必须原文核对 |
| 2 | **L05-1** Johansen 2013 | "饱和=可达集"这一整段论证的地基 |
| 3 | **L02-1** Fossen 2011 | 模型章所有假设（相对速度、附加质量）的出处 |
| 4 | **L09-1** Hart 1968 | A* 最优性的原始定义（可采纳性判据在正文） |
| 5 | **L10-2** Latombe 1991 | 障碍膨胀与 clearance 的唯一硬依据 |

这 5 条拿到后，引文台账里 23 条 C 级中的 8 条可以立刻升 D 级。

---

## 5. 开放获取但被反爬拦截（浏览器点一次即可）

> **2026-11-25 20:30 更新：本节全部解决。** 用户手工补下 **L11-1**、**L15-1**、**L20-2**（原文件名缺 bibkey，已按约定改名为 `L11-1_geraerts2007creating.pdf`、`L15-1_donald1993kinodynamic.pdf`、`L20-2_peng2011reproducible.pdf`，并做过内容校验）。此前脚本已解决 L12-1、L22-2、L08-1、L18-3、L01-3。
>
> 现在 `ref/` 共 **18 / 48**，剩余 **30 条全部是订阅墙**（见 §1），与反爬无关。
>
> 下面表格保留作历史记录与备查地址。

这 7 条**不是找不到**，是脚本拿不到：出版方或机构库加了 JS 挑战（Cloudflare / Anubis）或返回 403。用浏览器打开下列地址、点下载，然后按第 4 列的名字存进 `ref/` 即可。

| 编号 | 文献 | 打开这个地址 | 存成 |
|---|---|---|---|
| L01-3 | Wynn 2014 | https://www.sciencedirect.com/science/article/pii/S0025322714000747 | `L01-3_wynn2014auvs.pdf` |
| L22-2 | Tiboni 2023 | https://doi.org/10.1016/j.robot.2023.104432 | `L22-2_tiboni2023dropo.pdf` |
| L09-2 | Dechter & Pearl 1985 | https://dl.acm.org/doi/10.1145/3828.3830 | `L09-2_dechter1985generalized.pdf` |
| L15-1 | Donald 1993 | https://dl.acm.org/doi/10.1145/174147.174150 | `L15-1_donald1993kinodynamic.pdf` |
| L12-1 | Fraichard & Scheuer 2004 | https://inria.hal.science/inria-00000009 | `L12-1_fraichard2004reeds.pdf` |
| L11-1 | Geraerts & Overmars 2007 | https://dspace.library.uu.nl/handle/1874/31850 | `L11-1_geraerts2007creating.pdf` |
| L20-2 | Peng 2011 | https://pmc.ncbi.nlm.nih.gov/articles/PMC3383002/ | `L20-2_peng2011reproducible.pdf` |

各条的实际拦截现象（2026-11-25 实测）：

| 编号 | 现象 |
|---|---|
| L01-3 | ScienceDirect 返回 403（Cloudflare），换 `/pdfft?...&download=true` 同样 403 |
| L22-2 | DOI 解析到 landing page；`Accept: application/pdf` 也被忽略 |
| L09-2 / L15-1 | ACM DL 403，带 `Referer` 也不行 |
| L12-1 | HAL 返回 Anubis 工作量证明页（`Making sure you're not a bot!`） |
| L11-1 | Utrecht DSpace 一次 502、一次纯 JS 落地页，页内无可解析的 bitstream 链接 |
| L20-2 | PMC 文章页与 Europe PMC 的 `?pdf=render`、`ptpmcrender.fcgi` 全部 403 |

> 提醒：L20-2 的 PMC 页面在浏览器里通常有一个 "PDF" 按钮；若该文只提供 HTML 全文，可退回 `L20-2_peng2011reproducible.html`（把正文页另存），不影响引用。
