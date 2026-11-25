# _unverified/ · 内容未经验证的文件

这里只放**无法通过内容校验**、但来源看起来可信的文件，主目录 `ref/` 只保留已校验的文件。

## L09-2_dechter1985generalized.pdf

- 来源：`https://ics.uci.edu/~dechter/publications/r0.pdf`（作者主页的出版物列表第 0 条）
- 大小：2.42 MB
- 为什么没通过校验：该 PDF 是扫描件，内容流不可解压，我无法抽取任何文字来核对标题/作者（另试了 UCLA 技术报告版 `ftp.cs.ucla.edu/.../840068.pdf`，同样是扫描件，0 字符可抽）。
- 判断依据仅剩"来自作者主页 + 文件名编号最早"这一条，**不足以确认它就是 1985 年 JACM 那篇**。
- 怎么办：打开看一眼标题页；如果是 "Generalized best-fit search strategies and the optimality of A*"，把它移到 `../L09-2_dechter1985generalized.pdf` 即可；如果不是，删掉并从 ACM DL 重新下载。

## 另一处已修正的记录

本轮一开始从 FSU 数据服务器下到过一个 `AutonomousUnderwaterVehicles.pdf`（126 KB），**内容其实是 2003 年 Christopher von Alt 的 AUV workshop 报告，不是 Wynn 2014**。该文件已删除，L01-3 重新回到"待下载"状态。这条正好说明为什么每篇都要做内容校验，而不是只看文件名。
