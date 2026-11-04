from pathlib import Path
from docx import Document
from docx.shared import Cm, Pt, RGBColor
from docx.enum.text import WD_ALIGN_PARAGRAPH
from docx.enum.table import WD_TABLE_ALIGNMENT
from docx.enum.section import WD_SECTION
from docx.oxml import OxmlElement
from docx.oxml.ns import qn

ROOT = Path(__file__).resolve().parents[1]
OUT_DIR = ROOT / "deliverables" / "lotusim_comparison_20261028"
OUT_DIR.mkdir(parents=True, exist_ok=True)
OUT = OUT_DIR / "LOTUSim_replay_vs_closed_loop_20261028.docx"

FONT = "Microsoft YaHei"
NAVY = RGBColor(22, 32, 51)
INK = RGBColor(37, 48, 66)
MUTED = RGBColor(95, 110, 128)
TEAL = RGBColor(15, 124, 134)
ORANGE = RGBColor(184, 90, 34)
RED = RGBColor(176, 52, 69)
GREEN = RGBColor(42, 120, 82)
BLUE = RGBColor(39, 86, 210)
WHITE = RGBColor(255, 255, 255)

doc = Document()
section = doc.sections[0]
section.page_width = Cm(21.0)
section.page_height = Cm(29.7)
section.top_margin = Cm(1.9)
section.bottom_margin = Cm(1.7)
section.left_margin = Cm(2.1)
section.right_margin = Cm(2.1)

# Base styles
normal = doc.styles["Normal"]
normal.font.name = FONT
normal.font.size = Pt(10.5)
normal.font.color.rgb = INK
normal._element.rPr.rFonts.set(qn("w:eastAsia"), FONT)
normal.paragraph_format.line_spacing = 1.35
normal.paragraph_format.space_after = Pt(4)

for name, size, color, before, after in [
    ("Heading 1", 16, TEAL, 14, 5),
    ("Heading 2", 12.5, NAVY, 10, 4),
    ("Heading 3", 11, NAVY, 7, 3),
]:
    st = doc.styles[name]
    st.font.name = FONT
    st.font.size = Pt(size)
    st.font.bold = True
    st.font.color.rgb = color
    st._element.rPr.rFonts.set(qn("w:eastAsia"), FONT)
    st.paragraph_format.space_before = Pt(before)
    st.paragraph_format.space_after = Pt(after)
    st.paragraph_format.keep_with_next = True


def shade(cell, hex_fill):
    tcPr = cell._tc.get_or_add_tcPr()
    shd = OxmlElement("w:shd")
    shd.set(qn("w:val"), "clear")
    shd.set(qn("w:color"), "auto")
    shd.set(qn("w:fill"), hex_fill)
    tcPr.append(shd)


def cell_text(cell, text, size=9.5, bold=False, color=INK, align=WD_ALIGN_PARAGRAPH.LEFT):
    cell.text = ""
    p = cell.paragraphs[0]
    p.alignment = align
    p.paragraph_format.space_after = Pt(1)
    r = p.add_run(text)
    r.font.name = FONT
    r.font.size = Pt(size)
    r.font.bold = bold
    r.font.color.rgb = color
    r._element.rPr.rFonts.set(qn("w:eastAsia"), FONT)


def add_table(headers, rows, widths, caption=None, font_size=9.3, first_col_bold=False):
    if caption:
        cp = doc.add_paragraph()
        cp.paragraph_format.space_before = Pt(4)
        cp.paragraph_format.space_after = Pt(2)
        cr = cp.add_run(caption)
        cr.font.name = FONT
        cr.font.size = Pt(9)
        cr.font.bold = True
        cr.font.color.rgb = TEAL
    table = doc.add_table(rows=1, cols=len(headers))
    table.style = "Table Grid"
    table.alignment = WD_TABLE_ALIGNMENT.CENTER
    table.autofit = False
    table.allow_autofit = False
    hdr = table.rows[0]
    trPr = hdr._tr.get_or_add_trPr()
    tblHeader = OxmlElement("w:tblHeader")
    tblHeader.set(qn("w:val"), "true")
    trPr.append(tblHeader)
    for i, h in enumerate(headers):
        cell = hdr.cells[i]
        cell.width = Cm(widths[i])
        shade(cell, "162033")
        cell_text(cell, h, font_size, True, WHITE)
    for ri, row in enumerate(rows):
        new_row = table.add_row()
        trPr = new_row._tr.get_or_add_trPr()
        cantSplit = OxmlElement("w:cantSplit")
        trPr.append(cantSplit)
        cells = new_row.cells
        for i, value in enumerate(row):
            cells[i].width = Cm(widths[i])
            shade(cells[i], "F4F7FB" if ri % 2 == 0 else "FFFFFF")
            cell_text(cells[i], str(value), font_size, first_col_bold and i == 0)
    doc.add_paragraph().paragraph_format.space_after = Pt(0)
    return table


def callout(title, body, fill="E9F1FF", border="2E68FF", title_color=BLUE):
    table = doc.add_table(rows=1, cols=1)
    table.alignment = WD_TABLE_ALIGNMENT.CENTER
    table.autofit = False
    table.allow_autofit = False
    cell = table.cell(0, 0)
    cell.width = Cm(16.5)
    shade(cell, fill)
    cell.text = ""
    p = cell.paragraphs[0]
    p.paragraph_format.space_after = Pt(1)
    r = p.add_run(title)
    r.font.name = FONT
    r.font.size = Pt(9.5)
    r.font.bold = True
    r.font.color.rgb = title_color
    r._element.rPr.rFonts.set(qn("w:eastAsia"), FONT)
    p2 = cell.add_paragraph()
    p2.paragraph_format.space_after = Pt(2)
    r2 = p2.add_run(body)
    r2.font.name = FONT
    r2.font.size = Pt(9.5)
    r2.font.color.rgb = INK
    r2._element.rPr.rFonts.set(qn("w:eastAsia"), FONT)
    doc.add_paragraph().paragraph_format.space_after = Pt(0)


def p(text, bold_lead=None, size=10.5, color=INK, space_after=4):
    par = doc.add_paragraph()
    par.paragraph_format.space_after = Pt(space_after)
    if bold_lead:
        r = par.add_run(bold_lead)
        r.font.name = FONT
        r.font.size = Pt(size)
        r.font.bold = True
        r.font.color.rgb = color
        r._element.rPr.rFonts.set(qn("w:eastAsia"), FONT)
    r = par.add_run(text)
    r.font.name = FONT
    r.font.size = Pt(size)
    r.font.color.rgb = color
    r._element.rPr.rFonts.set(qn("w:eastAsia"), FONT)
    return par


def bullet(text, lead=None):
    par = doc.add_paragraph(style="List Bullet")
    par.paragraph_format.space_after = Pt(2)
    if lead:
        r = par.add_run(lead)
        r.font.name = FONT
        r.font.size = Pt(10.3)
        r.font.bold = True
        r.font.color.rgb = TEAL
        r._element.rPr.rFonts.set(qn("w:eastAsia"), FONT)
    r = par.add_run(text)
    r.font.name = FONT
    r.font.size = Pt(10.3)
    r.font.color.rgb = INK
    r._element.rPr.rFonts.set(qn("w:eastAsia"), FONT)


def code(text):
    par = doc.add_paragraph()
    par.paragraph_format.left_indent = Cm(0.4)
    par.paragraph_format.space_after = Pt(3)
    r = par.add_run(text)
    r.font.name = "Consolas"
    r.font.size = Pt(8.8)
    r.font.color.rgb = NAVY
    r._element.rPr.rFonts.set(qn("w:eastAsia"), FONT)


# Footer
footer_p = section.footer.paragraphs[0]
footer_p.alignment = WD_ALIGN_PARAGRAPH.CENTER
fr = footer_p.add_run("UUV simulator  ·  LOTUSim / Gazebo 对比说明  ·  2026-10-28")
fr.font.name = FONT
fr.font.size = Pt(8)
fr.font.color.rgb = MUTED
fr._element.rPr.rFonts.set(qn("w:eastAsia"), FONT)

# Title page
t = doc.add_paragraph()
t.paragraph_format.space_before = Pt(48)
t.paragraph_format.space_after = Pt(6)
r = t.add_run("从历史回放到闭环证据")
r.font.name = FONT
r.font.size = Pt(26)
r.font.bold = True
r.font.color.rgb = NAVY
r._element.rPr.rFonts.set(qn("w:eastAsia"), FONT)

st = doc.add_paragraph()
st.paragraph_format.space_after = Pt(4)
r = st.add_run("两版 LOTUSim / Gazebo 可视化对比说明")
r.font.name = FONT
r.font.size = Pt(15)
r.font.bold = True
r.font.color.rgb = TEAL
r._element.rPr.rFonts.set(qn("w:eastAsia"), FONT)

p("double_baffle_detour 历史回放  vs.  恒流 S1/S2 闭环与记录回放", size=11.5, color=MUTED, space_after=18)
p("版本：2026-10-28 v1    对象：UUV simulator / LOTUSim / xdyn", size=9.5, color=MUTED, space_after=4)
p("本报告只解释两版结果的关系、LOTUSim 交互和证据边界；不修改现有代码、验收标准或冻结数据。",
  size=9.5, color=MUTED, space_after=24)

callout("一句话结论",
        "上次交付解决“在 Gazebo 里看见旧实验”；这次工作解决“用 LOTUSim+xdyn 产生可验收的新历史，并仍能回看”。"
        "两版画面相似，但证据等级、动力学归属和 LOTUSim 交互层级不同。",
        fill="E9F6F5", title_color=TEAL)
doc.add_page_break()

# 1
doc.add_heading("1. 执行摘要", level=1)
p("两版结果在 Gazebo 里都表现为“UUV 在挡板间运动”，但它们的生产链不同：")
bullet("从旧 Python 3DoF 实验读取冻结历史，用 MAS 在 Gazebo 中创建/移动实体；"
       "控制曲线来自历史，回放本身不运行动力学。", "上次交付：")
bullet("用 Python 规划/LOS/PID/分配产生命令，经 gateway 送入 LOTUSim 内的 xdyn 6DoF；"
       "新记录通过 acceptance/suite 验收，再由 recorded_replay 只读显示。", "这次工作：")
p("因此，这次新增的核心不是“更精细的模型外观”，而是“新的实验与证据”。"
  "两版都仍然只是二维 x–z 实验的三维呈现，都不是实时联合仿真，也都不能称为经验证的 LRAUV。")

add_table(
    ["维度", "上次：历史回放", "这次：闭环 + 记录回放"],
    [
        ["轨迹来源", "旧 Python 3DoF 实验的冻结 JSON/历史", "LOTUSim + xdyn 实际运行产生的新记录"],
        ["动力学归属", "Python 3DoF；Gazebo 只显示", "xdyn 6DoF 在 LOTUSim 内积分"],
        ["证据", "显示与记录；不新增实验结论", "S1/S2 各 13/13 + 里程碑封存；回放仍非验收"],
        ["LOTUSim 层级", "L2：MAS 实体回放", "L3：物理闭环；L4 实时联合仿真尚未完成"],
    ],
    [3.2, 6.6, 6.7],
    caption="表 1  两版结果的核心关系",
    first_col_bold=True,
)

doc.add_heading("2. 先定义对象：四个入口不要混为一谈", level=1)
p("仓库里同时存在“旧 renderer 接口”“MAS 实体回放”“新记录回放”和“真实闭环”。"
  "本报告中的“上次/这次”对应如下：")
add_table(
    ["入口", "它是什么", "层级"],
    [
        ["player.py", "向 /renderer_cmd、/renderer_poses 发布，不创建 Gazebo 实体；旧的外部 renderer 适配。", "L1"],
        ["gazebo_player.py", "用 /lotusim/mas_cmd_array 的 CREATE/MOVE/DELETE 回放冻结的 double_baffle_detour 历史；本报告的“上次”。", "L2"],
        ["ros2/recorded_replay", "读取新的 S1/S2 记录；MOVE 只读显示；不启动 xdyn/PID；本报告的“这次显示层”。", "L2"],
        ["twin_axial_current / _double", "Python 控制 + gateway + xdyn 6DoF + LOTUSim；产生新记录；本报告的“这次实验层”。", "L3"],
    ],
    [4.0, 10.6, 2.0],
    caption="表 2  四个容易混淆的入口",
    first_col_bold=True,
    font_size=9.0,
)
p("关键区别：`player.py` 是外部渲染话题；`gazebo_player.py` 是 MAS 实体回放；"
  "`recorded_replay` 是记录观察器；只有 `twin_axial_current/_double` 的实验层才实际运行 xdyn 闭环。")

# 2
doc.add_heading("3. 上次交付：double_baffle_detour 历史回放", level=1)
p("上次交付的价值是“可观察性”：把已经冻结的旧实验带到 Gazebo，而不是重新做实验。")
doc.add_heading("3.1 它做了什么", level=2)
bullet("挡板、边界、起点、终点和固定 P1 路径来自冻结的 double_baffle_detour 配置；"
       "障碍沿 y 加厚 0.6 m，只是把二维挡板变成可见的三维薄片。", "场景：")
bullet("P1 与 APPROACH 共用同一物理时间；APPROACH 在 98.3 s 以 SUCCESS 结束并保持终态，"
       "P1 到 120 s 为 TIMEOUT；短臂结束后不再继续运动。", "双臂：")
bullet("Python 端做插值和墙钟推进，通过 MAS 的 CREATE/MOVE/DELETE 管理实体；"
       "每批动作等待确认，并订阅 /lotusim/poses 记录 Gazebo 侧位姿。", "Gazebo 交互：")
bullet("保留原网页播放器的计划路径/实际轨迹、pitch/q、施加推力/力矩、目标距/最小净余量/碰撞标记；"
       "轨迹偏差主要通过轨迹与路径叠加观察，而不是另造一条“跟踪误差”曲线。", "网页曲线：")
bullet("记录 environment/scene、历史 states/controls、commands、gazebo_poses、events、summary 和 manifest，"
       "并可用 recording_report 生成离线图。", "证据：")
p("补充：Gazebo 里持续运动的是 UUV 本身，另有一条静态目标路径；"
  "已执行轨迹的完整叠加和曲线在网页播放器/recording_report 中，不是在 Gazebo 中额外生成一条累计尾迹。")

doc.add_heading("3.2 它没有做什么", level=2)
bullet("没有在回放中运行 PID/LOS/Planner，也没有把历史力/力矩重新施加给 Gazebo。", "没有新控制：")
bullet("没有用 Gazebo 物理或碰撞几何判定成功/碰撞；终态和碰撞截断来自原历史 exporter。", "没有新物理：")
bullet("没有新增 y 向避障；挡板加厚只用于观看。", "没有三维避障：")
bullet("简化艇形是姿态示意/展示资产，不是 LRAUV 网格；合成模型不能被称为经验证的 LRAUV。", "没有 LRAUV 验证：")
bullet("墙钟回放，不是实时状态；没有时钟锁步、速度传递、Gazebo 暂停/恢复。", "没有实时同步：")
callout("上次交付的准确定位",
        "它是一个“冻结历史 → Gazebo 实体”的可视化适配层。它能证明历史可以按原几何/原俯仰/共同时间重现，"
        "不能证明新的闭环控制、Gazebo 碰撞判定、三维避障或 LRAUV 一致性。",
        fill="FFF2E5", title_color=ORANGE)

# 3
doc.add_heading("4. 这次工作：闭环实验 + 记录回放 + 封存", level=1)
p("这次工作分为三层：实验层、显示层、证据层。三者不能混在一起说。")

doc.add_heading("4.1 实验层：恒流 S1 与缩比 S2", level=2)
p("S1 单障碍恒流：F0/F1/F2 × P0–P4，加 F1/P0、F2/P0 重复，共 13 次；"
  "S2 缩比双障碍同样 13 次。两者都使用现有 v2 模型、PID、LOS、双推进器分配和恒流修正。")
add_table(
    ["对象", "规模", "结果", "关键指标"],
    [
        ["S1 单障碍恒流", "F0/F1/F2 × P0–P4 + 重复", "13/13", "最小净距 1.62–1.74 m；最大 RMS 0.272 m"],
        ["S2 缩比双障碍", "F0/F1/F2 × P0–P4 + 重复", "13/13", "最小净距 1.54–1.69 m；最大 RMS 0.338 m；最大路径误差 0.773 m"],
    ],
    [3.1, 5.0, 2.1, 6.8],
    caption="表 3  这次新增的导航验收",
    first_col_bold=True,
)
p("重要限定：S2 是有限高挡板的缩比场景，不是全高 S2；F2 的最大路径误差 0.773 m 已接近 0.8 m 阈值；"
  "没有 S3、完整 45 次矩阵、任意流场或实艇结论。")

doc.add_heading("4.2 显示层：recorded_replay 不是生产者", level=2)
p("`recorded_replay` 读取新记录并发送 MOVE，显示：")
bullet("记录中实际加载的 SDF 挡板（单挡板或 S2 双挡板），不是另画近似障碍；", "实际几何：")
bullet("plan.json 的计划路径、起点、终点；", "任务叠加：")
bullet("F1/F2 的蓝色流向箭头和品红色随流参照球；F0 不显示；", "水流辅助：")
bullet("套件切换、暂停/倍速、终态保持；", "播放控制：")
p("它不启动 xdyn、不运行 PID/LOS，也不产生新的验收。它的输出是显示证据，不是实验证据。"
  "另外，新 S1/S2 记录目前没有接入原网页播放器；网页播放器仍保留给旧的 double_baffle_detour 数据。")

doc.add_heading("4.3 证据层：里程碑封存", level=2)
p("2026-10-28 封存了最小动力学闭环：12 个证据条目、519 个文件哈希、413 条受保护代码记录。"
  "C1/C3 为“限定范围满足”，C2/C4/C5/C6 仍为部分满足。")

# 4
doc.add_heading("5. 两条链：轨迹在哪里被积分", level=1)
p("把两版并排，最容易忽略的是“轨迹是谁积分的”。")
add_table(
    ["步骤", "上次：历史 → 显示", "这次：控制 → 物理 → 证据"],
    [
        ["1", "旧 Python 3DoF 实验产生历史", "Python 规划/LOS/PID/分配产生命令"],
        ["2", "冻结 JSON + manifest 校验", "VesselCmdArray + gateway 送入"],
        ["3", "gazebo_player 插值/墙钟推进", "xdyn 6DoF 在 LOTUSim 内积分"],
        ["4", "MAS CREATE/MOVE/DELETE", "状态/位姿回写，watchdog 检查"],
        ["5", "Gazebo 画面 + 网页曲线", "记录 → acceptance/suite → recorded_replay"],
    ],
    [1.4, 7.2, 8.4],
    caption="表 4  两条链的逐步对比",
    font_size=9.2,
)
p("橙色链的动力学在 Python 里已经结束；Gazebo 只是把结果摆出来。"
  "青色链的动力学在 LOTUSim/xdyn 中发生；Python 仍在环内计算控制，但不负责状态积分。"
  "最后的 recorded_replay 又是 MOVE 显示，不要把它误认为闭环本身。")

# 5
doc.add_heading("6. 核心差异表", level=1)
add_table(
    ["维度", "上次：double_baffle_detour", "这次：恒流 S1/S2 + 记录回放"],
    [
        ["数据来源", "旧 Python 3DoF 实验的冻结历史", "LOTUSim + xdyn 实际运行产生的新记录"],
        ["动力学积分", "Python 3DoF；Gazebo 不积分", "xdyn 6DoF 在 LOTUSim 内积分"],
        ["控制器", "历史冻结；回放中不运行", "Python 规划/LOS/PID/分配在运行中产生命令"],
        ["水流", "零流", "F0/F1/F2；恒流修正、相对速度、watchdog"],
        ["场景/轴", "1 个场景；P1 vs APPROACH 双臂", "S1 + S2 缩比；F0/F1/F2 × P0–P4"],
        ["时间/暂停", "墙钟插值；Gazebo 暂停不支持", "实际运行用 xdyn 时间 + 墙钟 watchdog；回放可暂停/倍速"],
        ["网页曲线", "保留 P1/APPROACH 轨迹/状态/控制曲线", "新 S1/S2 记录尚未接入该网页播放器"],
        ["证据等级", "显示 + 记录；不新增实验结论", "26 次验收 + 里程碑封存；回放仍非验收"],
    ],
    [3.1, 6.7, 7.2],
    caption="表 5  两版结果的核心差异",
    first_col_bold=True,
    font_size=9.0,
)
p("这两批数据不是同一实验轴：上次是 2 臂 × 1 场景的历史对比；这次是 2 场景 × 3 水流 × 5 初态 + 重复。"
  "因此不能按成功率、RMS 或到达时间把两版直接排名；能比较的是架构、交互层级和证据边界。")

# 6
doc.add_heading("7. Python → LOTUSim：哪些搬了，哪些没搬", level=1)
p("这次不是“把整个仿真搬到 LOTUSim”，而是把动力学/物理闭环搬过去；控制与任务逻辑仍在 Python。")
add_table(
    ["能力", "上次/旧链路", "这次/新链路", "状态"],
    [
        ["状态积分", "Python 3DoF 产生历史", "xdyn 6DoF 在 LOTUSim 内积分", "已迁移"],
        ["力/力矩应用与执行器", "历史值，不重新施加", "VesselCmd → gateway → xdyn；限幅/watchdog", "已迁移"],
        ["位姿/状态反馈", "Gazebo 观测只用于记录", "xdyn 状态 + Gazebo 位姿回写", "已迁移"],
        ["恒流修正", "无", "xdyn 内修正；Python 配置/预检", "已迁移（修正）"],
        ["规划/LOS/PID/分配", "Python 历史算法", "同一套 Python 算法在环内", "仍在 Python"],
        ["历史回放/插值/网页", "Python/JS 读取历史", "recorded_replay 读取新记录", "仍在 Python/JS"],
        ["碰撞判定", "原历史 exporter 截断", "任务 sweep 在 Python 验收；Gazebo 不做碰撞", "未迁移"],
    ],
    [3.2, 5.4, 7.4, 2.2],
    caption="表 6  能力迁移矩阵",
    first_col_bold=True,
    font_size=8.9,
)
p("所以最准确的说法是：旧链路中“Python 3DoF 产生轨迹”这一步，已经被 LOTUSim+xdyn 的 6DoF 闭环替代；"
  "但“Python 计算控制/任务”这一步仍然保留，而且这是有意的架构边界。")

# 7
doc.add_heading("8. LOTUSim 交互增加了什么", level=1)
add_table(
    ["层级", "交互方式", "本版状态"],
    [
        ["L0", "离线网页：纯历史数据/JS，不接 LOTUSim", "旧网页播放器"],
        ["L1", "外部 renderer 话题：player.py 发布 cmd/poses，不创建 Gazebo 实体", "旧适配层"],
        ["L2", "MAS 实体回放：gazebo_player / recorded_replay 用 CREATE/MOVE/DELETE", "上次交付 + 这次显示层"],
        ["L3", "物理闭环：xdyn 6DoF + LOTUSim physics plugin + gateway + 控制", "这次实验层"],
        ["L4", "实时联合仿真：实时 VehicleState、时钟同步、速度传递、暂停、状态所有权", "尚未完成"],
    ],
    [1.6, 11.2, 4.0],
    caption="表 7  LOTUSim 交互层级",
    font_size=9.2,
)
p("从上次到这次，增量主要发生在 L3：")
bullet("实际加载 LOTUSim 物理世界，由 xdyn 以 0.05 s 步长积分 6DoF；", "物理：")
bullet("Python 发布 VesselCmdArray，gateway 做分配、时间戳、watchdog，xdyn 返回状态；", "命令/状态：")
bullet("LOTUSim 回写位姿，任务记录同时保存 commands、transactions、navigation 和 gazebo_poses；", "回写/记录：")
bullet("恒流修正、F0/F1/F2 配置和相对速度诊断进入实验链；", "水流：")
bullet("静态场景从记录 SDF 提取实际挡板，任务路径和水流箭头/参照球由回放叠加；", "显示：")
bullet("每个运行有 acceptance.json，套件有 suite.json，最终有证据索引和封存。", "验收：")

# 8
doc.add_heading("9. C1–C6 与证据边界", level=1)
p("封存文档把 C1–C6 固定为“限定范围满足/部分满足”，没有把整个论文范围说成完成。")
add_table(
    ["标准", "状态", "说明"],
    [
        ["C1 模型/坐标/执行器", "满足（限定范围）", "最小 xdyn 桥接、v2 开环、M4–M6、F0/F1/F2 数值预检；合成模型、纵向平面"],
        ["C2 可配置障碍/水流", "部分满足", "S1 + S2 缩比已验收；全高 S2、S3、45 次矩阵未做"],
        ["C3 实际动力学闭环", "满足（限定范围）", "PID/断流/S1/S2 导航均真实 xdyn 闭环；完整故障矩阵未做"],
        ["C4 安全到达", "部分满足", "S1/S2 缩比各 13/13；S3/全高 S2/N1–N6 全矩阵未做"],
        ["C5 对照/边界", "部分满足", "F0/F1/F2 的 S1/S2 对照；更强流/敏感性/45 矩阵未做"],
        ["C6 可复现/论文", "部分满足", "依赖、记录、测试、索引较完整；论文/答辩包未完成"],
    ],
    [4.5, 3.3, 10.2],
    caption="表 8  C1–C6 当前状态",
    first_col_bold=True,
    font_size=9.0,
)
p("证据入口：`artifacts/milestones/lotusim_minimal_dynamics_20261028/evidence_index.json`；"
  "校验脚本：`scripts/verify_lotusim_minimal_dynamics_freeze.py`。封存时检查 519 个文件，"
  "六份保护清单的 413 条记录全部未变。")

# 9
doc.add_heading("10. 四个边界：不要把相似画面说过头", level=1)
bullet("上次是历史回放；这次是闭环实验 + 记录回放。最终画面不能替代 acceptance.json / suite.json。", "相同画面 ≠ 相同证据：")
bullet("挡板沿 y 加厚只是显示；规划/执行仍是 x–z 二维，没有新增 y 向避障。", "三维呈现 ≠ 三维避障：")
bullet("简化艇形/示意模型只用于展示；合成动力学模型不能被称为经验证的 LRAUV。", "艇形资产 ≠ LRAUV 验证：")
bullet("没有实时 VehicleState、时钟锁步、速度传递、暂停/恢复；L2 仍是历史墙钟。", "历史回放 ≠ 实时联合仿真：")
callout("评审时最稳妥的表述",
        "“这次在限定场景/水流/初态下完成了 LOTUSim+xdyn 闭环导航验收，并封存了证据；"
        "回放是显示层，不是实时联合仿真；S2 是缩比双障碍，不是全高 S2。”",
        fill="FDECEF", title_color=RED)

# 10
doc.add_heading("11. 从 L3 到 L4：下一步升级路线", level=1)
p("上次交付已经把 Gazebo 能承载实体回放这件事跑通；这次把 xdyn 闭环和证据链接上。"
  "下一步不是继续追求“更像”的画面，而是时间/状态所有权。")
add_table(
    ["步骤", "要做的事", "验收要点"],
    [
        ["1. 实时 VehicleState", "用实际状态流替代历史 JSON；保留来源/时序校验。", "状态频率、延迟、丢包/重复、来源可追溯"],
        ["2. 统一时钟", "xdyn、Gazebo、ROS 时间源；超时/watchdog；失步处理。", "时间单调、超时归零、暂停/恢复语义"],
        ["3. 速度/位姿传递", "状态回写；命令/观测分离；避免双发布者。", "同一实体只有一个状态/命令所有者"],
        ["4. 暂停/恢复", "暂停时不发旧命令；恢复时重置时间基准；清理只属于本次的实体。", "无旧命令残留、无实体泄漏、可重复"],
    ],
    [3.6, 8.3, 6.2],
    caption="表 9  从 L3 升级到 L4 的最小任务",
    first_col_bold=True,
    font_size=9.1,
)
p("在此之前，现有证据边界保持不变：画面可以继续用于解释，但不能替代 acceptance/suite/manifest。")

doc.add_heading("12. 结论", level=1)
p("两版结果的关系可以压缩成三句话：")
bullet("上次交付把旧实验带进 Gazebo；它证明“历史可见”，不证明新实验。", "上次：")
bullet("这次工作把控制、xdyn 6DoF、LOTUSim 和证据链接起来；S1/S2 各 13/13，并完成封存。", "这次：")
bullet("两版仍是二维实验的三维呈现；L2 回放不是 L4 实时联合仿真；S2 缩比不是全高 S2；合成模型不是 LRAUV。", "边界：")
p("如果需要对外汇报，建议把“视觉相似”放在第一页，“证据差异”放在第二页，"
  "并在最后明确列出尚未完成的 S3、全高 S2、45 次矩阵和实时状态升级。"
  "本报告只做解释，没有重跑历史实验；数字和层级来自现有记录、源代码和封存索引。")

doc.add_heading("附录：主要来源与复现入口", level=1)
code("ros2/lotusim_replay/lotusim_replay/gazebo_player.py")
code("ros2/lotusim_replay/lotusim_replay/gazebo_scene.py")
code("ros2/lotusim_replay/lotusim_replay/recording.py")
code("docs/lotusim_gazebo_replay.md")
code("docs/lotusim_frozen_replay_v1.md")
code("ros2/twin_axial_current/trial.py")
code("ros2/twin_axial_v2/gateway.py")
code("docs/lotusim_minimal_dynamics_milestone_20261028.md")
code("artifacts/milestones/lotusim_minimal_dynamics_20261028/evidence_index.json")
code("docs/lotusim_recorded_current_suite_replay.md")
code("docs/lotusim_recorded_pose_replay.md")
code("visualization/replay/player.js")


doc.save(OUT)
print(OUT)
print("paragraphs", len(doc.paragraphs), "tables", len(doc.tables))
