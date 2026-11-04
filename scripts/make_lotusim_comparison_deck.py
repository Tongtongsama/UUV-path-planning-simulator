from pathlib import Path
from pptx import Presentation
from pptx.util import Inches, Pt
from pptx.dml.color import RGBColor
from pptx.enum.text import PP_ALIGN, MSO_ANCHOR
from pptx.enum.shapes import MSO_SHAPE
from pptx.enum.text import MSO_AUTO_SIZE

ROOT = Path(__file__).resolve().parents[1]
OUT_DIR = ROOT / "deliverables" / "lotusim_comparison_20261028"
OUT_DIR.mkdir(parents=True, exist_ok=True)
OUT = OUT_DIR / "LOTUSim_replay_vs_closed_loop_20261028.pptx"

FONT = "Microsoft YaHei"
NAVY = RGBColor(22, 32, 51)
INK = RGBColor(37, 48, 66)
MUTED = RGBColor(95, 110, 128)
LIGHT = RGBColor(246, 248, 251)
WHITE = RGBColor(255, 255, 255)
BLUE = RGBColor(46, 104, 255)
TEAL = RGBColor(15, 124, 134)
ORANGE = RGBColor(224, 122, 45)
GREEN = RGBColor(55, 145, 100)
RED = RGBColor(190, 67, 82)
LINE = RGBColor(218, 225, 234)
PALE_BLUE = RGBColor(234, 240, 255)
PALE_ORANGE = RGBColor(255, 242, 229)
PALE_TEAL = RGBColor(229, 246, 246)
PALE_GREEN = RGBColor(232, 246, 238)
PALE_RED = RGBColor(253, 236, 239)

prs = Presentation()
prs.slide_width = Inches(13.333)
prs.slide_height = Inches(7.5)
prs.core_properties.title = "两版 LOTUSim Gazebo 可视化对比"
prs.core_properties.subject = "double_baffle_detour 历史回放 vs. 恒流 S1/S2 闭环与记录回放"
prs.core_properties.author = "UUV simulator"


def set_bg(slide, color=LIGHT):
    slide.background.fill.solid()
    slide.background.fill.fore_color.rgb = color


def txt(slide, x, y, w, h, text, size=14, color=INK, bold=False,
        align=PP_ALIGN.LEFT, anchor=MSO_ANCHOR.TOP, name=FONT, margins=(0, 0, 0, 0)):
    box = slide.shapes.add_textbox(Inches(x), Inches(y), Inches(w), Inches(h))
    tf = box.text_frame
    tf.clear()
    tf.word_wrap = True
    tf.margin_left = Inches(margins[0])
    tf.margin_right = Inches(margins[1])
    tf.margin_top = Inches(margins[2])
    tf.margin_bottom = Inches(margins[3])
    tf.vertical_anchor = anchor
    p = tf.paragraphs[0]
    p.alignment = align
    r = p.add_run()
    r.text = text
    r.font.name = name
    r.font.size = Pt(size)
    r.font.bold = bold
    r.font.color.rgb = color
    return box


def add_para(tf, text, size=12, color=INK, bold=False, space_before=0,
             space_after=0, bullet=None, align=PP_ALIGN.LEFT):
    p = tf.add_paragraph()
    p.alignment = align
    p.space_before = Pt(space_before)
    p.space_after = Pt(space_after)
    if bullet:
        text = bullet + " " + text
    r = p.add_run()
    r.text = text
    r.font.name = FONT
    r.font.size = Pt(size)
    r.font.bold = bold
    r.font.color.rgb = color
    return p


def box(slide, x, y, w, h, fill=WHITE, line=LINE, radius=True,
        line_width=1.0):
    shape = slide.shapes.add_shape(
        MSO_SHAPE.ROUNDED_RECTANGLE if radius else MSO_SHAPE.RECTANGLE,
        Inches(x), Inches(y), Inches(w), Inches(h))
    shape.fill.solid()
    shape.fill.fore_color.rgb = fill
    shape.line.color.rgb = line
    shape.line.width = Pt(line_width)
    shape.shadow.inherit = False
    return shape


def pill(slide, x, y, w, h, text, fill, color=WHITE, size=10.5, bold=True):
    s = box(slide, x, y, w, h, fill=fill, line=fill)
    tf = s.text_frame
    tf.clear()
    tf.word_wrap = True
    tf.margin_left = Inches(0.05)
    tf.margin_right = Inches(0.05)
    tf.margin_top = Inches(0.0)
    tf.margin_bottom = Inches(0.0)
    tf.vertical_anchor = MSO_ANCHOR.MIDDLE
    p = tf.paragraphs[0]
    p.alignment = PP_ALIGN.CENTER
    r = p.add_run()
    r.text = text
    r.font.name = FONT
    r.font.size = Pt(size)
    r.font.bold = bold
    r.font.color.rgb = color
    return s


def title(slide, kicker, heading, sub=None):
    set_bg(slide)
    slide.shapes.add_shape(MSO_SHAPE.RECTANGLE, Inches(0), Inches(0),
                           prs.slide_width, Inches(0.09)).fill.solid()
    slide.shapes[-1].fill.fore_color.rgb = TEAL
    slide.shapes[-1].line.fill.background()
    txt(slide, 0.6, 0.30, 9.5, 0.28, kicker, 10.5, TEAL, True)
    txt(slide, 0.6, 0.55, 12.0, 0.55, heading, 25, NAVY, True)
    if sub:
        txt(slide, 0.6, 1.10, 12.1, 0.35, sub, 12.5, MUTED)


def footer(slide, page, note="来源：gazebo_player.py · recording.py · twin_axial_current/trial.py · 2026-10-28 里程碑封存"):
    txt(slide, 0.6, 7.16, 10.9, 0.18, note, 8.5, MUTED)
    txt(slide, 12.15, 7.16, 0.65, 0.18, f"{page:02d}", 9, MUTED, True,
        PP_ALIGN.RIGHT)


def notes(slide, text):
    slide.notes_slide.notes_text_frame.text = text


def arrow(slide, x, y, w=0.22, h=0.14, color=MUTED):
    s = slide.shapes.add_shape(MSO_SHAPE.RIGHT_ARROW, Inches(x), Inches(y),
                               Inches(w), Inches(h))
    s.fill.solid()
    s.fill.fore_color.rgb = color
    s.line.fill.background()
    s.shadow.inherit = False
    return s


def make_table(slide, rows, x, y, w, col_widths, row_h=0.42, header=True,
               font_size=10.5, first_col_bold=False, fills=None):
    nrows = len(rows)
    ncols = len(rows[0])
    shape = slide.shapes.add_table(nrows, ncols, Inches(x), Inches(y),
                                   Inches(w), Inches(row_h * nrows))
    table = shape.table
    for i, width in enumerate(col_widths):
        table.columns[i].width = Inches(width)
    for r, row in enumerate(rows):
        table.rows[r].height = Inches(row_h)
        for c, value in enumerate(row):
            cell = table.cell(r, c)
            cell.margin_left = Inches(0.06)
            cell.margin_right = Inches(0.05)
            cell.margin_top = Inches(0.035)
            cell.margin_bottom = Inches(0.025)
            cell.vertical_anchor = MSO_ANCHOR.MIDDLE
            cell.fill.solid()
            if header and r == 0:
                cell.fill.fore_color.rgb = NAVY
            elif fills and (r, c) in fills:
                cell.fill.fore_color.rgb = fills[(r, c)]
            else:
                cell.fill.fore_color.rgb = WHITE if r % 2 else RGBColor(249, 251, 253)
            tf = cell.text_frame
            tf.clear()
            tf.word_wrap = True
            p = tf.paragraphs[0]
            p.alignment = PP_ALIGN.LEFT
            run = p.add_run()
            run.text = str(value)
            run.font.name = FONT
            run.font.size = Pt(font_size)
            run.font.bold = (header and r == 0) or (first_col_bold and c == 0)
            run.font.color.rgb = WHITE if (header and r == 0) else INK
    return table


# ---------------------------------------------------------------- slide 1
s = prs.slides.add_slide(prs.slide_layouts[6])
set_bg(s, NAVY)
s.shapes.add_shape(MSO_SHAPE.RECTANGLE, Inches(0), Inches(0), Inches(0.12),
                   prs.slide_height).fill.solid()
s.shapes[-1].fill.fore_color.rgb = TEAL
s.shapes[-1].line.fill.background()
txt(s, 0.75, 0.65, 11.8, 0.35, "UUV SIMULATOR  ·  LOTUSim / Gazebo 对比说明", 11, RGBColor(142, 205, 208), True)
txt(s, 0.75, 1.30, 11.8, 1.30, "看起来相同的 Gazebo 回放，\n实际差在哪？", 35, WHITE, True)
txt(s, 0.78, 2.95, 11.1, 0.75,
    "double_baffle_detour 历史回放  vs.  恒流 S1/S2 闭环与记录回放",
    17, RGBColor(200, 214, 232))
pill(s, 0.78, 4.18, 2.15, 0.40, "上次：冻结历史", ORANGE, WHITE, 11)
pill(s, 3.08, 4.18, 2.55, 0.40, "这次：真实 xdyn 闭环", TEAL, WHITE, 11)
pill(s, 5.78, 4.18, 2.55, 0.40, "共同点：二维实验三维呈现", BLUE, WHITE, 11)
txt(s, 0.78, 5.20, 11.3, 0.95,
    "先说明：这是“可视化与交互层级”的对比，不是把两批不同场景、不同初始条件的数值结果直接排名。",
    12.5, RGBColor(178, 194, 214))
txt(s, 0.78, 6.62, 10.5, 0.25, "2026-10-28  ·  对比范围：历史回放、LOTUSim 交互、证据链与下一步", 9.5, RGBColor(142, 160, 184))
notes(s, "本页先定调：两版在 Gazebo 里都像‘UUV 在挡板间运动’，但一个是历史回放，一个是新产生的闭环实验。后面逐层拆。")


# ---------------------------------------------------------------- slide 2
s = prs.slides.add_slide(prs.slide_layouts[6])
title(s, "30 秒结论", "相同的是画面，不同的是证据等级",
      "把“谁产生轨迹、谁推进时间、谁拥有物理状态”分开看，差异就清楚了。")
cards = [
    ("上次交付", "冻结历史回放", ORANGE, PALE_ORANGE,
     ["输入：旧 Python 3DoF 实验历史", "Gazebo：MAS 创建/移动/删除实体", "Python：插值 + 墙钟 + 记录", "结论：只能证明“看得见历史”"]),
    ("这次工作", "闭环实验 + 记录回放", TEAL, PALE_TEAL,
     ["输入：LOTUSim + xdyn 新运行", "动力学：xdyn 6DoF 在 LOTUSim 内积分", "控制：Python LOS/PID/分配仍在环内", "结论：有 13/13 + 13/13 验收与封存"]),
    ("共同边界", "视觉相似，但不能混称", BLUE, PALE_BLUE,
     ["二维 x-z 实验的三维呈现", "都不是实时联合仿真", "都不是经验证的 LRAUV", "画面不能替代 acceptance.json"]),
]
for i, (kicker, head, accent, pale, bullets) in enumerate(cards):
    x = 0.6 + i * 4.16
    box(s, x, 1.55, 3.86, 4.05, WHITE)
    s.shapes.add_shape(MSO_SHAPE.RECTANGLE, Inches(x), Inches(1.55),
                       Inches(0.08), Inches(4.05)).fill.solid()
    s.shapes[-1].fill.fore_color.rgb = accent
    s.shapes[-1].line.fill.background()
    txt(s, x + 0.25, 1.78, 3.35, 0.25, kicker, 10.5, accent, True)
    txt(s, x + 0.25, 2.10, 3.35, 0.55, head, 18, NAVY, True)
    tf = txt(s, x + 0.25, 2.90, 3.30, 2.75, "", 11.5).text_frame
    tf.clear()
    for j, b in enumerate(bullets):
        p = tf.paragraphs[0] if j == 0 else tf.add_paragraph()
        p.space_after = Pt(8)
        p.level = 0
        r = p.add_run()
        r.text = "• " + b
        r.font.name = FONT
        r.font.size = Pt(11.5)
        r.font.color.rgb = INK
    pill(s, x + 0.25, 4.75, 1.15, 0.27, "L2" if i == 0 else ("L3" if i == 1 else "边界"), accent, WHITE, 8.5)
box(s, 0.6, 6.30, 12.10, 0.55, NAVY, NAVY)
txt(s, 0.82, 6.43, 11.65, 0.28, "一句话：这次新增的不是“更像 Gazebo 的模型”，而是“由 LOTUSim+xdyn 产生、可验收、可封存的新轨迹”。", 12.5, WHITE, True)
footer(s, 2)
notes(s, "三个卡片：上次=历史回放；这次=闭环实验+记录回放；共同点=二维实验的三维呈现，不能夸成实时或 LRAUV。")


# ---------------------------------------------------------------- slide 3
s = prs.slides.add_slide(prs.slide_layouts[6])
title(s, "先排除混淆", "四个入口，不是同一件事",
      "仓库里同时存在旧 renderer、MAS 回放、套件回放和真实闭环；本 PPT 的“上次/这次”如下。")
entries = [
    ("旧接口", "player.py", "向 /renderer_cmd、/renderer_poses 发布；不会在 Gazebo 创建实体。", "L1 · 外部渲染接口", ORANGE),
    ("上次交付", "gazebo_player.py", "MAS CREATE/MOVE/DELETE；从冻结场景和 P1/APPROACH 历史回放。", "L2 · MAS 实体回放", ORANGE),
    ("这次的显示", "ros2/recorded_replay", "读取新的 S1/S2 记录；MOVE 显示；不启动 xdyn/PID。", "L2 · 记录回放", BLUE),
    ("这次的实验", "twin_axial_current / _double", "Python 控制 + gateway + xdyn 6DoF + LOTUSim；产生新记录。", "L3 · 真实动力学闭环", TEAL),
]
for i, (tag, name, desc, level, color) in enumerate(entries):
    col, row = i % 2, i // 2
    x = 0.6 + col * 6.18
    y = 1.60 + row * 2.15
    box(s, x, y, 5.85, 1.85, WHITE)
    pill(s, x + 0.22, y + 0.20, 1.02, 0.27, tag, color, WHITE, 8.5)
    txt(s, x + 1.38, y + 0.17, 3.90, 0.30, name, 14, NAVY, True, name="Consolas")
    txt(s, x + 0.22, y + 0.62, 5.35, 0.55, desc, 10.8, INK)
    txt(s, x + 0.22, y + 1.35, 5.35, 0.25, level, 9.8, color, True)
box(s, 0.6, 6.05, 12.10, 0.62, PALE_BLUE, LINE)
txt(s, 0.82, 6.18, 11.65, 0.32, "本 PPT：上次 = gazebo_player 那版；这次 = twin_axial_current/double 的闭环结果 + recorded_replay 的新记录回放。", 11.5, NAVY, True)
footer(s, 3)
notes(s, "先区分四个入口。否则会把‘记录回放’误当成‘闭环’，或把旧 renderer 当成 Gazebo 实体。")


# ---------------------------------------------------------------- slide 4
s = prs.slides.add_slide(prs.slide_layouts[6])
title(s, "两条链", "上次：历史 → 显示；这次：控制 → 物理 → 证据",
      "关键差异不在最后一张画面，而在轨迹是在哪里被积分出来的。")
# top row
txt(s, 0.6, 1.55, 1.0, 0.28, "上次", 13, ORANGE, True)
top = [
    ("旧 Python 3DoF\n实验历史", PALE_ORANGE, ORANGE),
    ("冻结 JSON\n+ manifest", PALE_ORANGE, ORANGE),
    ("gazebo_player\n插值/墙钟", PALE_ORANGE, ORANGE),
    ("MAS CREATE\nMOVE/DELETE", PALE_ORANGE, ORANGE),
    ("Gazebo 画面\n+ 网页曲线", PALE_ORANGE, ORANGE),
]
x0 = 1.45
for i, (label, fill, accent) in enumerate(top):
    x = x0 + i * 2.32
    box(s, x, 1.82, 2.08, 0.95, fill, accent)
    txt(s, x + 0.10, 1.98, 1.88, 0.60, label, 9.8, NAVY, True, PP_ALIGN.CENTER, MSO_ANCHOR.MIDDLE)
    if i < len(top) - 1:
        arrow(s, x + 2.10, 2.23, 0.18, 0.13, ORANGE)
# bottom row
txt(s, 0.6, 3.55, 1.0, 0.28, "这次", 13, TEAL, True)
bottom = [
    ("Python 规划\nLOS/PID/分配", PALE_TEAL, TEAL),
    ("VesselCmdArray\n+ gateway", PALE_TEAL, TEAL),
    ("LOTUSim + xdyn\n6DoF 积分", PALE_TEAL, TEAL),
    ("位姿/状态\n回写", PALE_TEAL, TEAL),
    ("记录 + recorded\nreplay 回放", PALE_TEAL, TEAL),
]
for i, (label, fill, accent) in enumerate(bottom):
    x = x0 + i * 2.32
    box(s, x, 3.82, 2.08, 1.05, fill, accent)
    txt(s, x + 0.10, 3.97, 1.88, 0.72, label, 9.4, NAVY, True, PP_ALIGN.CENTER, MSO_ANCHOR.MIDDLE)
    if i < len(bottom) - 1:
        arrow(s, x + 2.10, 4.25, 0.18, 0.13, TEAL)
box(s, 0.6, 5.45, 12.10, 0.70, PALE_BLUE, LINE)
txt(s, 0.82, 5.58, 11.65, 0.44,
    "橙色链：轨迹来自旧 Python 3DoF，LOTUSim 只负责显示。\n青色链：轨迹由 xdyn 6DoF 在 LOTUSim 内产生；Python 仍负责控制，但不再负责积分。",
    11.2, NAVY)
box(s, 0.6, 6.35, 12.10, 0.45, WHITE, LINE)
txt(s, 0.82, 6.45, 11.65, 0.25,
    "注意：recorded_replay 仍用 MOVE 只读显示；它是“新记录的观察器”，不是新记录的生产者。",
    10.2, MUTED)
footer(s, 4)
notes(s, "把两条链并排。上次轨迹来自 Python 3DoF 历史；这次是 xdyn 在 LOTUSim 内积分。最后回放层仍可能是 MOVE。")


# ---------------------------------------------------------------- slide 5
s = prs.slides.add_slide(prs.slide_layouts[6])
title(s, "上次交付", "double_baffle_detour：把旧实验“搬进”Gazebo",
      "价值在可观察性，不在新增动力学或新成功率。")
left = [
    ("场景", "double_baffle_detour；挡板/边界/起点/终点/固定 P1 路径来自冻结配置"),
    ("双臂", "P1 与 APPROACH 共用同一物理时间；APPROACH 98.3 s SUCCESS，P1 120 s TIMEOUT"),
    ("显示", "MAS CREATE/MOVE/DELETE；等待每批确认；订阅 /lotusim/poses 记录观测"),
    ("三维化", "障碍沿 y 加厚 0.6 m，仅用于观看；二维 x-z 实验没有新增三维避障"),
    ("资产", "简化艇形/姿态示意，不是 LRAUV 网格，也不声称 LRAUV 已验证"),
    ("网页", "保留原播放器的轨迹、pitch/q、推力/力矩、目标距/净余量/碰撞标记"),
]
tf = txt(s, 0.6, 1.50, 6.30, 4.6, "", 10.3).text_frame
for i, (k, v) in enumerate(left):
    p = tf.paragraphs[0] if i == 0 else tf.add_paragraph()
    p.space_after = Pt(9)
    r = p.add_run()
    r.text = "• " + k + "："
    r.font.name = FONT
    r.font.size = Pt(10.3)
    r.font.bold = True
    r.font.color.rgb = ORANGE
    r2 = p.add_run()
    r2.text = v
    r2.font.name = FONT
    r2.font.size = Pt(10.3)
    r2.font.color.rgb = INK
box(s, 7.00, 1.50, 5.70, 2.05, PALE_ORANGE, ORANGE)
txt(s, 7.22, 1.68, 5.25, 0.28, "它证明了", 12, ORANGE, True)
txt(s, 7.22, 2.02, 5.25, 1.30,
    "冻结历史可以按原几何、原俯仰、双臂共同时间在 Gazebo 中重现；终态/碰撞截断保持在历史终止状态。",
    11.2, INK)
box(s, 7.00, 3.78, 5.70, 2.32, WHITE, LINE)
txt(s, 7.22, 3.96, 5.25, 0.28, "它没有证明", 12, RED, True)
items = ["新的闭环控制实验", "Gazebo 物理或碰撞判定", "三维避障能力", "LRAUV 模型/实艇一致性", "实时时钟同步、速度传递或暂停机制"]
tf = txt(s, 7.22, 4.28, 5.20, 1.65, "", 10.5).text_frame
for j, item in enumerate(items):
    p = tf.paragraphs[0] if j == 0 else tf.add_paragraph()
    p.space_after = Pt(3)
    r = p.add_run(); r.text = "– " + item
    r.font.name = FONT; r.font.size = Pt(10.5); r.font.color.rgb = INK
footer(s, 5)
notes(s, "上次交付做的是历史回放：MAS 实体、共同物理时间、原几何、网页曲线。它不能证明新的控制实验或 Gazebo 物理。")


# ---------------------------------------------------------------- slide 6
s = prs.slides.add_slide(prs.slide_layouts[6])
title(s, "这次新增", "从“看历史”到“产生可验收的新历史”",
      "四块新增：场景/水流、真实动力学、回放叠加、证据封存。")
quad = [
    ("01  恒流 S1", "F0/F1/F2 × P0–P4 + 重复；13/13；最小净距 1.62–1.74 m；最大 RMS 0.272 m", TEAL, PALE_TEAL),
    ("02  缩比 S2", "双障碍 S2；13/13；最小净距 1.54–1.69 m；最大 RMS 0.338 m；最大路径误差 0.773 m", TEAL, PALE_TEAL),
    ("03  新记录回放", "从记录加载实际挡板 + 计划路径/起终点；蓝色流向箭头；品红随流参照球；MOVE 显示", BLUE, PALE_BLUE),
    ("04  封存与索引", "12 个证据条目、519 个文件哈希、413 条保护记录；C1–C6 状态与剩余任务", ORANGE, PALE_ORANGE),
]
for i, (head, body, accent, pale) in enumerate(quad):
    col, row = i % 2, i // 2
    x = 0.6 + col * 6.18
    y = 1.55 + row * 2.18
    box(s, x, y, 5.85, 1.95, WHITE)
    s.shapes.add_shape(MSO_SHAPE.RECTANGLE, Inches(x), Inches(y), Inches(0.08), Inches(1.95)).fill.solid()
    s.shapes[-1].fill.fore_color.rgb = accent
    s.shapes[-1].line.fill.background()
    txt(s, x + 0.25, y + 0.20, 5.25, 0.35, head, 14, accent, True)
    txt(s, x + 0.25, y + 0.66, 5.25, 1.05, body, 10.6, INK)
box(s, 0.6, 6.10, 12.10, 0.64, NAVY, NAVY)
txt(s, 0.82, 6.23, 11.65, 0.30,
    "边界：S2 是有限挡板缩比版，不是全高 S2；没有 S3、完整 45 次矩阵、实时 VehicleState 或全故障矩阵。",
    11.2, WHITE, True)
footer(s, 6)
notes(s, "这次新增四块：S1 恒流导航、S2 缩比双障碍、记录回放叠加、封存索引。数字只适用于各自限定场景。")


# ---------------------------------------------------------------- slide 7
s = prs.slides.add_slide(prs.slide_layouts[6])
title(s, "核心差异", "同一个 Gazebo 窗口，背后是两种证据",
      "比较维度：数据、动力学、时间、场景、显示、结论。")
rows = [
    ["维度", "上次：double_baffle_detour 回放", "这次：恒流 S1/S2 闭环 + 记录回放"],
    ["数据来源", "旧 Python 3DoF 实验的冻结 JSON/历史", "LOTUSim + xdyn 实际运行产生的记录"],
    ["谁积分动力学", "Python 3DoF；Gazebo 只显示", "xdyn 6DoF 在 LOTUSim 内积分"],
    ["控制器", "历史冻结，回放中不运行", "Python 规划/LOS/PID/分配在运行中产生命令"],
    ["水流", "零流", "F0/F1/F2；恒流修正、相对速度、watchdog"],
    ["场景/轴", "1 个场景；P1 vs APPROACH 双臂", "S1 + S2 缩比；F0/F1/F2 × P0–P4"],
    ["时间/暂停", "墙钟插值；Gazebo 暂停不支持", "实际运行用 xdyn 时间 + 墙钟 watchdog；回放可暂停/倍速"],
    ["网页曲线", "保留 P1/APPROACH 的轨迹/状态/控制曲线", "新 S1/S2 记录尚未接入该网页播放器"],
    ["证据等级", "显示 + 记录；不新增实验结论", "26 次验收 + 里程碑封存；回放仍非验收"],
]
make_table(s, rows, 0.55, 1.50, 12.20, [1.35, 5.20, 5.65], row_h=0.47, font_size=9.8, first_col_bold=True)
box(s, 0.55, 6.02, 12.20, 0.48, PALE_BLUE, LINE)
txt(s, 0.78, 6.12, 11.75, 0.26, "核心区别：上次的证据停在“显示”；这次多出 acceptance.json 与封存索引。", 10.5, NAVY, True)
footer(s, 7)
notes(s, "核心表。最后一行是关键：上次是显示，这次有验收和封存；但这次的 recorded_replay 本身仍是显示。")


# ---------------------------------------------------------------- slide 8
s = prs.slides.add_slide(prs.slide_layouts[6])
title(s, "迁移矩阵", "哪些从 Python 搬到了 LOTUSim，哪些仍留在 Python",
      "不是“整个仿真搬到 LOTUSim”；搬走的是动力学/物理闭环。")
rows = [
    ["能力", "上次/旧链路", "这次/新链路", "状态"],
    ["3DoF/6DoF 状态积分", "Python 3DoF 产生历史", "xdyn 6DoF 在 LOTUSim 内积分", "已迁移"],
    ["力/力矩应用与执行器", "历史值，不重新施加", "VesselCmd → gateway → xdyn；限幅/watchdog", "已迁移"],
    ["位姿/状态反馈", "Gazebo 观测仅用于记录", "xdyn 状态 + Gazebo 位姿回写", "已迁移"],
    ["恒流修正", "无", "xdyn 内修正；Python 配置/预检", "已迁移（修正）"],
    ["规划 / LOS / PID / 分配", "Python 历史算法", "同一套 Python 算法在环内", "仍在 Python"],
    ["历史回放/插值/网页", "Python/JS 读取历史", "recorded_replay 读取新记录", "仍在 Python/JS"],
]
fills = {}
for r in range(1, 5):
    fills[(r, 3)] = PALE_TEAL
for r in range(5, 7):
    fills[(r, 3)] = PALE_ORANGE
make_table(s, rows, 0.55, 1.50, 12.20, [2.20, 3.25, 4.85, 1.90], row_h=0.55, font_size=10.0, first_col_bold=True, fills=fills)
box(s, 0.55, 5.75, 12.20, 0.70, PALE_BLUE, LINE)
txt(s, 0.78, 5.87, 11.75, 0.45,
    "结论：规划/LOS/PID 仍是 Python；被搬到 LOTUSim+xdyn 的是状态积分、命令应用、物理反馈和恒流修正。",
    11.5, NAVY, True)
footer(s, 8)
notes(s, "迁移矩阵：四类已迁移，规划/LOS/PID 仍 Python，回放/网页仍历史读取。避免说‘全部搬到 LOTUSim’。")


# ---------------------------------------------------------------- slide 9
s = prs.slides.add_slide(prs.slide_layouts[6])
title(s, "交互层级", "从外部渲染到实时联合仿真，还差两级",
      "上次把 L1 推到 L2；这次把 L2 推到 L3；L4 尚未完成。")
levels = [
    ("L0", "离线网页", "纯历史数据/JS 播放，不接 LOTUSim", RGBColor(225, 230, 238), MUTED),
    ("L1", "外部 renderer 话题", "player.py 发布 cmd/poses，不创建 Gazebo 实体", PALE_ORANGE, ORANGE),
    ("L2", "MAS 实体回放", "gazebo_player / recorded_replay：CREATE/MOVE/DELETE；仍是历史", PALE_ORANGE, ORANGE),
    ("L3", "物理闭环", "xdyn 6DoF + LOTUSim physics plugin + gateway + 控制", PALE_TEAL, TEAL),
    ("L4", "实时联合仿真", "实时 VehicleState、时钟同步、速度传递、暂停、状态所有权", PALE_BLUE, BLUE),
]
for i, (tag, head, body, pale, accent) in enumerate(levels):
    y = 1.52 + i * 0.92
    box(s, 0.65, y, 11.95, 0.74, pale if i in (2, 3) else WHITE, accent if i in (2, 3) else LINE)
    pill(s, 0.85, y + 0.18, 0.55, 0.30, tag, accent, WHITE, 9)
    txt(s, 1.60, y + 0.13, 2.05, 0.28, head, 12.5, NAVY, True)
    txt(s, 3.75, y + 0.17, 8.55, 0.30, body, 10.5, INK)
    if i == 3:
        pill(s, 11.35, y + 0.17, 1.02, 0.30, "本次", TEAL, WHITE, 8.5)
    if i == 2:
        pill(s, 11.35, y + 0.17, 1.02, 0.30, "上次", ORANGE, WHITE, 8.5)
    if i == 4:
        pill(s, 11.35, y + 0.17, 1.02, 0.30, "未完成", BLUE, WHITE, 8.5)
box(s, 0.65, 6.25, 11.95, 0.50, NAVY, NAVY)
txt(s, 0.85, 6.35, 11.55, 0.28,
    "不要把 L2 的回放叫作联合仿真；L4 至少需要实时 VehicleState、统一时钟、速度/位姿传递和暂停语义。",
    10.8, WHITE, True)
footer(s, 9)
notes(s, "五级 ladder：L2 是两次都有的实体回放；这次到 L3 物理闭环；L4 才是实时联合仿真。")


# ---------------------------------------------------------------- slide 10
s = prs.slides.add_slide(prs.slide_layouts[6])
title(s, "结果与证据", "有数字，但数字只属于各自场景",
      "S1 与 S2 缩比各 13/13；两批数据不可直接横向排名。")
rows = [
    ["对象", "规模", "到达", "最小净距", "路径跟踪"],
    ["S1 单障碍恒流", "F0/F1/F2 × P0–P4 + 重复", "13/13", "1.62–1.74 m", "RMS ≤ 0.272 m"],
    ["S2 缩比双障碍", "F0/F1/F2 × P0–P4 + 重复", "13/13", "1.54–1.69 m", "RMS ≤ 0.338 m；最大 0.773 m"],
    ["测试", "replay + current + double", "37 passed", "—", "—"],
    ["里程碑封存", "证据索引", "12 entries", "519 files", "413 protected hashes"],
]
make_table(s, rows, 0.55, 1.55, 12.20, [2.55, 3.25, 1.85, 2.15, 2.40], row_h=0.62, font_size=10.2, first_col_bold=True)
box(s, 0.55, 4.55, 5.95, 1.25, PALE_GREEN, GREEN)
txt(s, 0.78, 4.72, 5.45, 0.28, "能说", 12, GREEN, True)
txt(s, 0.78, 5.02, 5.45, 0.65, "在这些限定场景/水流/初态下，LOTUSim+xdyn 闭环完成到达、安全净距和路径跟踪；证据已封存。", 11, INK)
box(s, 6.85, 4.55, 5.90, 1.25, PALE_RED, RED)
txt(s, 7.08, 4.72, 5.40, 0.28, "不能说", 12, RED, True)
txt(s, 7.08, 5.02, 5.40, 0.65, "全高 S2/S3、完整 45 次矩阵、任意流场、实艇/LRAUV、实时联合仿真已通过。", 11, INK)
box(s, 0.55, 6.10, 12.20, 0.55, WHITE, LINE)
txt(s, 0.78, 6.20, 11.80, 0.30,
    "上次是 2 臂 × 1 场景的历史对比；这次是 2 场景 × 3 水流 × 5 初态 + 重复。实验轴不同，不能按成功率/误差排名。",
    10.2, MUTED)
footer(s, 10)
notes(s, "结果页。S1/S2 数字来自 acceptance/suite/freeze；强调场景不同，不可直接排名。")


# ---------------------------------------------------------------- slide 11
s = prs.slides.add_slide(prs.slide_layouts[6])
title(s, "边界", "四个最容易说错的点",
      "视觉相似会诱导过度解读；这页用来把结论钉在证据上。")
items = [
    ("01", "相同画面 ≠ 相同证据", "上次是历史回放；这次是闭环实验 + 记录回放。最终画面不能替代 acceptance.json。", RED),
    ("02", "三维呈现 ≠ 三维避障", "挡板沿 y 加厚只是显示；规划和执行仍是 x-z 二维，没有新增 y 向避障。", ORANGE),
    ("03", "艇形资产 ≠ LRAUV 验证", "简化外形/示意模型只用于展示；合成动力学模型不能被称为经验证的 LRAUV。", BLUE),
    ("04", "历史回放 ≠ 实时联合仿真", "没有实时 VehicleState、时钟锁步、速度传递、暂停/恢复；L2 仍是历史墙钟。", TEAL),
]
for i, (n, head, body, accent) in enumerate(items):
    col, row = i % 2, i // 2
    x = 0.6 + col * 6.18
    y = 1.55 + row * 2.28
    box(s, x, y, 5.85, 2.00, WHITE)
    txt(s, x + 0.25, y + 0.20, 0.55, 0.30, n, 18, accent, True)
    txt(s, x + 0.90, y + 0.22, 4.65, 0.30, head, 14, NAVY, True)
    txt(s, x + 0.25, y + 0.75, 5.30, 1.05, body, 10.8, INK)
box(s, 0.6, 6.20, 12.10, 0.48, PALE_BLUE, LINE)
txt(s, 0.82, 6.31, 11.65, 0.26, "回放/画面只回答“看起来怎样”；suite.json、acceptance.json、manifest 和封存索引才回答“是否通过”。", 10.2, NAVY, True)
footer(s, 11)
notes(s, "边界四连：证据、三维、LRAUV、实时。每一条都是评审最可能追问的点。")


# ---------------------------------------------------------------- slide 12
s = prs.slides.add_slide(prs.slide_layouts[6])
title(s, "下一步", "从 L3 到 L4：把“历史读取器”换成实时状态",
      "上次已经证明 Gazebo 能承载实体回放；这次证明 xdyn 闭环能产生证据；下一步是时间/状态所有权。")
steps = [
    ("1", "实时 VehicleState", "订阅实际状态，替代历史 JSON；保留来源/时序校验。"),
    ("2", "统一时钟", "xdyn、Gazebo、ROS 时间源；超时/watchdog；失步处理。"),
    ("3", "速度/位姿传递", "状态回写；命令/观测分离；避免双发布者。"),
    ("4", "暂停/恢复语义", "暂停时不发旧命令；恢复时重置时间基准；清理只属于本次的实体。"),
]
for i, (n, head, body) in enumerate(steps):
    x = 0.6 + i * 3.10
    box(s, x, 1.65, 2.85, 2.40, WHITE)
    pill(s, x + 0.20, 1.85, 0.45, 0.30, n, TEAL, WHITE, 9.5)
    txt(s, x + 0.20, 2.28, 2.45, 0.35, head, 12.2, NAVY, True)
    txt(s, x + 0.20, 2.72, 2.45, 1.10, body, 9.6, INK)
box(s, 0.6, 4.38, 12.10, 1.18, NAVY, NAVY)
txt(s, 0.85, 4.55, 11.55, 0.30, "目标形态", 11, RGBColor(142, 205, 208), True)
txt(s, 0.85, 4.88, 11.55, 0.50, "同一个 Gazebo 世界：实时状态进入，控制命令回到物理；回放/验收仍使用独立记录，不把画面当证据。", 13, WHITE, True)
# conclusion + links
box(s, 0.6, 5.80, 5.95, 1.12, PALE_ORANGE, ORANGE)
txt(s, 0.82, 5.95, 5.50, 0.28, "上次交付", 10.5, ORANGE, True)
txt(s, 0.82, 6.22, 5.50, 0.55, "解决“在 Gazebo 里看见旧实验”。", 11.5, INK, True)
box(s, 6.75, 5.80, 5.95, 1.12, PALE_TEAL, TEAL)
txt(s, 6.97, 5.95, 5.50, 0.28, "这次工作", 10.5, TEAL, True)
txt(s, 6.97, 6.22, 5.50, 0.55, "解决“用 LOTUSim+xdyn 产生可验收的新历史，并仍能回看”。", 11.5, INK, True)
footer(s, 12, "详见 docs/lotusim_gazebo_replay.md · docs/lotusim_integration_and_stage_report_20261027.md · docs/lotusim_minimal_dynamics_milestone_20261028.md")
notes(s, "收尾：上次→看见历史；这次→产生并封存新历史；下一步是 L4 实时联合仿真，补 VehicleState、时钟、速度、暂停。")


prs.save(OUT)
print(OUT)
print("slides", len(prs.slides))
