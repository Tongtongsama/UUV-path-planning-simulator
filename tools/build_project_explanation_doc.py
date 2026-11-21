from pathlib import Path

from docx import Document
from docx.enum.section import WD_SECTION
from docx.enum.table import WD_CELL_VERTICAL_ALIGNMENT, WD_TABLE_ALIGNMENT
from docx.enum.text import WD_ALIGN_PARAGRAPH, WD_BREAK
from docx.oxml import OxmlElement
from docx.oxml.ns import qn
from docx.shared import Cm, Inches, Pt, RGBColor


ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / "deliverables" / "UUV项目完整技术说明_20261121.docx"


def set_cell_shading(cell, fill):
    tc_pr = cell._tc.get_or_add_tcPr()
    shd = tc_pr.find(qn("w:shd"))
    if shd is None:
        shd = OxmlElement("w:shd")
        tc_pr.append(shd)
    shd.set(qn("w:fill"), fill)


def set_cell_margins(cell, top=100, start=120, bottom=100, end=120):
    tc = cell._tc
    tcPr = tc.get_or_add_tcPr()
    tcMar = tcPr.first_child_found_in("w:tcMar")
    if tcMar is None:
        tcMar = OxmlElement("w:tcMar")
        tcPr.append(tcMar)
    for m, v in (("top", top), ("start", start), ("bottom", bottom), ("end", end)):
        node = tcMar.find(qn(f"w:{m}"))
        if node is None:
            node = OxmlElement(f"w:{m}")
            tcMar.append(node)
        node.set(qn("w:w"), str(v))
        node.set(qn("w:type"), "dxa")


def set_repeat_table_header(row):
    trPr = row._tr.get_or_add_trPr()
    tblHeader = OxmlElement("w:tblHeader")
    tblHeader.set(qn("w:val"), "true")
    trPr.append(tblHeader)


def add_page_number(paragraph):
    paragraph.alignment = WD_ALIGN_PARAGRAPH.CENTER
    run = paragraph.add_run()
    fldChar1 = OxmlElement("w:fldChar")
    fldChar1.set(qn("w:fldCharType"), "begin")
    instrText = OxmlElement("w:instrText")
    instrText.set(qn("xml:space"), "preserve")
    instrText.text = " PAGE "
    fldChar2 = OxmlElement("w:fldChar")
    fldChar2.set(qn("w:fldCharType"), "end")
    run._r.extend([fldChar1, instrText, fldChar2])


def set_run_font(run, name="Microsoft YaHei", size=10.5, bold=None, color=None):
    run.font.name = name
    run._element.get_or_add_rPr().rFonts.set(qn("w:eastAsia"), name)
    run.font.size = Pt(size)
    if bold is not None:
        run.bold = bold
    if color:
        run.font.color.rgb = RGBColor.from_string(color)


def add_para(doc, text="", style=None, bold_lead=None, keep=False):
    p = doc.add_paragraph(style=style)
    p.paragraph_format.space_after = Pt(6)
    p.paragraph_format.line_spacing = 1.35
    if keep:
        p.paragraph_format.keep_with_next = True
    if bold_lead and text.startswith(bold_lead):
        r = p.add_run(bold_lead)
        set_run_font(r, bold=True)
        r = p.add_run(text[len(bold_lead):])
        set_run_font(r)
    else:
        r = p.add_run(text)
        set_run_font(r)
    return p


def add_bullets(doc, items, level=0):
    for item in items:
        p = doc.add_paragraph(style="List Bullet" if level == 0 else "List Bullet 2")
        p.paragraph_format.space_after = Pt(3)
        p.paragraph_format.line_spacing = 1.25
        set_run_font(p.add_run(item))


def add_steps(doc, items):
    for index, item in enumerate(items, 1):
        p = doc.add_paragraph()
        p.paragraph_format.left_indent = Cm(0.5)
        p.paragraph_format.first_line_indent = Cm(-0.5)
        p.paragraph_format.space_after = Pt(4)
        p.paragraph_format.line_spacing = 1.25
        set_run_font(p.add_run(f"{index}.  {item}"))


def add_table(doc, headers, rows, widths=None):
    table = doc.add_table(rows=1, cols=len(headers))
    table.alignment = WD_TABLE_ALIGNMENT.CENTER
    table.autofit = False
    table.style = "Table Grid"
    hdr = table.rows[0]
    set_repeat_table_header(hdr)
    for i, h in enumerate(headers):
        cell = hdr.cells[i]
        set_cell_shading(cell, "173A5E")
        set_cell_margins(cell)
        cell.vertical_alignment = WD_CELL_VERTICAL_ALIGNMENT.CENTER
        p = cell.paragraphs[0]
        p.alignment = WD_ALIGN_PARAGRAPH.CENTER
        r = p.add_run(h)
        set_run_font(r, size=9.5, bold=True, color="FFFFFF")
        if widths:
            cell.width = widths[i]
    for ridx, row in enumerate(rows):
        cells = table.add_row().cells
        for i, value in enumerate(row):
            if ridx % 2 == 1:
                set_cell_shading(cells[i], "F2F6FA")
            set_cell_margins(cells[i])
            cells[i].vertical_alignment = WD_CELL_VERTICAL_ALIGNMENT.CENTER
            p = cells[i].paragraphs[0]
            p.alignment = WD_ALIGN_PARAGRAPH.CENTER if i == 0 else WD_ALIGN_PARAGRAPH.LEFT
            r = p.add_run(str(value))
            set_run_font(r, size=9)
            if widths:
                cells[i].width = widths[i]
    doc.add_paragraph().paragraph_format.space_after = Pt(1)
    return table


doc = Document()
section = doc.sections[0]
section.page_width = Inches(8.5)
section.page_height = Inches(11)
section.top_margin = Inches(0.72)
section.bottom_margin = Inches(0.68)
section.left_margin = Inches(0.82)
section.right_margin = Inches(0.82)

styles = doc.styles
styles["Normal"].font.name = "Microsoft YaHei"
styles["Normal"]._element.rPr.rFonts.set(qn("w:eastAsia"), "Microsoft YaHei")
styles["Normal"].font.size = Pt(10.5)
styles["Title"].font.name = "Microsoft YaHei"
styles["Title"]._element.rPr.rFonts.set(qn("w:eastAsia"), "Microsoft YaHei")
styles["Title"].font.size = Pt(28)
styles["Title"].font.bold = True
styles["Title"].font.color.rgb = RGBColor(0, 0, 0)
for style_name, size in (("Heading 1", 17), ("Heading 2", 13), ("Heading 3", 11)):
    st = styles[style_name]
    st.font.name = "Microsoft YaHei"
    st._element.rPr.rFonts.set(qn("w:eastAsia"), "Microsoft YaHei")
    st.font.size = Pt(size)
    st.font.bold = True
    st.font.color.rgb = RGBColor(0, 0, 0)
    st.paragraph_format.space_before = Pt(12)
    st.paragraph_format.space_after = Pt(6)
    st.paragraph_format.keep_with_next = True

footer = section.footer
add_page_number(footer.paragraphs[0])
set_run_font(footer.paragraphs[0].runs[0], size=8, color="6B7280")

# Cover
p = doc.add_paragraph(style="Title")
p.alignment = WD_ALIGN_PARAGRAPH.CENTER
p.paragraph_format.space_before = Pt(120)
p.paragraph_format.space_after = Pt(24)
set_run_font(p.add_run("UUV 仿真联合验证项目完整技术说明"), size=28, bold=True)
p = doc.add_paragraph()
p.alignment = WD_ALIGN_PARAGRAPH.CENTER
p.paragraph_format.space_after = Pt(10)
set_run_font(p.add_run("项目建设过程 系统运行逻辑 Python 与 LOTUSim 联通方式"), size=15, color="173A5E")
p = doc.add_paragraph()
p.alignment = WD_ALIGN_PARAGRAPH.CENTER
p.paragraph_format.space_before = Pt(180)
set_run_font(p.add_run("用于与陆博士技术对接"), size=12, bold=True)
p = doc.add_paragraph()
p.alignment = WD_ALIGN_PARAGRAPH.CENTER
set_run_font(p.add_run("2026 年 11 月 21 日"), size=10.5, color="4B5563")
doc.add_page_break()

doc.add_heading("文档目的与核心结论", level=1)
add_para(doc, "本文从项目起点开始说明 UUV 仿真联合验证平台如何建设、当前系统如何运行、Python 与 LOTUSim 如何分工和联通，以及现阶段证据能够支持什么结论。本文面向项目对接与技术评审，既解释算法和软件结构，也保留验证范围与限制。")
add_para(doc, "核心结论：", bold_lead="核心结论：")
add_para(doc, "项目已经把多障碍路径规划、轨迹制导、闭环控制、推力分配、xdyn 6DOF 动力学、LOTUSim/Gazebo 场景回写、时变实测水流输入、运行门禁、验收记录和离线展示连接成一条可复现的 simulator-only 联合验证链路。S4 联合矩阵 18/18 通过，并补充到 45 条 joint 记录用于重复性分析。该成果证明了软件与仿真闭环的可运行性和可审计性，但不等同于实艇海试或运行认证。")

doc.add_heading("阅读路线", level=1)
add_steps(doc, [
    "第 1 至 3 章说明项目为什么建立、采用什么总体思路，以及各阶段如何逐步完成。",
    "第 4 至 7 章解释当前系统架构、Python 与 LOTUSim 的分工和一条 trial 从启动到结束的完整运行逻辑。",
    "第 8 至 10 章说明规划、控制、水流、门禁和验收如何实现，以及项目形成了哪些证据。",
    "第 11 至 13 章给出复现方式、限制和最终解决的问题。",
])

doc.add_heading("术语说明", level=1)
add_table(doc, ["术语", "本文含义"], [
    ("LOTUSim", "项目使用的外部仿真与 Gazebo 集成环境。用户口语中的 LotoSim 在本文统一写作 LOTUSim。"),
    ("xdyn", "由 xdyn-for-cs 执行的完整 6DOF 动力学求解器。"),
    ("joint trial", "Python 控制、xdyn 动力学与 LOTUSim/Gazebo 同时参与的联合仿真试验。"),
    ("pilot", "受控范围内的先导试验记录，不代表海试。"),
    ("F0 F1 F2", "F0 为无流控制，F1 为常流控制，F2 为 v10 measured-current。"),
    ("P0 至 P4", "五组冻结初始条件。P0 repeat 是矩阵中的重复运行。"),
], [Cm(3.0), Cm(13.0)])

doc.add_heading("1 项目背景与需要解决的问题", level=1)
add_para(doc, "UUV 在水下执行任务时，需要在受限空间和水流扰动下，从起点安全到达目标。单独验证路径规划器并不足以说明系统可用，因为几何上可行的路径可能无法被真实动力学和受限执行器稳定跟踪；单独验证控制器也不足以说明它能在多障碍场景中完成任务。项目因此需要一个统一平台，把场景、规划、轨迹、控制、动力学、外部仿真、记录和验收放在同一条链路中。")
add_para(doc, "项目最初面对的核心缺口包括：")
add_bullets(doc, [
    "各模块接口缺少统一的数据结构和坐标约定，容易出现位置、速度、姿态和水流方向含义不一致。",
    "Python 算法结果与外部仿真环境之间缺少可验证的双向通信，难以区分离线回放与真实闭环。",
    "路径规划、控制器和动力学的成功标准不统一，失败可能被图像展示掩盖。",
    "水流输入长期停留在无流或常流，需要接入带来源和限制说明的 measured-current 数据。",
    "运行结果缺少冻结证据、门禁和重复性记录，不利于复现、审计与对接。",
])
add_para(doc, "因此，项目解决的不是单一算法问题，而是一个端到端工程验证问题：如何让 UUV 的规划和控制算法在外部动力学与可视化环境中形成真实反馈闭环，并让每次运行都有明确输入、输出、验收结果和证据边界。")

doc.add_heading("2 总体技术路线", level=1)
add_para(doc, "项目采用分层、可替换、逐级验证的结构。基础数据和算法主要使用 Python 实现；动力学由 xdyn-for-cs 计算；LOTUSim/Gazebo 负责实体、世界和位姿展示；ROS 2 和 WebSocket 网关承担跨进程通信。每一层先单独验证，再进入联合闭环。")
add_table(doc, ["层次", "主要职责", "实现方式"], [
    ("Core", "定义 Pose Twist ControlInput VehicleState Path Trajectory", "Python 不可变数据模型"),
    ("Environment", "边界 障碍 clearance collision current 查询", "Python Shapely 封装与配置文件"),
    ("Planner", "在 x z 平面生成安全路径", "Python A star 安全捷径 圆角与终点段"),
    ("Guidance Control", "把路径与状态转换为 X 和 M 请求", "Python G12 guide PID 与推力分配"),
    ("Dynamics", "根据推力和环境流积分完整状态", "xdyn-for-cs 6DOF RK4 dt 0.05 s"),
    ("Simulation", "维护场景实体并发布实际位姿", "LOTUSim Gazebo 与 lotusim_msgs"),
    ("Orchestration", "启动进程 调度 trial 清理 记录", "Python runner Bash Nix ROS 2"),
    ("Evidence", "门禁 acceptance repeatability dashboard", "Python JSON JSONL HTML 与图像"),
], [Cm(2.4), Cm(7.1), Cm(6.5)])

doc.add_heading("3 项目如何逐步完成", level=1)
doc.add_heading("3.1 建立统一数据模型和坐标约定", level=2)
add_para(doc, "首先在 Core 层冻结位置、速度、控制量、路径和轨迹的数据合同。世界坐标采用 ENU，艇体速度和控制采用 SNAME body frame。时间戳作为仿真元数据保存，不混入 12 维物理状态向量。这个阶段解决了后续模块之间传什么、单位是什么、谁负责坐标变换的问题。")

doc.add_heading("3.2 建立环境和几何查询", level=2)
add_para(doc, "Environment 层实现边界、圆形矩形多边形障碍、碰撞、净距和水流查询。Shapely 被封装在 Environment 内部，Planner 不直接依赖具体几何库。这样规划器只使用稳定接口，未来可以替换地图或几何后端。")

doc.add_heading("3.3 建立 Python 参考动力学和控制基线", level=2)
add_para(doc, "项目先实现 Python 3DOF 参考动力学，用于快速、确定性的单元测试和闭环验证；随后实现级联 PID、抗积分饱和、恢复力矩候选以及导航终点策略。Python 参考链路的作用是验证算法合同和数值行为，不等同于最终的 LOTUSim 联合链路。")

doc.add_heading("3.4 完成规划 轨迹与导航闭环", level=2)
add_para(doc, "Planner 从基本接口发展到 A star 基线，再加入安全捷径、圆角处理和 S4 候选终点段。Trajectory 把几何路径转换为可跟踪的时间或进度参考。Navigation 负责根据当前状态推进路径进度、计算制导目标并管理终点阶段。")

doc.add_heading("3.5 接入 xdyn 与 LOTUSim", level=2)
add_para(doc, "联合链路没有把 LOTUSim 当作 Python 算法的替代品。Python 仍然计算制导、控制和试验逻辑；xdyn 负责积分；LOTUSim/Gazebo 负责实体和位姿。项目通过自定义 gateway 把命令和状态在 ROS 2 与 xdyn 之间传递，并通过 lotusim_msgs 读取 Gazebo 中实际发布的位姿。")

doc.add_heading("3.6 加入门禁 验收和证据冻结", level=2)
add_para(doc, "在正式 trial 前执行传输、运行时、Python 证据和 release 哈希检查。运行后由 acceptance 程序复算状态、控制、分配、净距、到达条件和记录一致性。失败、门禁拒绝和动态碰撞分别保存，不把未启动试验写成运行失败，也不把失败轨迹裁剪成成功。")

doc.add_heading("3.7 加入实测流和联合矩阵", level=2)
add_para(doc, "在无流和常流基线稳定后，项目接入 NOAA CO OPS 单站观测，生成 v10 measured-current schedule，并将 S4 D 路径、G12 控制和 v10 流表组合成 twin_axial_s4_current_v1。最终形成 F0 F1 F2 三种流场与 P0 至 P4 加 P0 repeat 的 18 条矩阵，并继续补跑到每个 initial 三条运行。")

doc.add_page_break()
doc.add_heading("4 当前系统架构", level=1)
add_para(doc, "当前联合系统由六类进程或模块组成。其关系可以用下面的数据流描述。")
add_para(doc, "场景与路径配置 → Python Mission 和 Guide → Python PID 与推力分配 → ROS 2 命令 → gateway → xdyn 6DOF → gateway 状态 → Python 控制循环，同时 LOTUSim/Gazebo 发布实体位姿 → Python 记录与 acceptance。")
add_table(doc, ["组件", "输入", "输出", "是否参与在线闭环"], [
    ("Mission Planner", "场景 初始条件 流场", "路径 配置 Guide", "是"),
    ("AllocatedPID", "solver state guidance target dt", "X M 与上下推进器请求", "是"),
    ("ROS 2 trial node", "状态反馈与 Gazebo pose", "VesselCmdArray 记录与停止命令", "是"),
    ("gateway", "ROS 2 命令和 xdyn socket 状态", "xdyn 输入与 /state 消息", "是"),
    ("xdyn-for-cs", "model.yml 推力 current dt", "完整 6DOF 状态", "是"),
    ("LOTUSim/Gazebo", "world 与 vessel command", "VesselPositionArray 和 GUI", "是 用于实体与位姿"),
    ("dashboard viewer", "冻结 JSON JSONL 和图片", "浏览器展示", "否 只做离线展示"),
], [Cm(3.0), Cm(5.0), Cm(5.0), Cm(3.0)])

doc.add_heading("5 Python 与 LOTUSim 的分工", level=1)
doc.add_heading("5.1 Python 完成的工作", level=2)
add_bullets(doc, [
    "读取 scene flow case 和 release 配置，生成本次 mission。",
    "运行 A star 与后处理，或者加载冻结的 S4 D 路径。",
    "根据 xdyn 返回的实际状态计算路径进度、LOS 或 G12 制导目标。",
    "运行 PID、限幅、抗积分饱和和方向保持推力分配。",
    "启动和监督 xdyn gateway LOTUSim，检查端口、超时和 stale feedback。",
    "通过 ROS 2 发布命令、订阅状态和 LOTUSim 位姿。",
    "记录 navigation state commands gazebo poses logs metadata manifest。",
    "运行 gate acceptance 汇总 repeatability 并生成 dashboard。",
])
doc.add_heading("5.2 xdyn 完成的工作", level=2)
add_para(doc, "xdyn-for-cs 读取 model.yml，以 0.05 s 固定步长使用 RK4 积分完整 6DOF 状态。它接收上下推进器的理想推力和水流条件，计算下一步位置、姿态、线速度和角速度。项目明确记录 no state clamping，避免通过裁剪状态制造稳定结果。")
doc.add_heading("5.3 LOTUSim Gazebo 完成的工作", level=2)
add_para(doc, "LOTUSim 启动 Gazebo world、创建 UUV 和障碍实体，并通过 lotusim_msgs 发布 VesselPositionArray。联合 trial 订阅 /uuv_twin_axial/poses，把 Gazebo 端实际发布的位姿写入 gazebo_poses.jsonl。GUI 用于观察场景和艇体，但不替代 xdyn 动力学。")
doc.add_heading("5.4 为什么需要三者协作", level=2)
add_para(doc, "Python 适合快速实现和测试规划控制算法，xdyn 提供独立的高维动力学求解，LOTUSim/Gazebo 提供外部仿真接口和可视化。把三者分开，可以避免算法、动力学和显示相互隐藏错误，也使每一层可以单独测试和替换。")

doc.add_heading("6 Python 与 LOTUSim 如何联通", level=1)
add_para(doc, "联合运行使用两条通信通道。第一条是 Python trial node 与 LOTUSim 之间的 ROS 2 通道；第二条是 gateway 与 xdyn-for-cs 之间的本机 WebSocket 通道。")
add_table(doc, ["通道", "方向", "内容", "接口"], [
    ("ROS 2 命令", "Python → LOTUSim gateway", "VesselCmdArray 内含 JSON 命令和时间戳", "/uuv_twin_axial/vessel_cmd_array"),
    ("ROS 2 状态", "gateway → Python", "xdyn sequence elapsed 完整状态 valid error", "/uuv_twin_axial/state"),
    ("ROS 2 位姿", "LOTUSim → Python", "Gazebo 实际位置和四元数", "/uuv_twin_axial/poses"),
    ("ROS 2 action", "Python ↔ LOTUSim", "创建或删除本次 trial 的实体", "/uuv_twin_axial/mas_cmd_array"),
    ("WebSocket", "gateway ↔ xdyn", "推力 水流 状态积分事务", "127.0.0.1 ports 12356 12357"),
], [Cm(2.8), Cm(3.5), Cm(6.0), Cm(4.0)])
add_para(doc, "每次 trial 使用隔离的 ROS_DOMAIN_ID 和 GZ_PARTITION，运行前检查端口是否可绑定，并拒绝干扰已有实例。结束时先把命令归零，再停止 gateway 和 xdyn，最后通过 MAS action 删除本次实体。这个顺序确保停止后不再继续积分，并保留完整终止证据。")

doc.add_page_break()
doc.add_heading("7 一条联合 trial 的完整运行逻辑", level=1)
doc.add_heading("7.1 启动前门禁", level=2)
add_steps(doc, [
    "检查 v6 transport probes 是否完整，包括正常、重复和中止路径。",
    "检查运行时是否存在 python3、xdyn-for-cs、lotusim、rclpy 和 lotusim_msgs。",
    "校验 release 文件与源代码哈希，防止证据对应的代码已经漂移。",
    "检查 S4 Python 证据与 v10 measured-current Python 证据。",
    "只有 gate passed 且 ready_to_run 为 true 才创建正式 trial。",
])
doc.add_heading("7.2 生成本次运行目录和配置", level=2)
add_para(doc, "runner 根据 scene flow case 生成唯一输出目录，复制模型、路径、world、release 和必要源文件，并写入 metadata。输出目录不会覆盖历史记录。F2 还会写入 current_schedule.json，保留 true current、controller current 和 estimated current 的来源。")
doc.add_heading("7.3 启动 xdyn", level=2)
add_para(doc, "runner 启动 xdyn-for-cs，指定 model.yml、RK4、127.0.0.1 端口和 dt 0.05 s。随后持续探测 solver port。如果进程提前退出或启动超时，trial 立即失败并保存日志。")
doc.add_heading("7.4 启动 gateway", level=2)
add_para(doc, "gateway 建立 WebSocket 服务端和客户端连接，把 ROS 2 命令转换为 xdyn 事务，并把 xdyn 返回状态发布为 /uuv_twin_axial/state。gateway 不预测或伪造状态。gateway.ready 文件出现后，runner 才继续。")
doc.add_heading("7.5 创建 ROS 2 控制节点并启动 LOTUSim", level=2)
add_para(doc, "Python 创建 uuv_twin_axial_navigation 节点，建立 VesselCmdArray publisher、String state subscriber 和 VesselPositionArray pose subscriber。随后启动 lotusim run --debug minimal.world；需要 GUI 时追加 --gui。")
doc.add_heading("7.6 等待真实状态并计算控制", level=2)
add_steps(doc, [
    "接收新的 xdyn sequence，检查 valid 字段和反馈新鲜度。",
    "Mission 根据当前仿真时间取得 F0 F1 或 F2 的 current。",
    "Guide 依据实际位置、姿态和路径进度计算目标速度、目标俯仰和阶段。",
    "AllocatedPID 计算请求合力 X 与俯仰力矩 M，并执行限幅和抗积分饱和。",
    "分配器把 X M 转换为上下推进器理想推力，保持请求方向和执行器约束。",
    "trial node 将命令、时间戳和诊断发布到 ROS 2，并同时写入 commands.jsonl 与 navigation.jsonl。",
])
doc.add_heading("7.7 xdyn 积分与 LOTUSim 位姿反馈", level=2)
add_para(doc, "gateway 把本步推力和水流送入 xdyn。xdyn 计算下一步完整状态，再由 gateway 发布。LOTUSim/Gazebo 侧同步实体并发布 /poses；trial node 把接收到的实体位置和四元数保存为 gazebo_poses.jsonl。控制器下一步使用的是求解器实际反馈，而不是预先生成的轨迹动画。")
doc.add_heading("7.8 到达 停止与清理", level=2)
add_para(doc, "当到达条件成立，或发生超时、碰撞、stale feedback、进程退出等异常时，runner 进入终止阶段。正常情况下先发布零命令，停止 gateway 和 xdyn，随后删除 LOTUSim 实体并关闭 Gazebo。所有子进程返回码、清理结果和错误写入 run_status.json。")
doc.add_heading("7.9 acceptance 复算", level=2)
add_para(doc, "运行结束后 acceptance 重新读取所有记录，检查场景身份、流场、时间连续性、控制请求与实际发布一致性、路径误差、净距、到达时间、碰撞、饱和和终止行为。最后生成 acceptance.json。passed=true 只说明该固定 trial 满足定义的 simulator integration acceptance，不扩展为一般安全性。")

doc.add_heading("8 规划与控制逻辑", level=1)
doc.add_heading("8.1 S4 路径规划", level=2)
add_para(doc, "S4 是多障碍场景。项目比较了 raw A star、安全捷径、圆角平滑和带 4 m 水平终点段的 D 路径。A star 负责在离散占据空间中寻找无碰撞基线，安全捷径删除可以直接连通的冗余节点，圆角降低尖角导致的跟踪突变，D 路径通过明确终点段改善末段进入与保持。")
img = ROOT / "artifacts" / "presentation_20261119_01" / "s4_planning_comparison.png"
if img.exists():
    p = doc.add_paragraph()
    p.alignment = WD_ALIGN_PARAGRAPH.CENTER
    p.add_run().add_picture(str(img), width=Inches(6.7))
    cap = doc.add_paragraph("图 1  S4 A B C D 路径与 D 路径 pilot 对照")
    cap.alignment = WD_ALIGN_PARAGRAPH.CENTER
    cap.paragraph_format.keep_with_next = False
    set_run_font(cap.runs[0], size=9, color="4B5563")
doc.add_heading("8.2 G12 guide", level=2)
add_para(doc, "Guide 不直接输出推进器命令。它根据当前状态、路径进度和流场生成控制器可以理解的参考量，并管理启动、巡航、终点捕获和保持等阶段。F2 中 v10 schedule 同时进入真实流条件和控制器当前流信息，确保数据来源在记录中可追踪。")
doc.add_heading("8.3 PID 与推力分配", level=2)
add_para(doc, "控制器只请求 surge force X 和 pitch moment M，不直接请求 heave force。分配器把 X M 映射到上下轴向推进器的理想推力，保留执行器上下限和方向。控制参数在矩阵运行中冻结，不按 trial 调参，因此矩阵比较的是场景与初始条件变化，而不是不同控制器。")

doc.add_heading("9 水流数据如何进入系统", level=1)
add_para(doc, "项目保留三类水流输入。F0 为无流；F1 为常流控制；F2 为 v10 measured-current。F2 数据来自 NOAA CO OPS pm0101 bin 1，使用 2026 年 11 月 17 日 05:15 至 05:45 UTC 的 6 个样本，并重采样到 0 30 60 90 120 150 s。东向分量进入二维 x z 模型，北向分量只保留在 metadata。")
add_para(doc, "这一处理解决了实测数据如何以明确 schema、单位、时间轴和来源进入控制与动力学的问题，但没有解决空间剪切、垂直流、潮位基准或导航级测量精度。因此 measured-current 在本文中表示有来源的观测时间序列输入，不表示完整海洋环境重建。")

doc.add_heading("10 验证结果与证据边界", level=1)
add_table(doc, ["证据层", "数量或结果", "可以支持的结论", "不能支持的结论"], [
    ("S4 joint 18", "18/18 passed", "三流六运行定义下联合闭环通过", "海试或全工况安全"),
    ("joint repeats", "总计 45 条", "固定 initial 的仿真重复性", "45 条独立真实矩阵"),
    ("真实 S4 pilot", "5 条", "五个指定 S4 pilot 通过", "18 条真实矩阵"),
    ("S4 Python", "18 条", "常量流 Python 参考证据", "joint measured-current"),
    ("全量测试", "1256 passed 的冻结记录", "代码回归与接口合同", "实艇可靠性"),
], [Cm(2.5), Cm(2.6), Cm(5.5), Cm(5.4)])
add_para(doc, "Joint 18 的到达时间范围为 60.45 至 61.35 s，最小净距范围为 1.4415 至 1.5696 m，安全裕量为 0.5415 至 0.6696 m，RMS 误差为 0.1325 至 0.1672 m。历史早期 P2 的 15.1 mm spread 与当前矩阵 P2 的 0.11 mm spread 属于不同数据集，必须分开表达。")

doc.add_heading("11 展示系统与真实运行的区别", level=1)
add_para(doc, "joint dashboard 和 trajectory viewer 是自包含离线展示页面。它们读取已经冻结的记录，在浏览器中切换热力表、轨迹和证据分层。页面不会启动 solver、planner、controller、simulation、server 或 network。因此它们适合演示和审阅，但不能作为新的在线联合运行证据。")
img = ROOT / "artifacts" / "s4_trajectory_viewer_20261120_01" / "gui_demo_still.png"
if img.exists():
    p = doc.add_paragraph()
    p.alignment = WD_ALIGN_PARAGRAPH.CENTER
    p.add_run().add_picture(str(img), width=Inches(6.5))
    cap = doc.add_paragraph("图 2  S4 GUI 展示静帧与 v10 水流参照球")
    cap.alignment = WD_ALIGN_PARAGRAPH.CENTER
    set_run_font(cap.runs[0], size=9, color="4B5563")
add_para(doc, "当前 camera_demo.mp4 因 EGL 环境问题为黑帧，不作为有效证据。有效展示使用 GUI 静帧、浏览器轨迹回放和场景图；真正 GUI 视频需要可用 GPU EGL 环境或 Windows 侧录屏。")

doc.add_heading("12 复现与运行方式", level=1)
doc.add_heading("12.1 运行环境", level=2)
add_bullets(doc, [
    "Windows 主机与 WSL2 NixOS。",
    "项目仓库位于 C:\\Users\\Songyuan\\uuv_simulator，并从 WSL 以 /mnt/c/Users/Songyuan/uuv_simulator 访问。",
    "Nix 固定 xdyn LOTUSim ROS 2 与 Python 运行依赖；UUV_XDYN_LOCK 绑定求解器环境。",
    "Python 项目环境用于构建、测试、汇总和展示生成。",
])
doc.add_heading("12.2 推荐演示", level=2)
add_steps(doc, [
    "先打开 artifacts/s4_joint_dashboard_20261120_01/index.html，说明 18/18、轨迹和 evidence layers。",
    "需要运行 gate 时，在 WSL NixOS 执行 scripts/run_s4_current_joint_gate.sh。",
    "需要运行单条 joint trial 时执行 scripts/run_s4_current_joint_trial.sh。",
    "需要重跑矩阵时执行 scripts/run_s4_joint18_matrix.sh，并保留足够时间和独立输出目录。",
    "运行完成后执行 summarize_joint18.py 和 build_joint_dashboard.py 生成汇总与展示。",
])
doc.add_heading("12.3 每条 trial 的主要文件", level=2)
add_table(doc, ["文件", "作用"], [
    ("metadata.json", "场景 流场 case 模型 dt 可执行文件和作用域"),
    ("model.yml plan.json", "动力学模型与冻结路径"),
    ("navigation.jsonl", "制导阶段 路径误差 clearance current 和控制诊断"),
    ("state.jsonl", "xdyn 完整状态反馈"),
    ("commands.jsonl", "Python 实际发布的控制命令"),
    ("gazebo_poses.jsonl", "LOTUSim Gazebo 实际发布的位姿"),
    ("run_status.json", "终止状态 错误 清理和进程返回码"),
    ("acceptance.json", "复算后的验收检查与最终 passed"),
    ("manifest.json", "运行文件哈希"),
], [Cm(4.2), Cm(11.8)])

doc.add_heading("13 项目最终解决了什么问题", level=1)
add_para(doc, "项目最终解决的是 UUV 导航算法从孤立模块走向可验证联合闭环的问题。过去可以分别展示路径、控制曲线或 Gazebo 画面，但难以证明它们来自同一次真实反馈运行。现在，系统能够在多障碍场景中生成路径，以实际求解器状态驱动制导和控制，把推力送入独立 6DOF 动力学，再把 LOTUSim/Gazebo 位姿和所有中间量记录下来，并用门禁和 acceptance 判断这次运行是否成立。")
add_para(doc, "更具体地说，项目完成了四项关键闭环：")
add_bullets(doc, [
    "算法闭环：规划、轨迹、制导、PID 和推力分配能够连续工作。",
    "仿真闭环：控制命令经过 xdyn 积分并通过 LOTUSim/Gazebo 返回状态和位姿。",
    "数据闭环：无流、常流和有来源的 measured-current 可以按统一接口进入 trial。",
    "证据闭环：运行前有 gate，运行中有完整日志，运行后有 acceptance、repeatability 和展示包。",
])
add_para(doc, "因此，本项目的价值不只是得到 18/18 的结果，而是建立了一套可复现、可审计、可扩展的 UUV 联合仿真方法。它为下一阶段接入硬件在环、实验水池或海试提供了清晰接口和验收基线，同时也明确保留了 simulator-only 的边界。")

doc.add_heading("14 下一阶段建议", level=1)
add_steps(doc, [
    "与陆博士确认下一阶段的首要验证目标，是规划控制性能、软硬件接口，还是水流环境真实性。",
    "冻结 ROS 2 外部接口、坐标系、时间同步、采样率和故障处理合同。",
    "把当前 ideal thrust 接口映射到实际推进器或实验平台命令，并单独验证分配误差和延迟。",
    "增加硬件在环或水池试验，保持与当前 trial 相同的 metadata、日志和 acceptance 结构。",
    "扩展水流数据，加入空间变化、垂向分量和传感器不确定性，并避免把单站观测外推为完整海域。",
])

doc.add_heading("附录 A 关键实现入口", level=1)
add_table(doc, ["用途", "路径"], [
    ("项目总览", "README.md"),
    ("S4 实现说明", "docs/implementation_overview_20261120.md"),
    ("演示指南", "docs/demo_guide_20261120.md"),
    ("联合 trial", "ros2/twin_axial_s4_current_v1/trial.py"),
    ("联合 gate", "ros2/twin_axial_s4_current_v1/gate.py"),
    ("S4 measured current mission", "ros2/twin_axial_s4_current_v1/mission.py"),
    ("xdyn gateway", "ros2/twin_axial_g12_link_v6/gateway.py"),
    ("控制与分配", "ros2/twin_axial_pid/control.py"),
    ("主 dashboard", "artifacts/s4_joint_dashboard_20261120_01/index.html"),
    ("joint 18 汇总", "artifacts/s4_joint18_20261120_01/JOINT18.md"),
], [Cm(5.0), Cm(11.0)])

doc.add_heading("附录 B 结论口径", level=1)
add_bullets(doc, [
    "可以说：完成 S4 多障碍规划 控制 xdyn LOTUSim Gazebo 联合闭环。",
    "可以说：S4 joint 18 矩阵 18/18 通过，并有 45 条 joint 记录用于重复性分析。",
    "必须说：F0 F1 是 control，只有 F2 是 v10 measured-current。",
    "必须说：五条真实 pilot、18 条 Python 和 45 条 joint 是不同证据层。",
    "不能说：45 joint 等于 45 次海试，或 simulator-only 结果等于 operational certification。",
])

doc.core_properties.title = "UUV 仿真联合验证项目完整技术说明"
doc.core_properties.subject = "项目建设过程 系统运行逻辑 Python 与 LOTUSim 联通方式"
doc.core_properties.author = "UUV Simulator Project"
doc.core_properties.keywords = "UUV Python LOTUSim xdyn ROS2 S4 joint validation"

OUT.parent.mkdir(parents=True, exist_ok=True)
doc.save(OUT)
print(OUT)
