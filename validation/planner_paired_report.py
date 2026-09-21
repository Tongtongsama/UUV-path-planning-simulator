"""Read-only P0/P1 evidence analysis and static paired scientific plots."""
import argparse
from collections import Counter
import json
from pathlib import Path
from math import hypot
import numpy as np
from matplotlib.figure import Figure
from matplotlib.backends.backend_agg import FigureCanvasAgg
from matplotlib.patches import Rectangle
from validation.guidance_matched_comparison import ROOT, digest
from validation.navigation_acceptance import build_cases
from validation.planner_paired import write


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--input', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    manifest = json.loads((args.input/'manifest.json').read_text())
    if any(digest(args.input/name)!=value for name,value in manifest.items()):
        raise ValueError('source evidence checksum mismatch')
    args.output.mkdir(parents=True,exist_ok=False)
    summary = json.loads((args.input/'summary.json').read_text())
    geometry = json.loads((args.input/'geometry.json').read_text())
    settings = json.loads((ROOT/'config/navigation_v08.json').read_text())
    scenes = {case['name']:(suite,case) for suite,case,_ in build_cases('all',settings)}
    rows = [r for r in summary['rows'] if r['repeat']==1]
    fig = Figure(figsize=(14,17),layout='constrained'); FigureCanvasAgg(fig)
    axes = fig.subplots(4,3).flat
    diagnostics = []
    for ax,g in zip(axes,geometry):
        name = g['scene']; suite,case = scenes[name]
        for o in case['obstacles']:
            ax.add_patch(Rectangle((o['x_min'],o['z_min']),o['x_max']-o['x_min'],o['z_max']-o['z_min'],color='#a7adb5'))
        statuses = []
        for arm,color in [('P0','#3579b8'),('P1','#d87525')]:
            record = json.loads((args.input/'runs'/f'{name}_{arm}_1.json').read_text())
            if 'history' not in record:
                statuses.append(arm+': rejected'); continue
            row = record['summary']; states = record['history']['states']; controls = record['history']['controls']
            route = g['arms'][arm]['path']
            ax.plot([p['x'] for p in route],[p['z'] for p in route],'--',color=color,alpha=.6)
            ax.plot([s['pose']['x'] for s in states],[s['pose']['z'] for s in states],color=color,label=arm)
            statuses.append(arm+': '+row['status'])
            following = [s for s in controls if s['guidance_phase']=='PATH_FOLLOWING']
            net = [s['segment_clearance']-.3 for s in controls if s['segment_clearance'] is not None]
            events = record['turn_events']
            diagnostics.append(dict(scene=name,arm=arm,minimum_executed_obstacle_net=min(net) if net else None,
                following_collision=any(s['collision'] for s in following),
                following_out_of_bounds=any(not s['in_bounds'] for s in following),
                unknown_turns=len(events), preturn_observed=sum(e['preturn'] is not None for e in events),
                switch_observed=sum(e['switch'] is not None for e in events),
                preturn_before_switch=sum(e['preturn'] is not None and e['switch'] is not None and
                    e['preturn']['step']<e['switch']['step'] for e in events)))
        ax.set(title=name+'\n'+' / '.join(statuses),xlabel='East x (m)',ylabel='Up z (m)',
            xlim=(suite['boundary']['x_min'],suite['boundary']['x_max']),
            ylim=(suite['boundary']['z_min'],suite['boundary']['z_max']),aspect='equal')
        ax.legend(fontsize=8); ax.grid(alpha=.15)
    fig.suptitle('P0 / P1 | zero current, 0.2 m/s | dashed: selected path; solid: execution',fontsize=14)
    fig.savefig(args.output/'paired_paths.png',dpi=140); fig.clear()
    text = ['# P0/P1 有限候选对照结果','',
        '零流、0.2 m/s、原 120 s 预算，12 场景 × 2 臂 × 2 次。未调 PID/LOS/终端策略。',
        '两次独立执行逐步 history、事件、指标一致；规划墙钟耗时不参与确定性判定。',
        '原 0.5 m/s LOS 10/16 保持不变，本实验不能直接与之比较。','',
        '## 终止状态（每臂 12 个独立条件，不把重复当作新样本）','',
        '| Arm | Status counts |','| --- | --- |']
    for arm in ('P0','P1'):
        text.append('| '+arm+' | '+str(summary['counts'][arm])+' |')
    text += ['', '## 场景结果与计算开销','',
        '| Scene | Arm | Status | Length m | Plan ms | Expanded | Executed net m | Boundary net m |',
        '| --- | --- | --- | ---: | ---: | ---: | ---: | ---: |']
    for r in rows:
        d = next(d for d in diagnostics if d['scene']==r['scene'] and d['arm']==r['arm'])
        net = '—' if d['minimum_executed_obstacle_net'] is None else f"{d['minimum_executed_obstacle_net']:.4f}"
        text.append(f"| {r['scene']} | {r['arm']} | {r['status']} | {r['path_length']:.3f} | {r['planning_time']*1000:.2f} | {r['expanded_nodes']} | {net} | {r['metrics']['minimum_boundary_margin']:.4f} |")
    text += ['', '## 解释边界','',
        '- P1 只在 detour 改变最终路径；其余 11 个条件不是不同路径的强对照。',
        '- detour 转角数 5→2，路径 10.071→10.485 m；采样参考障碍净距 0.200→0.700 m。',
        '- 软净距由 0.1 m 采样积分计算，不是连续最小值证明；可行性仍执行原连续碰撞检查。',
        '- 额外 buffer 只用于候选生成。0.75 m buffer 在这些起点可能被拒绝，不是原任务 INVALID_START。',
        '- 代价最小只指有限候选集合；转角/间距属于最终排序，不具有该总代价的 A* 搜索最优性。',
        '- 4 m preferred spacing 是经验软偏好，并非 v2 恢复距离的认证下界。所有转弯证据仍为 UNKNOWN。',
        '- preturn 保存前视点首次触及角点的施令前状态，另存首次偏离入段切向命令和切段状态；不是孤立入口匹配证明。',
        '- 原稳定带下相邻转角是否恢复未作可靠匹配，不能由间距软代价推断恢复成功。',
        '- 失败的偏差/控制 effort 基于截短历史；本次不能据低累计 effort 或较短时长宣称改善。',
        '- 不同路径会产生不同 startup 目标和 terminal 入口，这是完整路径效果，非隔离转角的因果实验。',
        '- 路径生成能力有限：没有 heading-aware 搜索、曲率约束或 current-aware cost。',
        '- 可开始独立 ROS 2 adapter 接口工作，无须等待 terminal/current/全部困难场景解决。','',
        '完整每步历史、双重复身份、失败阶段、空间偏差/q/饱和指标和冻结指纹见输入 evidence。']
    (args.output/'report.md').write_text('\n'.join(text)+'\n',encoding='utf-8')
    write(args.output/'diagnostics.json',diagnostics)
    write(args.output/'manifest.json',dict(source_manifest_sha256=digest(args.input/'manifest.json'),
        report_source_sha256=digest(Path(__file__)),files={p.name:digest(p) for p in args.output.iterdir() if p.is_file()}))


if __name__=='__main__':
    main()
