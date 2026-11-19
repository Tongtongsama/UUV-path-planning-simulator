"""Visual comparison of constant, synthetic segmented and measured current S3/F2 runs."""
from pathlib import Path
import csv, json
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
import sys
from matplotlib.patches import Rectangle

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
OUT = ROOT / 'artifacts/current_comparison_20261119_01'
SCENES = [[6, 7, -20, -5.5], [13, 14, -14.5, 0]]

CASES = {
    'constant F2': {
        'root': ROOT/'artifacts/gui_current_v1_20261118_01/S3/F2',
        'color': '#d95f02', 'kind': 'constant',
    },
    'synthetic segmented (v4)': {
        'root': ROOT/'artifacts/gui_current_v4_20261118_01/S3/F2',
        'color': '#1b9e77', 'kind': 'synthetic',
    },
    'measured (v10)': {
        'root': ROOT/'artifacts/gui_current_v10_20261118_02/S3/F2',
        'color': '#3b5bdb', 'kind': 'measured',
    },
}


def load(case, initial):
    d = CASES[case]['root']/initial
    nav = [json.loads(s) for s in (d/'navigation.jsonl').read_text().splitlines() if s]
    acc = json.loads((d/'acceptance.json').read_text())
    plan = json.loads((d/'plan.json').read_text())
    return nav, acc, plan


def current_at(case, t):
    kind = CASES[case]['kind']
    if kind == 'constant':
        return (0.0, 0.05)
    if kind == 'synthetic':
        from ros2.twin_axial_g12_current_v4.schedule import current
        return current(t)
    from ros2.twin_axial_g12_current_v10.schedule import current
    return current(t)


def style(ax):
    ax.grid(True, alpha=.25, linewidth=.6)
    ax.set_axisbelow(True)
    for s in ('top', 'right'):
        ax.spines[s].set_visible(False)


def obstacle(ax):
    for x0, x1, z0, z1 in SCENES:
        ax.add_patch(Rectangle((x0, z0), x1-x0, z1-z0, facecolor='#666666', alpha=.55, edgecolor='none', zorder=1))
    ax.set_aspect('equal', adjustable='box')


def main():
    plan = json.loads((ROOT/'artifacts/gui_current_v10_20261118_02/S3/F2/P0/plan.json').read_text())
    path = np.asarray(plan['path'])

    # Figure 1: trajectories.
    fig, axes = plt.subplots(1, 2, figsize=(12.0, 4.6), dpi=180, sharey=True)
    for ax, initial in zip(axes, ('P0', 'P2')):
        obstacle(ax)
        ax.plot(path[:, 0], path[:, 1], color='#222222', lw=2.0, ls='--', label='planned path', zorder=3)
        for case in CASES:
            nav, _, _ = load(case, initial)
            xy = np.asarray([r['position_xz'] for r in nav])
            ax.plot(xy[:, 0], xy[:, 1], color=CASES[case]['color'], lw=1.5, label=case, zorder=4)
        ax.scatter([path[0,0]], [path[0,1]], s=35, c='#222222', marker='o', zorder=5)
        ax.scatter([path[-1,0]], [path[-1,1]], s=55, c='#222222', marker='*', zorder=5)
        ax.set_title(f'S3/F2/{initial}: actual trajectory', fontsize=11, fontweight='bold')
        ax.set_xlabel('planning x (m)'); ax.set_ylabel('planning z (m)')
        ax.set_xlim(4.0, 20.0); ax.set_ylim(-19.0, -4.0)
        style(ax)
    axes[0].legend(loc='lower right', fontsize=8, frameon=True)
    fig.suptitle('Current-aware G12: constant vs synthetic segmented vs measured current', fontsize=13, fontweight='bold')
    fig.tight_layout(rect=[0, 0, 1, 0.95])
    fig.savefig(OUT/'trajectory_comparison.png', bbox_inches='tight')
    plt.close(fig)

    # Figure 2: clearance and path error.
    fig, axes = plt.subplots(2, 2, figsize=(12.0, 7.0), dpi=180)
    for col, initial in enumerate(('P0', 'P2')):
        for case in CASES:
            nav, _, _ = load(case, initial)
            t = [r['elapsed'] for r in nav]
            axes[0, col].plot(t, [r['swept_clearance_m'] for r in nav], color=CASES[case]['color'], lw=1.4, label=case)
            axes[1, col].plot(t, [r['path_error'] for r in nav], color=CASES[case]['color'], lw=1.4, label=case)
        axes[0, col].axhline(0.9, color='#d62728', lw=1.0, ls=':')
        axes[0, col].set_title(f'{initial}: swept clearance', fontsize=11, fontweight='bold')
        axes[1, col].set_title(f'{initial}: path error', fontsize=11, fontweight='bold')
        axes[1, col].set_xlabel('simulation time (s)')
        for row in range(2):
            style(axes[row, col])
    axes[0,0].set_ylabel('clearance (m)'); axes[1,0].set_ylabel('path error (m)')
    axes[0,0].legend(fontsize=8, loc='upper right')
    fig.tight_layout()
    fig.savefig(OUT/'tracking_comparison.png', bbox_inches='tight')
    plt.close(fig)

    # Figure 3: current profiles.
    t = np.linspace(0, 150, 601)
    fig, axes = plt.subplots(1, 2, figsize=(12.0, 4.4), dpi=180, sharex=True)
    for case in CASES:
        vals = np.asarray([current_at(case, x) for x in t])
        axes[0].plot(t, vals[:,0], color=CASES[case]['color'], lw=2.0, label=case)
        axes[1].plot(t, vals[:,1], color=CASES[case]['color'], lw=2.0, label=case)
    for ax, title in zip(axes, ('current x (m/s)', 'current z (m/s)')):
        ax.axvspan(60, 150, color='#999999', alpha=.10)
        ax.axhline(0, color='#222222', lw=.8)
        ax.set_title(title, fontsize=11, fontweight='bold')
        ax.set_xlabel('simulation time (s)')
        style(ax)
    axes[0].legend(fontsize=8, loc='lower left')
    fig.suptitle('Current input profiles (measured profile has 60–150 s onset)', fontsize=13, fontweight='bold')
    fig.tight_layout(rect=[0, 0, 1, 0.94])
    fig.savefig(OUT/'current_profiles.png', bbox_inches='tight')
    plt.close(fig)

    # Figure 4: metrics table.
    rows = []
    for case in CASES:
        for initial in ('P0', 'P2'):
            nav, acc, _ = load(case, initial)
            rows.append({
                'case': case, 'initial': initial, 'passed': acc['passed'],
                'arrival_s': acc['arrival_time_s'], 'clearance_m': acc['minimum_clearance_m'],
                'margin_m': acc['minimum_safety_margin_m'], 'rms_m': acc['path_rms_m'],
                'max_m': acc['path_max_m'], 'pitch_limited_pct': 100*sum(bool(r['pitch_limited']) for r in nav)/len(nav),
            })
    with (OUT/'comparison_metrics.csv').open('w', newline='', encoding='utf-8') as f:
        w=csv.DictWriter(f, fieldnames=list(rows[0])); w.writeheader(); w.writerows(rows)
    (OUT/'comparison_metrics.json').write_text(json.dumps(rows, indent=2), encoding='utf-8')

    fig, ax = plt.subplots(figsize=(12.0, 4.8), dpi=180)
    ax.axis('off')
    headers = ['case','initial','arrival s','clearance m','margin m','RMS m','max m','pitch-lim %']
    cells = []
    for r in rows:
        cells.append([r['case'], r['initial'], f"{r['arrival_s']:.2f}", f"{r['clearance_m']:.4f}",
                      f"{r['margin_m']:.4f}", f"{r['rms_m']:.4f}", f"{r['max_m']:.4f}", f"{r['pitch_limited_pct']:.1f}"])
    tab = ax.table(cellText=cells, colLabels=headers, loc='center', cellLoc='center')
    tab.auto_set_font_size(False); tab.set_fontsize(8.5); tab.scale(1, 1.55)
    for (i,j), cell in tab.get_celld().items():
        if i == 0:
            cell.set_facecolor('#343a40'); cell.set_text_props(color='white', weight='bold')
        elif i % 2 == 0:
            cell.set_facecolor('#f1f3f5')
    ax.set_title('S3/F2 current-aware comparison: acceptance metrics', fontsize=13, fontweight='bold', pad=18)
    fig.tight_layout()
    fig.savefig(OUT/'comparison_metrics.png', bbox_inches='tight')
    plt.close(fig)

    # Figure 5: combined dashboard.
    fig = plt.figure(figsize=(13.0, 8.2), dpi=180)
    gs = fig.add_gridspec(2, 2, height_ratios=[1.15, 1.0])
    ax0 = fig.add_subplot(gs[0,0]); ax1 = fig.add_subplot(gs[0,1]); ax2 = fig.add_subplot(gs[1,0]); ax3 = fig.add_subplot(gs[1,1])
    obstacle(ax0); obstacle(ax1)
    for ax, initial in ((ax0,'P0'),(ax1,'P2')):
        ax.plot(path[:,0], path[:,1], color='#222222', lw=1.8, ls='--', label='planned', zorder=3)
        for case in CASES:
            nav,_,_=load(case,initial); xy=np.asarray([r['position_xz'] for r in nav])
            ax.plot(xy[:,0], xy[:,1], color=CASES[case]['color'], lw=1.3, label=case, zorder=4)
        ax.set_title(f'{initial}: trajectory', fontsize=10, fontweight='bold'); ax.set_xlabel('x (m)'); ax.set_ylabel('z (m)')
        ax.set_xlim(4,20); ax.set_ylim(-19,-4); style(ax)
    ax0.legend(fontsize=6.5, loc='lower right')
    for case in CASES:
        nav,_,_=load(case,'P0'); ax2.plot([r['elapsed'] for r in nav],[r['swept_clearance_m'] for r in nav],color=CASES[case]['color'],lw=1.2,label=case)
    ax2.axhline(.9,color='#d62728',lw=.9,ls=':'); ax2.set_title('P0 clearance',fontsize=10,fontweight='bold'); ax2.set_xlabel('t (s)'); ax2.set_ylabel('m'); style(ax2)
    for case in CASES:
        vals=np.asarray([current_at(case,x) for x in t]); ax3.plot(t, vals[:,0], color=CASES[case]['color'], lw=1.6, label=case)
    ax3.axvspan(60,150,color='#999999',alpha=.10); ax3.axhline(0,color='#222222',lw=.7)
    ax3.set_title('current x profile',fontsize=10,fontweight='bold'); ax3.set_xlabel('t (s)'); ax3.set_ylabel('m/s'); style(ax3)
    fig.suptitle('Current-aware G12 comparison dashboard', fontsize=14, fontweight='bold')
    fig.tight_layout(rect=[0,0,1,0.95])
    fig.savefig(OUT/'comparison_dashboard.png', bbox_inches='tight')
    plt.close(fig)

    print('wrote', OUT)

if __name__ == '__main__':
    main()
