"""Read-only extraction of frozen S4 evidence for dissertation figures."""
from pathlib import Path
import csv, hashlib, json
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle

ROOT=Path(__file__).resolve().parents[2]
OUT=Path(__file__).parent
A=ROOT/'artifacts/twin_axial_s4_planning_20261116_04'
B=ROOT/'artifacts/twin_axial_s4_validation_20261117_01'
D=ROOT/'artifacts/twin_axial_s4_terminal_validation_20261117_01'
R=ROOT/'artifacts/twin_axial_s4_terminal_real_pilot_20261118_01/real/S4'
names=['A_raw_astar','B_safe_shortcut','C_safe_smoothing']
labels=['A: raw A*','B: safe shortcut','C: safe smoothing','D: B + 4 m terminal']
colors=['#525d6a','#007f86','#e09b34','#8d4fb3']
obstacles=[(8,10,-11,-9),(4,6,-18,-14),(13,15,-6,-2),(13,15,-18,-14),(3,5,-6,-2)]
used=[]
def load(path):
    used.append(path)
    return json.loads(path.read_text(encoding='utf-8'))
def states(path):
    used.append(path)
    with path.open(encoding='utf-8') as f: return [json.loads(s) for s in f]
def save(fig,name):
    fig.savefig(OUT/(name+'.svg'),bbox_inches='tight')
    fig.savefig(OUT/(name+'.png'),dpi=180,bbox_inches='tight')
    plt.close(fig)
def axes_scene(ax):
    for x0,x1,z0,z1 in obstacles:
        ax.add_patch(Rectangle((x0,z0),x1-x0,z1-z0,facecolor='#d8dce0',edgecolor='#454b54',lw=.8))
    ax.scatter([2,18],[-10,-10],c=['#13874a','#b43f42'],s=26,zorder=5)
    ax.set(xlim=(0,20),ylim=(-20,0),xlabel='x (m)',ylabel='z (m)')
    ax.set_aspect('equal',adjustable='box');ax.grid(alpha=.18)

plans=[load(A/n/'plan.json') for n in names]+[load(D/'F0/P0/plan.json')]
reports=[load(A/n/'report.json') for n in names]+[load(D/'F0/P0/report.json')]
assert all(p['scene']=='S4' and p['flow']=='F0' and p['case']=='P0' for p in plans)
assert all(r['passed'] for r in reports)
assert all(p['geometry']['min_clearance_m']>=1.5 for p in plans)

fig,axs=plt.subplots(2,2,figsize=(10,8),layout='constrained')
for i,ax in enumerate(axs.flat):
    axes_scene(ax);xy=plans[i]['path'];ax.plot([v[0] for v in xy],[v[1] for v in xy],color=colors[i],lw=1.6)
    ax.set_title(f'{labels[i]} | {plans[i]["geometry"]["length_m"]:.2f} m')
    if i==3:ax.annotate('4 m level approach',xy=(16,-10),xytext=(11,-7),arrowprops={'arrowstyle':'->'})
save(fig,'01_s4_geometry_paths')

fig,axs=plt.subplots(1,2,figsize=(11,4.4),layout='constrained')
for i,n in enumerate(names+['D']):
    rows=states((A/n if i<3 else D/'F0/P0')/'states.jsonl')
    x=[v['state'][0] for v in rows];z=[v['state'][1] for v in rows]
    axes_scene(axs[0]);axs[0].plot(x,z,color=colors[i],lw=1.4,label=labels[i])
    axs[1].plot([v['t'] for v in rows],[v['nav']['path_error'] for v in rows],color=colors[i],lw=1,label=labels[i])
axs[0].legend(fontsize=8,loc='lower left');axs[0].set_title('F0/P0 Python actual trajectories')
axs[1].axhline(.8,color='#b43f42',ls='--',lw=.8,label='0.8 m max-error limit')
axs[1].set(xlabel='simulation time (s)',ylabel='path error (m)',title='Original path-error samples')
axs[1].grid(alpha=.2);axs[1].legend(fontsize=8)
save(fig,'02_s4_f0_python_closed_loop')

fig,axs=plt.subplots(1,2,figsize=(11,4.3),layout='constrained')
old=states(B/'F2/P0/states.jsonl');new=states(D/'F2/P0/states.jsonl')
old_report=load(B/'F2/P0/report.json');new_report=load(D/'F2/P0/report.json')
assert not old_report['passed'] and new_report['passed']
for ax in axs:axes_scene(ax)
for ax,rows,p,label,color in [(axs[0],old,load(B/'F2/P0/plan.json'),'B: F2/P0 failed',colors[1]),
                              (axs[1],new,load(D/'F2/P0/plan.json'),'D: F2/P0 passed',colors[3])]:
    path=p['path'];ax.plot([v[0] for v in path],[v[1] for v in path],ls='--',color='#555',lw=.9,label='reference')
    ax.plot([v['state'][0] for v in rows],[v['state'][1] for v in rows],color=color,lw=1.5,label='Python dynamics')
    ax.scatter(rows[-1]['state'][0],rows[-1]['state'][1],marker='x' if label.startswith('B:') else 'o',
               color='#b43f42' if label.startswith('B:') else '#13874a',s=45,zorder=5)
    ax.set_title(label);ax.legend(fontsize=8,loc='lower left')
save(fig,'03_s4_f2_terminal_failure_comparison')

fig,axs=plt.subplots(1,3,figsize=(13,4.5),layout='constrained')
for ax,flow in zip(axs,['F0','F1','F2']):
    axes_scene(ax)
    py=states(D/flow/'P0'/'states.jsonl')
    real=states(R/flow/'P0'/'navigation.jsonl')
    ax.plot([v['state'][0] for v in py],[v['state'][1] for v in py],color=colors[3],lw=1.7,label='Python reference')
    ax.plot([v['position_xz'][0] for v in real],[v['position_xz'][1] for v in real],color='#202c39',ls='--',lw=1.1,label='xdyn GUI')
    ax.set_title(f'D: S4/{flow}/P0');ax.legend(loc='lower left',fontsize=8)
save(fig,'04_s4_terminal_python_real_overlay')

with (OUT/'geometry_and_f0_python.csv').open('w',newline='',encoding='utf-8') as f:
    w=csv.writer(f);w.writerow(['scheme','backend','flow','case','length_m','turn_count','total_turn_deg','max_discrete_turn_deg','planned_clearance_m','arrival_s','actual_clearance_m','rms_m','max_error_m','passed'])
    for i in range(4):
        g=plans[i]['geometry'];r=reports[i];m=r['metrics']
        w.writerow([labels[i],'Python 3-DoF','F0','P0',g['length_m'],g['turn_count'],g['total_turn_deg'],g['max_turn_deg'],g['min_clearance_m'],m['arrival_time'],m['min_clearance'],m['rms_error'],m['max_error'],r['passed']])
with (OUT/'terminal_f2_comparison.csv').open('w',newline='',encoding='utf-8') as f:
    w=csv.writer(f);w.writerow(['scheme','backend','flow','case','passed','arrival_s','actual_clearance_m','rms_m','max_error_m','failure'])
    for label,r in [('B',old_report),('D',new_report)]:
        m=r['metrics'];w.writerow([label,'Python 3-DoF','F2','P0',r['passed'],m.get('arrival_time'),m.get('min_clearance'),m.get('rms_error'),m.get('max_error'),r.get('failure')])
with (OUT/'real_five_pilots.csv').open('w',newline='',encoding='utf-8') as f:
    w=csv.writer(f);w.writerow(['scene','flow','case','backend','passed','arrival_s','clearance_m','rms_m','max_error_m','all_original_checks'])
    for flow,case in [('F0','P0'),('F1','P0'),('F2','P0'),('F2','P2'),('F2','P4')]:
        r=load(R/flow/case/'acceptance.json');assert r['passed'] and all(r['checks'].values())
        w.writerow(['S4',flow,case,'LOTUSim/xdyn GUI',r['passed'],r['arrival_time_s'],r['minimum_clearance_m'],r['path_rms_m'],r['path_max_m'],True])
manifest={p.relative_to(ROOT).as_posix():hashlib.sha256(p.read_bytes()).hexdigest() for p in sorted(set(used))}
(OUT/'input_sha256.json').write_text(json.dumps(manifest,indent=2),encoding='utf-8')
print('Created four figures, three tables, and input hash manifest from',len(manifest),'source files')
