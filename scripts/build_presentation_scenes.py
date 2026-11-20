"""Build the five required presentation scenes from existing evidence only."""
from pathlib import Path
import json, sys
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.patches import Rectangle, FancyArrowPatch

ROOT=Path(__file__).resolve().parents[1]; sys.path.insert(0,str(ROOT))
OUT=ROOT/'artifacts/presentation_20261119_01'
S1_OB=[[9,10,-20,-8]]
S3_OB=[[6,7,-20,-5.5],[13,14,-14.5,0]]
S4_OB=[[8,10,-11,-9],[4,6,-18,-14],[13,15,-6,-2],[13,15,-18,-14],[3,5,-6,-2]]


def nav(path): return [json.loads(s) for s in (Path(path)/'navigation.jsonl').read_text().splitlines() if s]
def acc(path): return json.loads((Path(path)/'acceptance.json').read_text())
def plan(path): return json.loads((Path(path)/'plan.json').read_text())
def obs(ax, boxes):
    for a,b,c,d in boxes: ax.add_patch(Rectangle((a,c),b-a,d-c,facecolor='#606060',alpha=.65,edgecolor='none',zorder=1))
def style(ax):
    ax.grid(True,alpha=.22,lw=.6); ax.set_axisbelow(True)
    for s in ('top','right'): ax.spines[s].set_visible(False)
def save(fig,name):
    fig.savefig(OUT/name,bbox_inches='tight'); plt.close(fig); print('wrote',name)


def s1():
    p=ROOT/'artifacts/gui_reproduction_formal_20261115_01/S1/F1/P4'
    n=nav(p); a=acc(p); pl=plan(p); path=np.asarray(pl['path']); xy=np.asarray([r['position_xz'] for r in n])
    fig=plt.figure(figsize=(12.5,6.6),dpi=180)
    gs=fig.add_gridspec(2,1,height_ratios=[4.2,1.0])
    ax=fig.add_subplot(gs[0]); ax2=fig.add_subplot(gs[1]); ax2.axis('off')
    obs(ax,S1_OB)
    ax.plot(path[:,0],path[:,1],'--',color='#222',lw=2,label='planned')
    ax.plot(xy[:,0],xy[:,1],color='#1b6ca8',lw=1.8,label='actual')
    ax.scatter([path[0,0]],[path[0,1]],s=40,c='#222'); ax.scatter([path[-1,0]],[path[-1,1]],s=70,c='#222',marker='*')
    ax.set_aspect('equal'); ax.set_xlim(0,20); ax.set_ylim(-20,0); ax.set_xlabel('planning x (m)'); ax.set_ylabel('planning z (m)')
    ax.set_title('S1 basic closed loop: planned vs actual trajectory',fontsize=12,fontweight='bold'); style(ax); ax.legend(fontsize=8,loc='lower right')
    labels=['G12 guide','PID + allocator','xdyn RK4','Gazebo','state feedback']
    x=[.03,.23,.43,.63,.83]
    for i,(lab,xx) in enumerate(zip(labels,x)):
        ax2.add_patch(Rectangle((xx,.25),.15,.45,transform=ax2.transAxes,facecolor='#e8f0f7',edgecolor='#315b7d',lw=1.2))
        ax2.text(xx+.075,.47,lab,ha='center',va='center',fontsize=8.5,transform=ax2.transAxes)
        if i<len(labels)-1: ax2.annotate('',xy=(x[i+1],.47),xytext=(xx+.15,.47),xycoords=ax2.transAxes,arrowprops=dict(arrowstyle='->',lw=1.2,color='#315b7d'))
    ax2.text(.5,.05,'recorded state / command chain: navigation, allocation, solver and Gazebo write-back',ha='center',fontsize=8.5,color='#444',transform=ax2.transAxes)
    fig.suptitle('S1: basic closed-loop capability (not all conditions)',fontsize=13,fontweight='bold')
    save(fig,'s1_basic_closed_loop.png')


def s3():
    p=ROOT/'artifacts/gui_current_v1_20261118_01/S3/F2/P2'; n=nav(p); a=acc(p); pl=plan(p); path=np.asarray(pl['path']); xy=np.asarray([r['position_xz'] for r in n])
    mn=min(n,key=lambda r:r['swept_clearance_m']); mxz=mn['position_xz']
    fig,axs=plt.subplots(1,2,figsize=(12.5,4.8),dpi=180)
    ax=axs[0]; obs(ax,S3_OB); ax.plot(path[:,0],path[:,1],'--',color='#222',lw=1.8,label='planned'); ax.plot(xy[:,0],xy[:,1],color='#1b6ca8',lw=1.6,label='actual')
    ax.scatter([mxz[0]],[mxz[1]],c='#d62728',s=45,zorder=6); ax.set_aspect('equal'); ax.set_xlim(4,20); ax.set_ylim(-19,-4)
    ax.set_title(f'S3/F2/P2: full trajectory; min {a["minimum_clearance_m"]:.3f} m',fontsize=11,fontweight='bold'); ax.set_xlabel('x (m)'); ax.set_ylabel('z (m)'); style(ax); ax.legend(fontsize=8,loc='lower right')
    ins=ax.inset_axes([.43,.12,.5,.42]); obs(ins,S3_OB); ins.plot(xy[:,0],xy[:,1],color='#1b6ca8',lw=1.3); ins.scatter([mxz[0]],[mxz[1]],c='#d62728',s=35); ins.set_xlim(mxz[0]-1.5,mxz[0]+1.5); ins.set_ylim(mxz[1]-1.5,mxz[1]+1.5); ins.tick_params(labelsize=6); ins.set_title('near-obstacle zoom',fontsize=7)
    ax2=axs[1]; t=[r['elapsed'] for r in n]; ax2.plot(t,[r['swept_clearance_m'] for r in n],color='#1b6ca8',lw=1.6,label='clearance'); ax2.axhline(.9,color='#d62728',ls=':',lw=1.1); ax2.axvline(mn['elapsed'],color='#d62728',ls='--',lw=.9)
    ax2.set_xlabel('simulation time (s)'); ax2.set_ylabel('clearance (m)'); ax2.set_title('S3/F2/P2: clearance and path error',fontsize=11,fontweight='bold'); style(ax2)
    ax3=ax2.twinx(); ax3.plot(t,[r['path_error'] for r in n],color='#e67e22',lw=1.1,label='path error'); ax3.set_ylabel('path error (m)'); ax3.spines['top'].set_visible(False)
    h1,l1=ax2.get_legend_handles_labels(); h2,l2=ax3.get_legend_handles_labels(); ax2.legend(h1+h2,l1+l2,fontsize=8,loc='upper right')
    fig.text(0.5,0.01,f'min {a["minimum_clearance_m"]:.4f} m; margin {a["minimum_safety_margin_m"]*1000:.1f} mm; threshold 0.9 m',ha='center',fontsize=8.0,color='#444')
    fig.suptitle('S3/F2/P2: narrow-margin obstacle avoidance (full trajectory, not hidden)',fontsize=13,fontweight='bold')
    save(fig,'s3_narrow_margin.png')


def s4():
    base=ROOT/'artifacts/twin_axial_s4_planning_20261116_04'; dplan=ROOT/'artifacts/twin_axial_s4_terminal_candidate_20261117_01'
    labels={'A_raw_astar':'A: raw A*','B_safe_shortcut':'B: safe shortcut','C_safe_smoothing':'C: rounded/smoothing'}
    fig,axs=plt.subplots(1,2,figsize=(13.0,5.0),dpi=180)
    ax=axs[0]; obs(ax,S4_OB)
    colors={'A_raw_astar':'#7f7f7f','B_safe_shortcut':'#1f77b4','C_safe_smoothing':'#2ca02c'}
    for key,lab in labels.items():
        pl=json.loads((base/key/'plan.json').read_text()); path=np.asarray(pl['path']); ax.plot(path[:,0],path[:,1],color=colors[key],lw=1.4,label=lab)
    d=json.loads((dplan/'plan.json').read_text()); dp=np.asarray(d['path']); ax.plot(dp[:,0],dp[:,1],color='#d62728',lw=2.0,label='D: 4 m terminal segment')
    ax.set_aspect('equal'); ax.set_xlim(0,20); ax.set_ylim(-20,0); ax.set_xlabel('x (m)'); ax.set_ylabel('z (m)'); ax.set_title('S4 planning methods on the same map',fontsize=11,fontweight='bold'); style(ax); ax.legend(fontsize=7.5,loc='lower right')
    ax2=axs[1]; obs(ax2,S4_OB); ax2.plot(dp[:,0],dp[:,1],'--',color='#222',lw=1.6,label='D planned')
    rp=ROOT/'artifacts/twin_axial_s4_terminal_real_pilot_20261118_01/real/S4/F2/P0'
    rn=nav(rp); xy=np.asarray([r['position_xz'] for r in rn]); ax2.plot(xy[:,0],xy[:,1],color='#d62728',lw=1.7,label='D real F2/P0 (pilot)')
    ax2.set_aspect('equal'); ax2.set_xlim(0,20); ax2.set_ylim(-20,0); ax2.set_xlabel('x (m)'); ax2.set_ylabel('z (m)'); ax2.set_title('D: planned path + real F2/P0 pilot trajectory',fontsize=11,fontweight='bold'); style(ax2); ax2.legend(fontsize=7.5,loc='lower right')
    ax2.text(.02,.92,'A/B/C: F0/P0 Python ablation; D: independent candidate; five real pilots passed; not an 18-case matrix',transform=ax2.transAxes,fontsize=7.2,color='#8b1a1a',bbox=dict(facecolor='white',edgecolor='none',alpha=.85),va='top')
    fig.suptitle('S4: path geometry matters beyond finding a path',fontsize=13,fontweight='bold')
    save(fig,'s4_planning_comparison.png')


def failure():
    p=ROOT/'artifacts/gui_current_v2_20261118_03/S3/F2/P0'; n=nav(p); xy=np.asarray([r['position_xz'] for r in n]); last=n[-1]
    fig,axs=plt.subplots(1,2,figsize=(12.5,4.8),dpi=180)
    ax=axs[0]; obs(ax,S3_OB); ax.plot(xy[:,0],xy[:,1],color='#d62728',lw=1.7,label='v2 failed run'); ax.scatter([xy[-1,0]],[xy[-1,1]],marker='X',s=90,c='#8b0000',zorder=6,label='stop point')
    ax.set_aspect('equal'); ax.set_xlim(4,20); ax.set_ylim(-19,-4); ax.set_xlabel('x (m)'); ax.set_ylabel('z (m)'); ax.set_title('Historical failure: trajectory stops at recorded point',fontsize=11,fontweight='bold'); style(ax); ax.legend(fontsize=8)
    ax.text(.02,.13,f't={last["elapsed"]:.2f} s; clearance={last["swept_clearance_m"]:.3f} m; failure: Swept safety clearance',transform=ax.transAxes,fontsize=7.5,color='#8b0000',bbox=dict(facecolor='white',edgecolor='none',alpha=.85))
    ax2=axs[1]; ax2.axis('off')
    ax2.text(.02,.94,'Gate / precheck refusals',fontsize=12,fontweight='bold',va='top')
    txt=('Strong-current boundary (historical):\n'
         '  U10/U15/U20  real_started=false\n'
         '  classification: python_gate_refusal\n'
         '  → precheck refused; no dynamic trajectory\n\n'
         'S4 full gate (2026-11-18):\n'
         '  RuntimeError: Full gate rejected\n'
         '  formal_trials_started=false\n'
         '  → independent candidate not released\n\n'
         'Do not draw refusals as zero trajectories or as collisions.')
    ax2.text(.02,.82,txt,fontsize=8.5,va='top',linespacing=1.5,family='monospace')
    ax2.add_patch(Rectangle((.01,.02),.98,.22,transform=ax2.transAxes,facecolor='#fdecea',edgecolor='#d62728',lw=1.0))
    ax2.text(.03,.12,'Failure ≠ dynamic collision ≠ gate refusal. All three are preserved separately.',fontsize=8.2,color='#8b0000',transform=ax2.transAxes)
    fig.suptitle('Failure and gate boundary: the system can refuse safely',fontsize=13,fontweight='bold')
    save(fig,'failure_and_gate.png')


def fusion():
    dplan=ROOT/'artifacts/twin_axial_s4_terminal_candidate_20261117_01'; d=json.loads((dplan/'plan.json').read_text()); path=np.asarray(d['path'])
    fig,axs=plt.subplots(1,2,figsize=(12.5,4.8),dpi=180)
    ax=axs[0]; obs(ax,S4_OB); ax.plot(path[:,0],path[:,1],color='#6a3d9a',lw=2.2,label='D: 4 m terminal path'); ax.set_aspect('equal'); ax.set_xlim(0,20); ax.set_ylim(-20,0); ax.set_xlabel('x (m)'); ax.set_ylabel('z (m)'); ax.set_title('Proposed final sample: S4 D path',fontsize=11,fontweight='bold'); style(ax); ax.legend(fontsize=8)
    ax2=axs[1]; t=np.linspace(0,150,601); from ros2.twin_axial_g12_current_v10.schedule import current
    v=np.asarray([current(x) for x in t]); ax2.plot(t,v[:,0],color='#3b5bdb',lw=2.0,label='measured v10 east'); ax2.axhline(0,color='#222',lw=.7); ax2.axvspan(60,150,color='#999',alpha=.10)
    ax2.set_xlabel('simulation time (s)'); ax2.set_ylabel('current x (m/s)'); ax2.set_title('Measured current input (6 samples, 60–150 s onset)',fontsize=11,fontweight='bold'); style(ax2); ax2.legend(fontsize=8)
    fig.text(0.5,0.01,'Composition proposal only: no single S4+measured-current real trial has been run.',ha='center',fontsize=8.0,color='#8b1a1a')
    fig.suptitle('Final experimental sample composition: new path + real current (not yet executed)',fontsize=13,fontweight='bold')
    save(fig,'final_sample_fusion.png')


if __name__=='__main__':
    for fn in (s1,s3,s4,failure,fusion): fn()
