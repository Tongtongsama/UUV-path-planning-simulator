"""Plot verified S2/P0 F0/F1/F2 xdyn trajectories, including stopped runs."""
import argparse
import hashlib
import json
import math
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle

from core import Pose
from ros2.twin_axial_navigation.mission import environment_from_world,segment_clearance


def load(directory):
    directory=Path(directory).resolve()
    manifest=json.loads((directory/'manifest.json').read_text(encoding='utf-8'))
    for name,expected in manifest.items():
        path=(directory/name).resolve()
        if not path.is_relative_to(directory) or not path.is_file() or hashlib.sha256(path.read_bytes()).hexdigest()!=expected:
            raise ValueError(f'Recording hash mismatch: {name}')
    metadata=json.loads((directory/'metadata.json').read_text(encoding='utf-8'))
    plan=json.loads((directory/'plan.json').read_text(encoding='utf-8'))
    status=json.loads((directory/'run_status.json').read_text(encoding='utf-8'))
    acceptance=json.loads((directory/'acceptance.json').read_text(encoding='utf-8'))
    if metadata['scene']!='S2' or metadata['case']!='P0' or plan['config']['scene']!='double_baffle_scaled_v1':
        raise ValueError('Expected S2/P0 recording')
    flow=metadata.get('flow','F0')
    if flow not in ('F0','F1','F2'): raise ValueError('Unknown flow')
    records=[json.loads(line) for line in (directory/'transactions.jsonl').read_text(encoding='utf-8').splitlines() if line]
    records=[r for r in records if r.get('valid')]
    if not records: raise ValueError('No valid xdyn states')
    samples=[(r['elapsed_end'],r['position_planning_xz_m'][0],r['position_planning_xz_m'][1]) for r in records]
    world=environment_from_world((directory/'worlds/minimal.world').read_text(encoding='utf-8'))
    previous=Pose(x=2,z=-10)
    clearances=[]
    for _,x,z in samples:
        point=Pose(x=x,z=z)
        clearances.append(segment_clearance(world,previous,point))
        previous=point
    return {'flow':flow,'directory':str(directory),'samples':samples,'plan':plan,
            'status':status['status'],'error':status['error'],'accepted':acceptance['passed'],
            'minimum_clearance_m':min(clearances),'end_time_s':samples[-1][0]}


def compare(folders,output):
    output=Path(output).resolve()
    if output.exists(): raise FileExistsError(output)
    runs={run['flow']:run for run in map(load,folders)}
    if set(runs)!={'F0','F1','F2'}: raise ValueError('Exactly one F0/F1/F2 recording required')
    output.mkdir(parents=True)
    baseline={round(t,6):(x,z) for t,x,z in runs['F0']['samples']}
    summary={'scope':'S2/P0 fixed controller; xdyn state trajectories; F2 is truncated at safety stop','runs':{}}
    for flow,run in runs.items():
        shared=[(math.hypot(x-baseline[round(t,6)][0],z-baseline[round(t,6)][1]))
                for t,x,z in run['samples'] if round(t,6) in baseline]
        summary['runs'][flow]={'source':run['directory'],'status':run['status'],'accepted':run['accepted'],
            'error':run['error'],'end_time_s':run['end_time_s'],
            'minimum_clearance_m':run['minimum_clearance_m'],
            'max_position_difference_from_F0_m':max(shared) if shared else None,
            'matched_steps':len(shared),'endpoint_xz_m':list(run['samples'][-1][1:])}
    (output/'comparison.json').write_text(json.dumps(summary,indent=2),encoding='utf-8')
    fig,ax=plt.subplots(figsize=(9,8),layout='constrained')
    for i,(xmin,xmax,zmin,zmax) in enumerate(runs['F0']['plan']['config']['obstacles']):
        ax.add_patch(Rectangle((xmin,zmin),xmax-xmin,zmax-zmin,facecolor='#77818e',alpha=.72,label='Baffles' if i==0 else None))
        ax.add_patch(Rectangle((xmin-.9,zmin-.9),xmax-xmin+1.8,zmax-zmin+1.8,
                               fill=False,edgecolor='#ba5c5c',linestyle=':',linewidth=1.2,
                               label='0.9 m safety line' if i==0 else None))
    path=runs['F0']['plan']['path']
    ax.plot([p[0] for p in path],[p[1] for p in path],color='#338264',linestyle='--',linewidth=1.2,label='Planned path')
    colors={'F0':'#244ca0','F1':'#e38a23','F2':'#ba347d'}
    labels={'F0':'F0 no current','F1':'F1 current (-0.05, 0) m/s','F2':'F2 current (0, +0.05) m/s'}
    for flow in ('F0','F1','F2'):
        run=runs[flow];coords=run['samples'];color=colors[flow]
        ax.plot([p[1] for p in coords],[p[2] for p in coords],color=color,linewidth=2,label=labels[flow])
        ax.scatter(coords[-1][1],coords[-1][2],color=color,marker='X' if not run['accepted'] else 'o',s=95,zorder=5)
    ax.scatter([2,18],[-10,-10],color=['#111827','#3c9d56'],marker='*',s=130,zorder=6)
    ax.set(xlim=(0,20),ylim=(-20,0),xlabel='Planning x (m)',ylabel='Planning z (m)',
           title='S2/P0: identical guidance and PID, different constant currents')
    ax.set_aspect('equal');ax.grid(alpha=.2);ax.legend(loc='upper left',fontsize=8)
    fig.savefig(output/'trajectory.png',dpi=180)
    fig.savefig(output/'trajectory.pdf')
    plt.close(fig)
    return summary


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('f0',type=Path);parser.add_argument('f1',type=Path);parser.add_argument('f2',type=Path)
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    print(json.dumps(compare((args.f0,args.f1,args.f2),args.output),indent=2))


if __name__=='__main__':main()
