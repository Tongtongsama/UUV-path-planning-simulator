"""Static paired figures and audit-ready summary; no rerunning or tuning."""
import argparse
from collections import Counter
import json
from pathlib import Path
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from validation.guidance_matched_comparison import digest
from validation.planner_paired import write


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--input',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args();args.output.mkdir(parents=True,exist_ok=False)
    summary=json.loads((args.input/'summary.json').read_text())
    manifest=json.loads((args.input/'manifest.json').read_text())
    for name,sha in manifest.items():
        if digest(args.input/name)!=sha:raise ValueError('evidence mismatch')
    rows=[r for r in summary['rows'] if r['repeat']==1]
    counts={arm:dict(Counter(r['status'] for r in rows if r['arm']==arm)) for arm in ('P1','APPROACH')}
    details=[]
    for scene in dict.fromkeys(r['scene'] for r in rows):
        fig,axes=plt.subplots(2,2,figsize=(12,8),constrained_layout=True)
        for arm in ('P1','APPROACH'):
            data=json.loads((args.input/'runs'/f'{scene}_{arm}_1.json').read_text())
            states=data['history']['states'];controls=data['history']['controls'];t=[s['timestamp'] for s in states]
            axes[0,0].plot([s['pose']['x'] for s in states],[s['pose']['z'] for s in states],label=arm)
            axes[0,1].plot(t,[s['pose']['pitch'] for s in states],label=arm)
            axes[1,0].plot(t,[s['twist']['q'] for s in states],label=arm)
            axes[1,1].plot([s['time'] for s in controls],[s['applied']['tau_m'] for s in controls],label=arm)
            if data['diagnostics']:
                d=data['diagnostics'];selected=[e for e in d['evaluations'] if e['outcome']=='SELECTED']
                details.append(dict(scene=scene,handoff=d['handoff'],selected=selected,
                    candidate_outcomes=dict(Counter(c['outcome'] for e in d['evaluations'] for c in e['candidates']))))
        axes[0,0].set(xlabel='x [m]',ylabel='z [m]');axes[0,0].set_aspect('equal',adjustable='datalim')
        axes[0,1].set(xlabel='physical time [s]',ylabel='actual pitch [rad]')
        axes[1,0].set(xlabel='physical time [s]',ylabel='actual q [rad/s]')
        axes[1,1].set(xlabel='physical time [s]',ylabel='applied pitch moment [Nm]')
        for ax in axes.flat:ax.grid(alpha=.3);ax.legend()
        fig.suptitle(scene+' — same frozen P1 path / unchanged PID')
        fig.savefig(args.output/f'{scene}.png',dpi=140);plt.close(fig)
    write(args.output/'summary.json',dict(counts=counts,rows=rows,details=details,
        caution='Identical-model zero-current evidence; no robustness or collision-path repair claim.'))
    write(args.output/'manifest.json',{str(p.relative_to(args.output)):digest(p) for p in args.output.rglob('*') if p.is_file()})


if __name__=='__main__':main()
