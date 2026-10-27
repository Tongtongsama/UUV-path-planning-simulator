"""Offline scientific plots from a completed replay recording (matplotlib)."""
import argparse
import csv
import hashlib
import json
from pathlib import Path


def rows(path):
    with path.open(encoding='utf-8',newline='') as f: return list(csv.DictReader(f))


def main(argv=None):
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('directory',type=Path)
    args=p.parse_args(argv); base=args.directory
    manifest=json.loads((base/'manifest.json').read_text(encoding='utf-8'))
    for name,digest in manifest.items():
        file=(base/name).resolve()
        if not file.is_relative_to(base.resolve()) or hashlib.sha256(file.read_bytes()).hexdigest()!=digest:
            raise ValueError('Recording hash mismatch: '+name)
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.patches import Rectangle
    env=json.loads((base/'environment.json').read_text(encoding='utf-8'))
    meta=json.loads((base/'metadata.json').read_text(encoding='utf-8'))
    observed=rows(base/'gazebo_poses.csv')
    sent=rows(base/'commands.csv')
    fig,axes=plt.subplots(3,2,figsize=(13,13))
    traj=axes[0,0]
    for o in env['obstacles']:
        traj.add_patch(Rectangle((o['x_min'],o['z_min']),o['x_max']-o['x_min'],o['z_max']-o['z_min'],color='gray',alpha=.4))
    traj.plot(*zip(*env['path']),color='green',linestyle=':',label='Planned path')
    for arm in meta['settings']['arms']:
        states=rows(base/f'{arm}_states.csv'); controls=rows(base/f'{arm}_controls.csv')
        def series(data,key): return [float(r[key]) for r in data]
        traj.plot(series(states,'x_m'),series(states,'z_m'),label=arm+' frozen')
        samples=[r for r in observed if r['arm']==arm]
        if samples: traj.plot(series(samples,'x'),series(samples,'z'),'--',label=arm+' Gazebo')
        for ax,key,label in [(axes[0,1],'pitch_rad','Pitch [rad]'),(axes[1,0],'q_radps','Pitch rate [rad/s]')]:
            ax.plot(series(states,'time_s'),series(states,key),label=arm)
            ax.set(xlabel='Historical time [s]',ylabel=label)
        for ax,key,label in [(axes[1,1],'applied_tau_x_N','Applied surge force [N]'),(axes[2,0],'applied_tau_m_Nm','Applied pitch moment [Nm]')]:
            ax.step(series(controls,'time_s'),series(controls,key),where='post',label=arm)
            ax.set(xlabel='Historical time [s]',ylabel=label)
        commands=[r for r in sent if r['arm']==arm]
        axes[2,1].plot(series(commands,'receipt_wall_s'),series(commands,'source_time_s'),label=arm)
    traj.set(xlabel='ENU x [m]',ylabel='ENU z [m]',title='Environment and trajectories'); traj.set_aspect('equal')
    b=env['boundary']; traj.set_xlim(b['x_min']-.5,b['x_max']+.5); traj.set_ylim(b['z_min']-.5,b['z_max']+.5)
    axes[2,1].set(xlabel='Recording wall time [s]',ylabel='Command historical time [s]',title='Playback timing (not tracking error)')
    for ax in axes.flat: ax.grid(alpha=.2); ax.legend(fontsize=8)
    fig.suptitle('Frozen-history replay: controller curves are historical, Gazebo poses are observed\n'+('Gazebo telemetry available' if observed else 'WARNING: NO GAZEBO TELEMETRY'))
    fig.tight_layout()
    destination=base/'plots'
    destination.mkdir(exist_ok=False)
    fig.savefig(destination/'recording.png',dpi=160)
    fig.savefig(destination/'recording.pdf')
    plt.close(fig)
    print(destination)


if __name__=='__main__': main()
