"""Follow-up interaction and numerical checks for a completed primary scan."""
from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path
import time

from .run import HERE, fixture, run_case, write_case


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    output=args.output.resolve()
    if not (output/'run_summary.json').exists():raise ValueError('primary scan missing')
    config=json.loads((HERE/'config.json').read_text(encoding='utf-8'))
    grid=[]; started=time.perf_counter()
    for i,alpha in enumerate((0.,.125,.25,.375,.5)):
        for j,shift in enumerate((-.8,-.4,0.,.35,.7)):
            name=f'G_{i}_{j}'
            summary,rows,_=run_case(alpha,shift,config)
            write_case(output,name,summary,rows)
            grid.append(summary)
            print(name,summary['status'],round(summary['d_min_m'],4),flush=True)
    checks=[]
    for name,alpha,shift in [('identity',1.,0.),('large_bump',0.,0.),('borderline',.5,0.)]:
        summary,rows,_=run_case(alpha,shift,config,dt=config['dt_s']/2)
        write_case(output,'half_dt_'+name,summary,rows)
        checks.append(dict(name=name,half_dt=summary))
    original={name:json.loads((output/(('identity' if name=='identity' else 'E1_00' if name=='large_bump' else 'E1_08')+'.json')).read_text(encoding='utf-8'))
              for name in ('identity','large_bump','borderline')}
    if any(x['half_dt']['status']!=original[x['name']]['status'] for x in checks):
        raise AssertionError('half-step outcome changed')
    terminal=[]
    for name,options in [('terminal_hold',{}),
                         ('existing_capture',{'capture':True}),
                         ('open_loop_zero',{'zero_after_reference':True})]:
        summary,rows,_=run_case(0.,0.,config,**options)
        write_case(output,'E3_'+name,summary,rows)
        terminal.append(summary)
        print('E3_'+name,summary['status'],round(summary['d_min_m'],4),flush=True)
    # Verify the transformed reference remains inside the planning world's
    # circular-proxy envelope, including the chords between saved steps.
    _,world,_,constraints=fixture()
    reference_checks=[]
    for name in ('E1_00','E1_08','E2_00','E2_20'):
        with (output/f'{name}.csv').open(encoding='utf-8',newline='') as handle:
            rows=list(csv.DictReader(handle))
        positions=[(float(r['ref_x_m']),float(r['ref_z_m'])) for r in rows]
        gaps=[world.segment_clearance(a,b) for a,b in zip(positions,positions[1:])]
        bounds=[world.boundary.contains_segment(a,b,constraints.required_clearance) for a,b in zip(positions,positions[1:])]
        reference_checks.append(dict(name=name,minimum_center_chord_distance_m=min(gaps),
                                     minimum_proxy_hull_chord_clearance_m=min(gaps)-constraints.vehicle_radius,
                                     all_chords_in_bounds=all(bounds)))
    for name in ('identity','E1_00','E1_08','E2_00','E2_20'):
        summary=json.loads((output/f'{name}.json').read_text(encoding='utf-8'))
        if summary['status']=='COLLISION' and summary['d_min_m']>1e-12:
            raise AssertionError('safety violation must have nonpositive signed gap')
        if summary['success'] and not summary['exit_ok']:
            raise AssertionError('success without exit conditions')
    follow=dict(grid=grid,half_dt_checks=checks,terminal=terminal,reference_checks=reference_checks,
                wall_time_s=time.perf_counter()-started)
    (output/'followup_summary.json').write_text(json.dumps(follow,indent=2,default=float),encoding='utf-8')


if __name__=='__main__':main()
