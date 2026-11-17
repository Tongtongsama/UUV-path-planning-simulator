"""Recorded-state terminal audit only: no integrator, controller or simulator calls."""
import argparse
import hashlib
import json
import math
from pathlib import Path
import numpy as np
from core import Path as GeometricPath,Pose
from planner.candidates import simplify_path

ROOT=Path(__file__).resolve().parents[2]
SOURCE=ROOT/'artifacts/twin_axial_s4_validation_20261117_01'
def read(p):return json.loads(p.read_text(encoding='utf-8'))
def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
def write(p,v):
    with p.open('x',encoding='utf-8') as f:json.dump(v,f,indent=2,ensure_ascii=False)

def sample(r):
    x,z,theta,u,w,q=r['state'];nav=r['nav']
    vx=u*math.cos(theta)+w*math.sin(theta);vz=u*math.sin(theta)-w*math.cos(theta)
    return dict(t=r['t'],x=x,z=z,goal_distance=math.hypot(x-18,z+10),u=u,w=w,
        pitch_deg=math.degrees(theta),q=q,vx=vx,vz=vz,ground_speed=math.hypot(vx,vz),
        ground_course_deg=math.degrees(math.atan2(vz,vx)),target_speed=nav['target_speed'],
        target_pitch_deg=math.degrees(nav['target_pitch']),desired_course_deg=math.degrees(nav['desired_course']),
        uncompensated_pitch_deg=math.degrees(nav['uncompensated_limit_pitch']),pitch_limited=nav['pitch_limited'],
        bend_preview=nav['bend_preview'],path_error=nav['path_error'],clearance=r['clearance'])

def main(out):
    out.mkdir(parents=True,exist_ok=False)
    manifest=read(SOURCE/'final_manifest.json')
    assert all(sha(SOURCE/n)==h for n,h in manifest.items()),'Input record drift'
    protected={str(SOURCE/n):h for n,h in manifest.items()}
    frozen=read(SOURCE/'source_manifest.json')
    assert all(sha(ROOT/n)==h and sha(SOURCE/'source'/n)==h for n,h in frozen.items())
    protected.update({str(ROOT/n):h for n,h in frozen.items()})
    write(out/'input_manifest.json',protected)
    result={};paths={}
    for flow in ('F0','F1','F2'):
        for case in ('P0','P1','P2','P3','P4','P0_repeat'):
            key=f'{flow}/{case}';folder=SOURCE/key;plan=read(folder/'plan.json')
            rows=[json.loads(s) for s in (folder/'states.jsonl').read_text(encoding='utf-8').splitlines()]
            samples=[sample(r) for r in rows]
            simple=simplify_path(GeometricPath.from_list([Pose(x=x,z=z) for x,z in plan['path']]))
            anchors=[[p.x,p.z] for p in simple];a,b=anchors[-2:]
            events={}
            for name,predicate in [('enter_1p5',lambda s:s['goal_distance']<1.5),
                ('enter_0p5',lambda s:s['goal_distance']<=.5),('enter_0p3',lambda s:s['goal_distance']<.3),
                ('cross_goal_x',lambda s:s['x']>=18),('first_pitch_limit_after_x15',lambda s:s['x']>=15 and s['pitch_limited'])]:
                events[name]=next((s for s in samples if predicate(s)),None)
            events['closest']=min(samples,key=lambda s:s['goal_distance']);events['stop']=samples[-1]
            window=[s for s in samples if s['x']>=15]
            # Independently check recorded ground kinematics against finite differences.
            residual=[]
            for previous,current in zip(samples,samples[1:]):
                dt=current['t']-previous['t']
                residual.append(math.hypot((current['x']-previous['x'])/dt-(current['vx']+previous['vx'])/2,
                                           (current['z']-previous['z'])/dt-(current['vz']+previous['vz'])/2))
            result[key]={'events':events,'terminal_anchors':anchors,'last_straight_length_m':math.dist(a,b),
                'last_straight_angle_deg':math.degrees(math.atan2(b[1]-a[1],b[0]-a[0])),
                'x15_window':{'steps':len(window),'pitch_limit_steps':sum(s['pitch_limited'] for s in window),
                    'max_abs_pitch_deg':max(abs(s['pitch_deg']) for s in window)},
                'max_fd_ground_velocity_residual_mps':max(residual),
                'report_checks':read(folder/'report.json')['checks']}
            paths[key]=plan['path']
            if case=='P0':write(out/(flow+'_terminal_samples.json'),window)
    same={case:paths['F0/'+case]==paths['F1/'+case]==paths['F2/'+case] for case in ('P0','P1','P2','P3','P4','P0_repeat')}
    write(out/'audit.json',{'entries':result,'identical_paths_across_flows':same,'new_simulation_started':False})
    changed=[n for n,h in protected.items() if sha(Path(n))!=h]
    write(out/'preservation.json',{'passed':not changed,'checked_files':len(protected),'differences':changed})
    for f in ('F0','F1','F2'):
        v=result[f+'/P0'];print(f,json.dumps(v,ensure_ascii=False))

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('output');a=p.parse_args();main(Path(a.output).resolve())
