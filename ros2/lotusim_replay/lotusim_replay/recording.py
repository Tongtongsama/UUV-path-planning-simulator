"""Append-only replay telemetry; frozen controls are not live controller output."""
import csv
import hashlib
import json
import time
import os
import platform
from dataclasses import asdict
from datetime import datetime, timezone
from pathlib import Path


class Recorder:
    def __init__(self, directory, payload, scene, arms, settings, sdf):
        self.path = Path(directory)
        self.path.mkdir(parents=True, exist_ok=False)
        self.started = time.monotonic()
        self.closed = False
        self.counts = {'commands': 0, 'observations': 0}
        self.streams = []
        self.write_json('metadata.json', {
            'schema': 1, 'kind': 'frozen_history_gazebo_replay',
            'started_utc': datetime.now(timezone.utc).isoformat(),
            'settings': settings, 'source_commit': payload.get('source_commit'),
            'runtime': {'python':platform.python_version(),'platform':platform.platform(),
                        'ROS_DOMAIN_ID':os.environ.get('ROS_DOMAIN_ID','0'),
                        'RMW_IMPLEMENTATION':os.environ.get('RMW_IMPLEMENTATION','default')},
            'source_hashes': payload.get('sources'),
            'clock_contract': 'source_time is historical; receipt_wall_s is monotonic elapsed; gazebo_time_s is message stamp. No lockstep or exact sample pairing.',
            'control_contract': 'Historical applied forces/moments, not a live controller or Gazebo dynamics measurement.',
        })
        self.write_json('environment.json', {k:v for k,v in scene.items() if k != 'arms'})
        self.write_json('historical_results.json', {a:{k:v for k,v in d.items() if k not in ('samples','controls')} for a,d in arms.items()})
        (self.path/'scene.sdf').write_text(sdf, encoding='utf-8')
        # Archive only specific non-secret runtime inputs when locally available.
        runtime_inputs = {
            'runtime_world.sdf': Path(os.environ.get('LOTUSIM_STATE_HOME',str(Path.home()/'.local/share/lotusim')))/'worlds/lotusim.world',
            'runtime_flake.lock': Path(os.environ.get('XDG_CACHE_HOME',str(Path.home()/'.cache')))/'uuv-lotusim-replay-nix/flake.lock',
        }
        captured={}
        for name,source in runtime_inputs.items():
            if source.is_file():
                (self.path/name).write_bytes(source.read_bytes()); captured[name]=str(source)
        self.write_json('runtime_inputs.json',{'captured':captured,
            'note':'Local file snapshots, not proof of which configuration the running server loaded.'})
        for arm,data in arms.items():
            for name,columns,rows in [('states',payload['sample_columns'],data['samples']),
                                      ('controls',payload['control_columns'],data['controls'])]:
                with (self.path/f'{arm}_{name}.csv').open('w',newline='',encoding='utf-8') as f:
                    writer=csv.writer(f); writer.writerow(columns); writer.writerows(rows)
        self.commands=self.open_csv('commands.csv',['sequence','receipt_wall_s','source_time_s','arm','entity_name','x','y','z','qx','qy','qz','qw'])
        self.observations=self.open_csv('gazebo_poses.csv',['receipt_wall_s','gazebo_time_s','arm','entity_name','x','y','z','qx','qy','qz','qw'])
        self.events=self.open_csv('events.csv',['receipt_wall_s','event','details'])
        self.event('recording_started',{})

    def write_json(self,name,value):
        target=self.path/name
        temporary=target.with_suffix(target.suffix+'.tmp')
        temporary.write_text(json.dumps(value,indent=2,ensure_ascii=False,allow_nan=False),encoding='utf-8')
        temporary.replace(target)

    def open_csv(self,name,columns):
        f=(self.path/name).open('w',newline='',encoding='utf-8',buffering=1)
        self.streams.append(f)
        writer=csv.writer(f); writer.writerow(columns)
        return writer

    def event(self,name,details):
        if not self.closed:
            self.events.writerow([time.monotonic()-self.started,name,json.dumps(details,ensure_ascii=False)])

    def command(self,sequence,arm,name,pose):
        if self.closed: return
        p=asdict(pose)
        self.commands.writerow([sequence,time.monotonic()-self.started,p.pop('source_time'),arm,name,*p.values()])
        self.counts['commands']+=1

    def observe(self,arm,name,stamp,position,quaternion):
        if self.closed: return
        self.observations.writerow([time.monotonic()-self.started,stamp,arm,name,*position,*quaternion])
        self.counts['observations']+=1

    def finish(self,status,error=None):
        if self.closed: return
        self.event('recording_finished',{'status':status,'error':error})
        for f in self.streams: f.close()
        self.closed=True
        self.write_json('summary.json',{'status':status,'error':error,**self.counts,
            'gazebo_observed':self.counts['observations']>0,
            'note':'No Gazebo samples means telemetry is unverified, even if MAS actions succeeded.'})
        self.write_json('manifest.json',{p.name:hashlib.sha256(p.read_bytes()).hexdigest()
            for p in sorted(self.path.iterdir()) if p.is_file() and p.name!='manifest.json'})
