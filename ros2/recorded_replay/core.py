"""Verified pose records, safe visual extraction and a controllable replay clock."""
import bisect
import copy
import hashlib
import json
import math
from pathlib import Path
from xml.etree import ElementTree as ET


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def load_recording(directory):
    root=Path(directory).resolve()
    manifest=json.loads((root/'manifest.json').read_text(encoding='utf-8'))
    required={'gazebo_poses.jsonl','worlds/minimal.world','metadata.json','run_status.json'}
    if not required.issubset(manifest): raise ValueError('Recording manifest lacks required files')
    hashes={}
    for name,expected in manifest.items():
        path=(root/name).resolve()
        if not path.is_relative_to(root) or not path.is_file(): raise ValueError(f'Invalid manifest path: {name}')
        actual=digest(path)
        if actual!=expected: raise ValueError(f'Recording hash mismatch: {name}')
        hashes[name]=actual
    hashes['manifest.json']=digest(root/'manifest.json')
    if (root/'acceptance.json').is_file(): hashes['acceptance.json']=digest(root/'acceptance.json')
    metadata=json.loads((root/'metadata.json').read_text(encoding='utf-8'))
    status=json.loads((root/'run_status.json').read_text(encoding='utf-8'))
    if status.get('status')!='completed': raise ValueError('Only completed recordings can be replayed')
    poses=[]
    for line in (root/'gazebo_poses.jsonl').read_text(encoding='utf-8').splitlines():
        if not line.strip(): continue
        p=json.loads(line); t=p['time']; xyz=p['position']; q=p['quaternion_xyzw']
        if len(xyz)!=3 or len(q)!=4 or not all(isinstance(v,(int,float)) and not isinstance(v,bool) and math.isfinite(v) for v in [t,*xyz,*q]):
            raise ValueError('Invalid recorded pose')
        if poses and t<=poses[-1]['time']: raise ValueError('Recorded times must strictly increase')
        norm=math.sqrt(sum(v*v for v in q))
        if abs(norm-1)>.001: raise ValueError('Recorded quaternion is not unit length')
        poses.append({'time':float(t),'position':xyz,'quaternion_xyzw':[v/norm for v in q]})
    if len(poses)<2: raise ValueError('Need at least two recorded poses')
    return {'root':root,'hashes':hashes,'metadata':metadata,'poses':poses,
            'duration':poses[-1]['time']-poses[0]['time'],
            'times':[p['time'] for p in poses]}


def load_suite(directory):
    """Verify every completed run of a recorded validation suite, in manifest order."""
    root=Path(directory).resolve()
    manifest_path=root/'suite.json'
    if not manifest_path.is_file(): raise ValueError('Suite manifest suite.json is missing')
    try: suite=json.loads(manifest_path.read_text(encoding='utf-8'))
    except (OSError,ValueError) as exc: raise ValueError('Suite manifest is not valid JSON') from exc
    runs=suite.get('runs') if isinstance(suite,dict) else None
    if not isinstance(runs,dict) or not runs: raise ValueError('Suite manifest has no runs')
    records={}
    for name in runs:
        if not isinstance(name,str) or not name or name in ('.','..') or Path(name).name!=name or '/' in name or '\\' in name:
            raise ValueError(f'Invalid suite case name: {name!r}')
        case=root/name
        if not case.is_dir(): raise ValueError(f'Suite case directory is missing: {name}')
        case_resolved=case.resolve()
        if not case_resolved.is_relative_to(root): raise ValueError(f'Suite case escapes suite root: {name}')
        try: records[name]=load_recording(case_resolved)
        except (OSError,ValueError,KeyError) as exc: raise ValueError(f'Invalid suite case {name}: {exc}') from exc
    return {'root':root,'suite':suite,'records':records,'order':list(runs),
            'hashes':{'suite.json':digest(manifest_path)}}


def unchanged(record):
    return all((record['root']/name).is_file() and digest(record['root']/name)==h for name,h in record['hashes'].items())


def suite_unchanged(suite):
    manifest_path=suite['root']/'suite.json'
    return (manifest_path.is_file() and digest(manifest_path)==suite['hashes']['suite.json']
            and all(unchanged(record) for record in suite['records'].values()))


def sample(record, elapsed):
    poses=record['poses']; t=poses[0]['time']+max(0,min(record['duration'],elapsed))
    i=max(0,min(len(poses)-2,bisect.bisect_right(record['times'],t)-1))
    a,b=poses[i:i+2]; f=(t-a['time'])/(b['time']-a['time'])
    qa=a['quaternion_xyzw']; qb=b['quaternion_xyzw']; dot=sum(x*y for x,y in zip(qa,qb))
    if dot<0: qb=[-v for v in qb]; dot=-dot
    dot=min(1,max(-1,dot))
    if dot>.9995: q=[x+f*(y-x) for x,y in zip(qa,qb)]
    else:
        angle=math.acos(dot)
        q=[(math.sin((1-f)*angle)*x+math.sin(f*angle)*y)/math.sin(angle) for x,y in zip(qa,qb)]
    norm=math.sqrt(sum(v*v for v in q))
    return {'time':t,'position':[x+f*(y-x) for x,y in zip(a['position'],b['position'])],
            'quaternion_xyzw':[v/norm for v in q]}


class Playback:
    def __init__(self,duration,rate=.25,paused=True,loop=False):
        if not math.isfinite(duration) or duration<=0: raise ValueError('Invalid duration')
        self.duration=duration; self.elapsed=0.; self.paused=paused; self.loop=loop; self.cycles=0
        self.set_rate(rate)

    def set_rate(self,rate):
        if not math.isfinite(rate) or not .05<=rate<=4: raise ValueError('Rate must be 0.05 through 4')
        self.rate=rate

    def advance(self,dt):
        if dt<0 or not math.isfinite(dt): raise ValueError('Invalid wall-clock delta')
        if self.paused: return
        self.elapsed+=dt*self.rate
        if self.elapsed>=self.duration:
            if self.loop:
                self.cycles+=int(self.elapsed/self.duration); self.elapsed%=self.duration
            else: self.elapsed=self.duration; self.paused=True; self.cycles+=1

    def command(self,line):
        words=line.strip().lower().split()
        if not words: return
        if words==['p']: self.paused=not self.paused
        elif words==['pause']: self.paused=True
        elif words==['play']: self.paused=False
        elif words==['r']: self.elapsed=0.; self.paused=False
        elif words==['l']: self.loop=not self.loop
        elif len(words)==2 and words[0]=='speed': self.set_rate(float(words[1]))
        elif words!=['q']: raise ValueError('Use p, play, pause, r, l, speed 0.25, or q')


def fit_camera(records,margin=1.15,horizontal_fov_deg=60.,aspect=16/9):
    """A static side view that frames every recorded pose of every run."""
    records=list(records)
    if not records: raise ValueError('No recordings to frame')
    points=[p['position'] for record in records for p in record['poses']]
    xs=[p[0] for p in points]; zs=[p[2] for p in points]; ys=[p[1] for p in points]
    center=[(min(xs)+max(xs))/2,(min(ys)+max(ys))/2,(min(zs)+max(zs))/2]
    width=max(max(xs)-min(xs),1.); height=max(max(zs)-min(zs),1.)
    hfov=math.radians(horizontal_fov_deg); vfov=2*math.atan(math.tan(hfov/2)/aspect)
    distance=max(3.,margin/2*max(width/math.tan(hfov/2),height/math.tan(vfov/2)))
    return [center[0],center[1]-distance,center[2],0.,0.,math.pi/2]


def visual_model(record,name):
    source=ET.parse(record['root']/'worlds/minimal.world').getroot()
    models=[m for m in source.findall('./world/model') if m.get('name')==record['metadata']['model']]
    if len(models)!=1: raise ValueError('Recorded vehicle model missing or ambiguous')
    root=ET.Element('sdf',version='1.7'); out=ET.SubElement(root,'model',name=name)
    # Whitelist visuals and link poses only. Never copy plugins, sensors, collision or lotus_param.
    count=0
    for link in models[0].findall('link'):
        target=ET.SubElement(out,'link',name=link.get('name','base_link'))
        if link.find('pose') is not None: target.append(copy.deepcopy(link.find('pose')))
        for visual in link.findall('visual'):
            if visual.findall('.//plugin') or visual.findall('.//uri'):
                raise ValueError('This replay supports self-contained inline geometry only')
            rendered=copy.deepcopy(visual)
            material=rendered.find('material')
            if material is None: material=ET.SubElement(rendered,'material')
            color=material.findtext('ambient') or ('0.9 0.9 0.9 1' if visual.get('name')=='nose' else '0.3 0.4 0.5 1')
            for key in ('ambient','diffuse'):
                if material.find(key) is None: ET.SubElement(material,key).text=color
            # Display-only low emissive fill keeps recorded shapes readable on WSL renderers.
            if material.find('emissive') is None:
                rgba=[float(v) for v in color.split()]
                ET.SubElement(material,'emissive').text=' '.join(str(v*.2) for v in rgba[:3])+' 1'
            target.append(rendered); count+=1
    if not count: raise ValueError('No vehicle visuals')
    return ET.tostring(root,encoding='unicode')


def display_world(world_name,position,camera=None,title=None):
    x,y,z=position
    camera=[x,y-3,z,0,0,math.pi/2] if camera is None else camera
    if len(camera)!=6 or not all(math.isfinite(float(v)) for v in camera): raise ValueError('Invalid display camera pose')
    title='Recorded UUV — visualization only' if title is None else title
    root=ET.Element('sdf',version='1.7'); world=ET.SubElement(root,'world',name=world_name)
    timing=ET.SubElement(world,'physics',name='display_clock',type='ode')
    ET.SubElement(timing,'max_step_size').text='0.05'
    ET.SubElement(timing,'real_time_factor').text='1'
    ET.SubElement(world,'gravity').text='0 0 0'
    environment=ET.SubElement(world,'scene')
    ET.SubElement(environment,'ambient').text='0.7 0.7 0.7 1'
    ET.SubElement(environment,'background').text='0.12 0.16 0.22 1'
    light=ET.SubElement(world,'light',name='display_key_light',type='directional')
    ET.SubElement(light,'direction').text='0.2 0.8 -0.6'
    ET.SubElement(light,'diffuse').text='0.9 0.9 0.9 1'
    ET.SubElement(light,'specular').text='0.2 0.2 0.2 1'
    ET.SubElement(light,'cast_shadows').text='false'
    ET.SubElement(world,'plugin',filename='multi_agent_system_plugin',name='lotusim::gazebo::MultiAgentSystem')
    ET.SubElement(world,'plugin',filename='gz-sim-scene-broadcaster-system',name='gz::sim::systems::SceneBroadcaster')
    spherical=ET.SubElement(world,'spherical_coordinates')
    for key,value in [('surface_model','EARTH_WGS84'),('world_frame_orientation','ENU'),('latitude_deg','0'),('longitude_deg','0'),('elevation','0'),('heading_deg','0')]:
        ET.SubElement(spherical,key).text=value
    gui=ET.SubElement(world,'gui',fullscreen='false')
    scene=ET.SubElement(gui,'plugin',filename='MinimalScene',name='Recorded UUV')
    settings=ET.SubElement(scene,'gz-gui'); ET.SubElement(settings,'title').text=title
    ET.SubElement(settings,'property',key='showTitleBar',type='bool').text='false'
    ET.SubElement(settings,'property',key='state',type='string').text='docked'
    pose=' '.join(f'{float(v):.12g}' for v in camera)
    for key,value in [('engine','ogre2'),('scene','scene'),('ambient_light','1 1 1'),('background_color','0.12 0.16 0.22'),('camera_pose',pose)]:
        ET.SubElement(scene,key).text=value
    for filename in ('GzSceneManager','InteractiveViewControl','CameraTracking','EntityContextMenuPlugin'):
        plugin=ET.SubElement(gui,'plugin',filename=filename,name=filename); settings=ET.SubElement(plugin,'gz-gui')
        for key,kind,value in [('state','string','floating'),('width','double','5'),('height','double','5'),('showTitleBar','bool','false')]:
            ET.SubElement(settings,'property',key=key,type=kind).text=value
    ET.SubElement(gui,'plugin',filename='EntityTree',name='Entity Tree')
    return ET.tostring(root,encoding='unicode')
