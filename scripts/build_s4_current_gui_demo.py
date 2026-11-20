from pathlib import Path
import json, math
from xml.etree import ElementTree as ET
import sys
ROOT=Path(__file__).resolve().parents[1]; sys.path.insert(0,str(ROOT))
OUT=ROOT/'artifacts/s4_current_gui_demo_20261120_01'
S4_OB=[[8,10,-11,-9],[4,6,-18,-14],[13,15,-6,-2],[13,15,-18,-14],[3,5,-6,-2]]
plan=json.loads((ROOT/'artifacts/twin_axial_s4_terminal_candidate_20261117_01/plan.json').read_text())
PATHS={
 'A':json.loads((ROOT/'artifacts/twin_axial_s4_planning_20261116_04/A_raw_astar/plan.json').read_text())['path'],
 'B':json.loads((ROOT/'artifacts/twin_axial_s4_planning_20261116_04/B_safe_shortcut/plan.json').read_text())['path'],
 'C':json.loads((ROOT/'artifacts/twin_axial_s4_planning_20261116_04/C_safe_smoothing/plan.json').read_text())['path'],
 'D':plan['path']}
from ros2.twin_axial_g12_current_v10.schedule import current

root=ET.Element('sdf',version='1.7'); world=ET.SubElement(root,'world',name='s4_current_gui_demo')
scene=ET.SubElement(world,'scene'); ET.SubElement(scene,'ambient').text='0.55 0.55 0.55 1'; ET.SubElement(scene,'background').text='0.08 0.10 0.14 1'
light=ET.SubElement(world,'light',type='directional',name='sun'); ET.SubElement(light,'cast_shadows').text='false'; ET.SubElement(light,'pose').text='0 0 10 0 0 0'; ET.SubElement(light,'diffuse').text='1 1 1 1'; ET.SubElement(light,'direction').text='0 0 -1'
ET.SubElement(world,'physics',name='demo',type='ode')
# static obstacles
for i,(a,b,c,d) in enumerate(S4_OB):
    m=ET.SubElement(world,'model',name=f'S4_obstacle_{i}'); ET.SubElement(m,'static').text='true'
    ET.SubElement(m,'pose').text=f'{(a+b)/2} 0 {(c+d)/2-20} 0 0 0'
    link=ET.SubElement(m,'link',name='link')
    for kind in ('visual','collision'):
        node=ET.SubElement(link,kind,name=kind); geom=ET.SubElement(node,'geometry'); box=ET.SubElement(geom,'box'); ET.SubElement(box,'size').text=f'{b-a} 4 {d-c}'
        mat=ET.SubElement(node,'material'); ET.SubElement(mat,'ambient').text='.45 .45 .45 1'; ET.SubElement(mat,'diffuse').text='.55 .55 .55 1'
# planned path D as thin boxes
PATH_COLORS={'A':('0.48 0.48 0.48 1','-.18'),'B':('0.12 0.45 0.90 1','-.06'),'C':('0.12 0.70 0.30 1','.06'),'D':('0.85 0.12 0.12 1','.18')}
for key,pts in PATHS.items():
    color,yoff=PATH_COLORS[key]
    for i,(p,q) in enumerate(zip(pts,pts[1:])):
        x1,z1=p; x2,z2=q; dx=x2-x1; dz=z2-z1; length=math.hypot(dx,dz); yaw=math.atan2(dz,dx)
        if length<1e-9: continue
        m=ET.SubElement(world,'model',name=f'path_{key}_{i:03d}'); ET.SubElement(m,'static').text='true'
        ET.SubElement(m,'pose').text=f'{(x1+x2)/2} {yoff} {(z1+z2)/2-20} 0 {-yaw} 0'
        link=ET.SubElement(m,'link',name='link'); vis=ET.SubElement(link,'visual',name='path'); geom=ET.SubElement(vis,'geometry'); box=ET.SubElement(geom,'box'); ET.SubElement(box,'size').text=f'{length} .06 .06'
        mat=ET.SubElement(vis,'material'); ET.SubElement(mat,'ambient').text=color; ET.SubElement(mat,'diffuse').text=color
# UUV visual at start (planning 2,-10 -> ENU z=-30)
m=ET.SubElement(world,'model',name='uuv_start_marker'); ET.SubElement(m,'static').text='true'; ET.SubElement(m,'pose').text='2 0 -30 0 0 0'
link=ET.SubElement(m,'link',name='link')
for name,pose,size,color in [('body','0 0 0 0 0 0','1 .2 .2','1 .5 0 1'),('nose','.55 0 0 0 0 0','.16 .16 .16','1 .8 .2 1')]:
    v=ET.SubElement(link,'visual',name=name); ET.SubElement(v,'pose').text=pose; g=ET.SubElement(v,'geometry'); b=ET.SubElement(g,'box'); ET.SubElement(b,'size').text=size; mat=ET.SubElement(v,'material'); ET.SubElement(mat,'ambient').text=color; ET.SubElement(mat,'diffuse').text=color
# current reference ball trail: integrate v10 measured current from start; balls every 10 s
t=0.0; x=2.0; z=-30.0; dt=.05; idx=0
while t<=150.0:
    if idx % 20 == 0:  # 1 s? 20*0.05=1 s, too many; use 10 s below
        pass
    cx,cz=current(t)
    x += cx*dt; z += cz*dt
    if abs(t-round(t))<1e-9 and int(round(t))%10==0:
        name=f'current_ball_{int(round(t)):03d}'
        m=ET.SubElement(world,'model',name=name); ET.SubElement(m,'static').text='true'; ET.SubElement(m,'pose').text=f'{x:.4f} 0 {z:.4f} 0 0 0'
        l=ET.SubElement(m,'link',name='link'); v=ET.SubElement(l,'visual',name='ball'); g=ET.SubElement(v,'geometry'); s=ET.SubElement(g,'sphere'); ET.SubElement(s,'radius').text='.28'
        mat=ET.SubElement(v,'material'); ET.SubElement(mat,'ambient').text='.1 .35 .95 1'; ET.SubElement(mat,'diffuse').text='.15 .5 1 1'
    t += dt; idx += 1
# start ball bigger yellow
m=ET.SubElement(world,'model',name='current_start_ball'); ET.SubElement(m,'static').text='true'; ET.SubElement(m,'pose').text='2 0 -29 0 0 0'
l=ET.SubElement(m,'link',name='link'); v=ET.SubElement(l,'visual',name='ball'); g=ET.SubElement(v,'geometry'); s=ET.SubElement(g,'sphere'); ET.SubElement(s,'radius').text='.45'; mat=ET.SubElement(v,'material'); ET.SubElement(mat,'ambient').text='1 .8 .1 1'; ET.SubElement(mat,'diffuse').text='1 .8 .1 1'

# rendering/sensors system for camera
sp=ET.SubElement(world,'plugin',filename='gz-sim-sensors-system',name='gz::sim::systems::Sensors'); ET.SubElement(sp,'render_engine').text='ogre'
# overhead camera + camera video recorder system
cam=ET.SubElement(world,'model',name='overview_camera'); ET.SubElement(cam,'static').text='true'
ET.SubElement(cam,'pose').text='0 0 -10 0 0 0'
cl=ET.SubElement(cam,'link',name='camera_link')
cs=ET.SubElement(cl,'sensor',type='camera',name='camera')
ET.SubElement(cs,'pose').text='0 0 0 0 0 0'
ET.SubElement(cs,'always_on').text='1'
ET.SubElement(cs,'visualize').text='false'
cc=ET.SubElement(cs,'camera'); ET.SubElement(cc,'horizontal_fov').text='1.1'
ci=ET.SubElement(cc,'image'); ET.SubElement(ci,'width').text='1280'; ET.SubElement(ci,'height').text='720'; ET.SubElement(ci,'format').text='R8G8B8'
ET.SubElement(cc,'clip').text='0.1 100'

cp=ET.SubElement(cs,'plugin',filename='gz-sim-camera-video-recorder-system',name='gz::sim::systems::CameraVideoRecorder')
ET.SubElement(cp,'service').text='/record_video'; ET.SubElement(cp,'fps').text='25'; ET.SubElement(cp,'use_sim_time').text='false'

ET.indent(root); (OUT/'demo.world').write_text(ET.tostring(root,encoding='unicode'),encoding='utf-8')
(OUT/'scene.json').write_text(json.dumps({'name':'S4_D_path_plus_v10_current_GUI_demo','status':'visualization_only_not_dynamic','world':'demo.world','path_source':'S4 A/B/C/D paths (colored overlays)','current_source':'NOAA pm0101 v10, 60-150 s onset','notes':['A/B/C/D are colored static overlays; UUV response is not recomputed','current balls are a static trail sampled every 10 s; spacing encodes speed/direction','UUV is a start marker, not a simulated vehicle']},indent=2),encoding='utf-8')
print('balls',len([m for m in world.findall('model') if m.get('name','').startswith('current_ball_')]))
