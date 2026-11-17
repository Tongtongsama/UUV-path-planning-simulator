from pathlib import Path
from xml.etree import ElementTree as ET
from ros2.recorded_replay.environment import static_environment,environment_camera,environment_world

def test_static_walls_survive_without_dynamics(tmp_path):
    (tmp_path/'worlds').mkdir()
    (tmp_path/'worlds/minimal.world').write_text('''<sdf><world name="source">
    <model name="uuv"><link name="body"/></model>
    <model name="wall"><static>true</static><pose>6.5 0 -33.5 0 0 0</pose>
    <plugin filename="must_not_run"/><link name="base_link">
    <collision name="solid"/><visual name="wall"><geometry><box><size>1 4 13</size></box></geometry></visual>
    </link></model></world></sdf>''')
    record={'root':tmp_path,'metadata':{'model':'uuv'},'poses':[{'position':[2,0,-30]},{'position':[18,0,-30]}]}
    models,corners=static_environment(record)
    assert len(models)==1 and min(p[2] for p in corners)==-40
    root=ET.fromstring(environment_world('replay',record,environment_camera([record]),None))
    wall=root.find("./world/model[@name='wall']")
    assert wall.findtext('pose')=='6.5 0.0 -33.5 0.0 0.0 0.0'
    assert wall.findtext('link/visual/geometry/box/size')=='1 4 13'
    assert not wall.findall('.//plugin') and not wall.findall('.//collision')
    assert len(root.findall('./world/model'))==1
