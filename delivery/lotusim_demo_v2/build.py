"""Create v2 from verified v1 extraction; leave original replay and dynamics untouched."""
import hashlib
import json
from pathlib import Path
import shutil
import subprocess
import sys
ROOT=Path(__file__).resolve().parents[2]
HERE=Path(__file__).parent
dest=Path(sys.argv[1]).resolve()
subprocess.run([sys.executable,str(ROOT/'delivery/lotusim_demo/build.py'),str(dest)],check=True)
for name in ('launch.py','vehicle.py'):
    shutil.copy2(HERE/name,dest/'demo'/name)
for name in ('README.md','run-local.sh','run.sh'):
    shutil.copy2(HERE/name,dest/name)
script=(dest/'demo/gazebo_player.py').read_text(encoding='utf-8')
script=script.replace('from .gazebo_scene import scene_sdf, vehicle_sdf','from .gazebo_scene import scene_sdf\nfrom .vehicle import vehicle_sdf')
script=script.replace('names={}', 'names={}\n    last_feedback=[0.0]')
script=script.replace("parser.add_argument('--auto-close', action='store_true')", "parser.add_argument('--auto-close', action='store_true')\n    parser.add_argument('--scene-ready', type=Path)\n    parser.add_argument('--playback-gate', type=Path)")
script=script.replace('last_feedback=[0.0]', 'last_feedback=[0.0]\n    observed_names=set()')
script=script.replace('if arm is None: continue', 'if arm is None: continue\n            observed_names.add(vessel.vessel_name)')
script=script.replace("        print('Focus Entity Tree on '", """        if args.scene_ready:
            deadline=time.monotonic()+30
            while not set(names.values()).issubset(observed_names):
                if time.monotonic()>deadline:
                    raise RuntimeError('Initial UUV state feedback missing; GUI startup cancelled')
                rclpy.spin_once(node,timeout_sec=.1)
            args.scene_ready.write_text(json.dumps({'scene':prefix+'_scene','vehicles':names}),encoding='utf-8')
        if args.playback_gate:
            deadline=time.monotonic()+120
            while not args.playback_gate.exists():
                if time.monotonic()>deadline:
                    raise RuntimeError('GUI startup gate timed out')
                rclpy.spin_once(node,timeout_sec=.1)
        print('Focus Entity Tree on '""")
script=script.replace('stamp=message.header.stamp', '''stamp=message.header.stamp
            if time.monotonic()-last_feedback[0]>=2:
                print(f'LOTUSim feedback: {arm} x={p.x:.3f} z={p.z:.3f} sim_t={stamp.sec+stamp.nanosec*1e-9:.2f}',flush=True)
                last_feedback[0]=time.monotonic()''')
(dest/'demo/gazebo_player.py').write_text(script,encoding='utf-8')
(dest/'ros2/minimal_xdyn').mkdir(parents=True)
(dest/'ros2/__init__.py').write_text('')
for source in (ROOT/'ros2/minimal_xdyn').glob('*.py'): shutil.copy2(source,dest/'ros2/minimal_xdyn'/source.name)
# Restore the fixed environment containing actual xdyn for the optional dynamics demonstration.
shutil.copy2(ROOT/'ros2/minimal_xdyn/nix/flake.nix',dest/'flake.nix')
# A front-on view of the vertical x-z obstacle plane, with +x to the right.
gui='''<gui fullscreen="false">
<plugin filename="MinimalScene" name="3D View"><gz-gui><title>UUV - LOTUSim interaction</title><property type="bool" key="showTitleBar">false</property><property type="string" key="state">docked</property></gz-gui><engine>ogre2</engine><scene>scene</scene><ambient_light>1 1 1</ambient_light><background_color>0.15 0.2 0.3</background_color><camera_pose>5 -16 -5 0 0 1.57079632679</camera_pose></plugin>
<plugin filename="GzSceneManager" name="Scene Manager"/>
<plugin filename="InteractiveViewControl" name="Interactive view control"/>
<plugin filename="CameraTracking" name="Camera Tracking"/>
<plugin filename="EntityContextMenuPlugin" name="Entity context menu"/>
<plugin filename="EntityTree" name="Entity tree"/>
</gui>'''
world=(dest/'lotusim.world').read_text().replace('</world>',gui+'</world>')
# Match Gazebo's standard layout: helper plugins do not need visible panels.
from xml.etree import ElementTree as ET
tree=ET.fromstring(world)
for plugin in tree.findall('./world/gui/plugin'):
    if plugin.attrib['filename'] in ('GzSceneManager','InteractiveViewControl','CameraTracking','EntityContextMenuPlugin'):
        settings=ET.SubElement(plugin,'gz-gui')
        for key,kind,value in [('state','string','floating'),('width','double','5'),('height','double','5'),('showTitleBar','bool','false'),('resizable','bool','false')]:
            ET.SubElement(settings,'property',key=key,type=kind).text=value
world=ET.tostring(tree,encoding='unicode')
(dest/'lotusim.world').write_text(world,encoding='utf-8')
provenance=json.loads((dest/'PROVENANCE.json').read_text(encoding='utf-8'))
provenance['v2']='Schematic UUV hull/fins/nose, camera, existing-server connection, live feedback; separate unchanged minimal_xdyn module included.'
provenance['v2.1']='Stage standalone GUI startup after all MAS CREATE acknowledgements and initial UUV state feedback; hide helper panels. Algorithms and frozen payload unchanged.'
(dest/'PROVENANCE.json').write_text(json.dumps(provenance,ensure_ascii=False,indent=2),encoding='utf-8')
manifest={p.relative_to(dest).as_posix():hashlib.sha256(p.read_bytes()).hexdigest() for p in sorted(dest.rglob('*')) if p.is_file() and p.name!='manifest.json'}
(dest/'manifest.json').write_text(json.dumps(manifest,indent=2),encoding='utf-8')
