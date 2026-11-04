"""Display-only static box environment extracted from the verified recording."""
import copy
import math
from xml.etree import ElementTree as ET
from .core import display_world, fit_camera

def static_environment(record):
    models=[];corners=[]
    root=ET.parse(record['root']/'worlds/minimal.world').getroot()
    for source in root.findall('./world/model'):
        if source.get('name')==record['metadata']['model'] or source.findtext('static','false').strip().lower() not in ('true','1'):continue
        pose=list(map(float,source.findtext('pose','0 0 0 0 0 0').split()))
        if len(pose)!=6 or not all(math.isfinite(v) for v in pose) or any(pose[3:]):
            raise ValueError('Replay static boxes require finite unrotated model poses')
        target=ET.Element('model',name=source.get('name'))
        ET.SubElement(target,'static').text='true'
        ET.SubElement(target,'pose').text=' '.join(map(str,pose))
        count=0
        for link in source.findall('link'):
            lp=list(map(float,link.findtext('pose','0 0 0 0 0 0').split()))
            if len(lp)!=6 or not all(math.isfinite(v) for v in lp) or any(lp[3:]):raise ValueError('Unsupported static link pose')
            out=ET.SubElement(target,'link',name=link.get('name','base_link'))
            ET.SubElement(out,'pose').text=' '.join(map(str,lp))
            for visual in link.findall('visual'):
                size=visual.findtext('geometry/box/size')
                if size is None:raise ValueError('Replay environment currently supports inline boxes only')
                dims=list(map(float,size.split()))
                vp=list(map(float,visual.findtext('pose','0 0 0 0 0 0').split()))
                if len(dims)!=3 or any(not math.isfinite(v) or v<=0 for v in dims) or len(vp)!=6 or not all(math.isfinite(v) for v in vp) or any(vp[3:]):raise ValueError('Invalid static box')
                shown=ET.SubElement(out,'visual',name=visual.get('name','wall'))
                ET.SubElement(shown,'pose').text=' '.join(map(str,vp))
                shown.append(copy.deepcopy(visual.find('geometry')))
                material=ET.SubElement(shown,'material')
                color=visual.findtext('material/diffuse') or visual.findtext('material/ambient') or '0.3 0.65 0.9 1'
                for key in ('ambient','diffuse'):ET.SubElement(material,key).text=color
                ET.SubElement(material,'emissive').text='0.08 0.1 0.12 1'
                center=[pose[i]+lp[i]+vp[i] for i in range(3)]
                for x in (-1,1):
                    for y in (-1,1):
                        for z in (-1,1):corners.append([center[i]+sign*dims[i]/2 for i,sign in enumerate((x,y,z))])
                count+=1
        if count:models.append(target)
    return models,corners

def environment_camera(records):
    framed=[]
    for record in records:
        _,corners=static_environment(record)
        framed.append({'poses':record['poses']+[{'position':p} for p in corners]})
    return fit_camera(framed)

def environment_world(namespace,record,camera,title):
    root=ET.fromstring(display_world(namespace,record['poses'][0]['position'],camera,title))
    models,_=static_environment(record)
    for model in models:root.find('world').append(model)
    return ET.tostring(root,encoding='unicode')
