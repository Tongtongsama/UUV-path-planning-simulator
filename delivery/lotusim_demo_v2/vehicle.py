"""Original schematic UUV visuals, no downloaded mesh or collision claim."""
from xml.etree import ElementTree as ET

def vehicle_sdf(name,color):
    root=ET.Element('sdf',version='1.7'); model=ET.SubElement(root,'model',name=name)
    link=ET.SubElement(model,'link',name='base_link')
    parts=[('hull','0 0 0 0 1.57079632679 0','cylinder',{'radius':'.14','length':'.85'},color),
        ('nose','.425 0 0 0 0 0','sphere',{'radius':'.14'},'1 1 1 1'),
        ('tail','-.425 0 0 0 0 0','sphere',{'radius':'.14'},color),
        ('vertical_fin','-.35 0 .16 0 0 0','box',{'size':'.25 .025 .3'},'0.15 0.2 0.25 1'),
        ('horizontal_fin','-.35 0 0 0 0 0','box',{'size':'.25 .55 .025'},'0.15 0.2 0.25 1'),
        ('propeller','-.62 0 0 0 0 0','box',{'size':'.04 .38 .06'},'0.75 0.75 0.75 1')]
    for name,pose,kind,values,rgba in parts:
        visual=ET.SubElement(link,'visual',name=name); ET.SubElement(visual,'pose').text=pose
        geom=ET.SubElement(ET.SubElement(visual,'geometry'),kind)
        for k,v in values.items(): ET.SubElement(geom,k).text=v
        material=ET.SubElement(visual,'material')
        for k in ('ambient','diffuse'): ET.SubElement(material,k).text=rgba
    return ET.tostring(root,encoding='unicode')
