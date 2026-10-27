"""Display-only SDF generated from verified x-z evidence. No physics imports."""
from math import atan2, hypot
from xml.etree import ElementTree as ET


def scene_sdf(scene):
    root = ET.Element('sdf', version='1.7')
    model = ET.SubElement(root, 'model', name='uuv_replay_scene')
    ET.SubElement(model, 'static').text = 'true'
    link = ET.SubElement(model, 'link', name='base_link')

    def box(name, x, z, sx, sy, sz, color, angle=0):
        visual = ET.SubElement(link, 'visual', name=name)
        ET.SubElement(visual, 'pose').text = f'{x} 0 {z} 0 {-angle} 0'
        geom = ET.SubElement(visual, 'geometry')
        ET.SubElement(ET.SubElement(geom, 'box'), 'size').text = f'{sx} {sy} {sz}'
        material = ET.SubElement(visual, 'material')
        ET.SubElement(material, 'ambient').text = color
        ET.SubElement(material, 'diffuse').text = color

    def segment(name, a, b, color, width=0.025):
        dx, dz = b[0]-a[0], b[1]-a[1]
        if hypot(dx, dz) > 1e-12:
            box(name, (a[0]+b[0])/2, (a[1]+b[1])/2,
                hypot(dx, dz), width, width, color, atan2(dz, dx))

    for i, obstacle in enumerate(scene['obstacles']):
        x0, x1, z0, z1 = (obstacle[k] for k in ('x_min', 'x_max', 'z_min', 'z_max'))
        box(f'obstacle_{i}', (x0+x1)/2, (z0+z1)/2, x1-x0, 0.6, z1-z0, '0.6 0.6 0.65 1')
    boundary = scene['boundary']
    x0, x1, z0, z1 = (boundary[k] for k in ('x_min', 'x_max', 'z_min', 'z_max'))
    corners = [(x0,z0),(x1,z0),(x1,z1),(x0,z1),(x0,z0)]
    for i, (a,b) in enumerate(zip(corners,corners[1:])):
        segment(f'boundary_{i}', a, b, '0.7 0.7 0.7 1')
    for i, (a,b) in enumerate(zip(scene['path'],scene['path'][1:])):
        segment(f'path_{i}', a, b, '0.2 0.8 0.2 1')
    for name, color in [('start','0 0.7 1 1'),('goal','1 0.7 0 1')]:
        point = scene[name]
        box(name, point['x'], point['z'], .15, .15, .15, color)
    return ET.tostring(root, encoding='unicode')


def vehicle_sdf(name, color):
    # +X is the nose. Deliberately schematic, not an identified LRAUV model.
    root = ET.Element('sdf', version='1.7')
    model = ET.SubElement(root, 'model', name=name)
    link = ET.SubElement(model, 'link', name='base_link')
    for part, pose, size, rgba in [
        ('body','0 0 0 0 0 0','.4 .12 .12',color),
        ('nose','.23 0 0 0 0 0','.08 .08 .08','1 1 1 1')]:
        visual = ET.SubElement(link, 'visual', name=part)
        ET.SubElement(visual,'pose').text = pose
        geometry = ET.SubElement(visual,'geometry')
        ET.SubElement(ET.SubElement(geometry,'box'),'size').text = size
        material = ET.SubElement(visual,'material')
        ET.SubElement(material,'ambient').text = rgba
        ET.SubElement(material,'diffuse').text = rgba
    return ET.tostring(root, encoding='unicode')
