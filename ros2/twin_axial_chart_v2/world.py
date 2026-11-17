"""SDF writer using exact obstacle meshes instead of bounding boxes."""
from __future__ import annotations
from xml.etree import ElementTree as ET
from typing import TYPE_CHECKING
from math import isfinite
if TYPE_CHECKING:
    from .adapter import ChartV2

from .mesh import wall_segments


def world_xml(chart: "ChartV2", mesh_names: dict[str, str]) -> str:
    root = ET.Element("sdf", {"version": "1.7"})
    world = ET.SubElement(root, "world", {"name": chart.metadata["name"]})
    ET.SubElement(world, "scene")
    physics = ET.SubElement(world, "physics", {"type": "ode"})
    ET.SubElement(physics, "max_step_size").text = "0.05"
    spherical = ET.SubElement(world, "spherical_coordinates")
    ET.SubElement(spherical, "surface_model").text = "EARTH_WGS84"
    ET.SubElement(spherical, "world_frame_orientation").text = "ENU"
    ET.SubElement(spherical, "latitude_deg").text = f"{chart.origin.latitude:.12f}"
    ET.SubElement(spherical, "longitude_deg").text = f"{chart.origin.longitude:.12f}"
    ET.SubElement(spherical, "elevation").text = f"{chart.origin.altitude_m:.6f}"
    for obstacle in chart.obstacles:
        model = ET.SubElement(world, "model", {"name": f"chart_{obstacle.obstacle_id}"})
        ET.SubElement(model, "static").text = "true"
        ET.SubElement(model, "pose").text = "0 0 0 0 0 0"
        link = ET.SubElement(model, "link", {"name": "link"})
        # The visual keeps the exact polygon extrusion.  Dartsim in this
        # build cannot create mesh/polyline collision, so collision is a set
        # of thin boxes whose centrelines follow every polygon edge exactly.
        for index, (cx, cz, length, pitch) in enumerate(wall_segments(obstacle._geometry)):
            collision = ET.SubElement(link, "collision", {"name": f"shore_{index}"})
            ET.SubElement(collision, "pose").text = f"{cx:.9f} 0 {cz:.9f} 0 {pitch:.12f} 0"
            geometry = ET.SubElement(collision, "geometry")
            box = ET.SubElement(geometry, "box")
            ET.SubElement(box, "size").text = f"{length:.9f} 20 0.2"
        visual = ET.SubElement(link, "visual", {"name": "visual"})
        vgeom = ET.SubElement(visual, "geometry")
        vmesh = ET.SubElement(vgeom, "mesh")
        ET.SubElement(vmesh, "uri").text = mesh_names[obstacle.obstacle_id]
        ET.SubElement(vmesh, "scale").text = "1 1 1"
    seabed = ET.SubElement(world, "model", {"name": "chart_seabed"})
    ET.SubElement(seabed, "static").text = "true"
    ET.SubElement(seabed, "pose").text = "0 0 0 0 0 0"
    link = ET.SubElement(seabed, "link", {"name": "link"})
    # Dartsim cannot create SDF heightmap collision in this build; keep the
    # chart seabed visual only.  The entity smoke uses an explicit test plane.
    visual = ET.SubElement(link, "visual", {"name": "visual"})
    geometry = ET.SubElement(visual, "geometry")
    heightmap = ET.SubElement(geometry, "heightmap")
    ET.SubElement(heightmap, "uri").text = "depth.png"
    ET.SubElement(heightmap, "size").text = f"{chart.depth.x_max-chart.depth.x_min:.9f} {chart.depth.z_max-chart.depth.z_min:.9f} 1"
    return ET.tostring(root, encoding="unicode")
