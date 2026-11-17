"""Extrude a Shapely polygon to an exact OBJ mesh for SDF collision.

The mesh uses the project world convention: x east, y up, z north.  The
polygon boundary is kept exactly; triangulation is only used to fill the cap.
"""
from __future__ import annotations
from pathlib import Path
import math
from shapely.geometry import Polygon
from shapely.ops import triangulate

BASE_Y = -2.0
TOP_Y = 2.0


def _safe_name(value: str) -> str:
    cleaned = "".join(c if c.isalnum() or c in "-_." else "_" for c in value)
    return cleaned or "obstacle"


def write_obj(polygon: Polygon, path: str | Path) -> Path:
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    vertices: list[tuple[float, float, float]] = []
    index: dict[tuple[float, float, float], int] = {}
    faces: list[tuple[int, ...]] = []

    def vertex(x: float, y: float, z: float) -> int:
        key = (round(x, 9), round(y, 9), round(z, 9))
        if key not in index:
            index[key] = len(vertices) + 1
            vertices.append(key)
        return index[key]

    caps = [tri for tri in triangulate(polygon) if polygon.covers(tri)]
    if not caps:
        raise ValueError("polygon triangulation produced no interior triangles")
    for tri in caps:
        coords = list(tri.exterior.coords)[:-1]
        if len(coords) != 3:
            raise ValueError("triangulation returned a non-triangle")
        top = [vertex(x, TOP_Y, z) for x, z in coords]
        bottom = [vertex(x, BASE_Y, z) for x, z in coords]
        faces.append(tuple(top)); faces.append(tuple(reversed(bottom)))

    rings = [polygon.exterior, *polygon.interiors]
    for ring in rings:
        coords = list(ring.coords)
        for (x0, z0), (x1, z1) in zip(coords, coords[1:]):
            a = vertex(x0, BASE_Y, z0); b = vertex(x1, BASE_Y, z1)
            c = vertex(x1, TOP_Y, z1); d = vertex(x0, TOP_Y, z0)
            faces.append((a, b, c, d))

    with path.open("w", encoding="ascii", newline="\n") as stream:
        stream.write("# exact chart-v2 obstacle extrusion\n")
        for x, y, z in vertices:
            stream.write(f"v {x:.9f} {y:.9f} {z:.9f}\n")
        for face in faces:
            stream.write("f " + " ".join(str(i) for i in face) + "\n")
    return path


def obj_bounds(path: str | Path) -> tuple[float, float, float, float, float, float]:
    xs=[]; ys=[]; zs=[]
    for line in Path(path).read_text(encoding="ascii").splitlines():
        if line.startswith("v "):
            _, x, y, z = line.split()
            xs.append(float(x)); ys.append(float(y)); zs.append(float(z))
    return min(xs), min(ys), min(zs), max(xs), max(ys), max(zs)


def wall_segments(polygon: Polygon, thickness: float = 0.2) -> list[tuple[float, float, float, float]]:
    """Return (center_x, center_z, length, pitch) for each boundary edge.

    The wall centreline follows the exact polygon ring; a finite thickness is
    used because the Dartsim SDF path does not create mesh/polyline collision.
    """
    segments = []
    for ring in [polygon.exterior, *polygon.interiors]:
        coords = list(ring.coords)
        for (x0, z0), (x1, z1) in zip(coords, coords[1:]):
            dx, dz = x1 - x0, z1 - z0
            length = (dx * dx + dz * dz) ** 0.5
            if length <= 1e-9:
                continue
            # Local box x runs along the edge; pitch about y maps x into x-z.
            segments.append(((x0 + x1) / 2, (z0 + z1) / 2, length,
                             -math.atan2(dz, dx)))
    return segments
