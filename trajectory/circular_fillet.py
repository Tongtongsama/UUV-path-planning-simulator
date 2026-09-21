"""Conservative circular fillets in ENU x-z; no dynamics or controller changes.

Each arc lies inside its sampled chord capsule of radius sagitta. Checking that
entire capsule with footprint+sagitta certifies the arc's circular footprint,
not merely its vertices. Executed LOS motion still needs online safety checks.
"""
from dataclasses import dataclass
from enum import Enum
from math import acos, asin, atan2, ceil, cos, pi, sin, tan

from core import Path, Pose
from planner import PlanningRequest, PlanningSpace, validate_path
from planner._checks import scalar
from planner.candidates import simplify_path


class FilletStatus(Enum):
    ACCEPTED = 'ACCEPTED'
    UNSMOOTHABLE = 'UNSMOOTHABLE'


@dataclass(frozen=True)
class FilletConfig:
    """Fixed geometric radius (m), not a measured dynamic minimum radius.

    lookahead must match the unchanged LOS adapter. A complete sampled arc
    must exceed lookahead / max_lookahead_fraction. No radius shrink fallback.
    """
    radius: float = 1.
    max_sagitta: float = .001
    max_chord: float = .1
    lookahead: float = .5
    max_lookahead_fraction: float = .8
    max_points: int = 10000

    def __post_init__(self) -> None:
        for name in ('radius', 'max_sagitta', 'max_chord', 'lookahead', 'max_lookahead_fraction'):
            value = scalar(getattr(self, name), name)
            if value == 0:
                raise ValueError(name+' must be positive')
            object.__setattr__(self, name, value)
        if self.max_lookahead_fraction >= 1 or self.max_sagitta >= self.radius:
            raise ValueError('lookahead fraction < 1 and sagitta < radius required')
        if isinstance(self.max_points, bool) or not isinstance(self.max_points, int):
            raise TypeError('max_points must be integer')
        if self.max_points < 2:
            raise ValueError('max_points must be >= 2')


@dataclass(frozen=True)
class ArcRecord:
    """Signed ENU arc, original corner identity, and continuous-check evidence."""
    corner_index: int
    center: Pose
    entry: Pose
    exit: Pose
    radius: float
    signed_angle: float
    tangent_distance: float
    chord_count: int
    sagitta: float
    sampled_length: float


@dataclass(frozen=True)
class FilletResult:
    """No partial curved path is returned when any corner cannot be certified."""
    status: FilletStatus
    path: Path | None
    arcs: tuple[ArcRecord, ...]
    reason: str | None = None
    corner_index: int | None = None


def circular_fillet(path: Path, request: PlanningRequest, space: PlanningSpace,
                    config: FilletConfig = FilletConfig()) -> FilletResult:
    """Fillet every noncollinear corner or return an explicit conservative rejection.

    C1 tangent continuous underlying curve; curvature jumps at straight/arc
    joins. The returned Path is an error-bounded polygonal approximation, NOT
    a mathematically curvature-bounded polyline or actuator feasibility proof.
    """
    if not isinstance(config, FilletConfig):
        raise TypeError('config must be FilletConfig')
    path = simplify_path(path)
    validation = validate_path(path, space, request.constraints, request=request)
    def reject(reason, corner=None):
        return FilletResult(FilletStatus.UNSMOOTHABLE, None, (), reason, corner)
    if not validation.valid:
        return reject('INPUT_PATH_INVALID: '+str(validation.message))
    lengths = [a.distance_to(b) for a,b in zip(path,path.poses[1:])]
    trims = [0.]*len(path)
    angles = {}
    for i in range(1,len(path)-1):
        a,b,c = path[i-1:i+2]
        ux,uz = (b.x-a.x)/lengths[i-1],(b.z-a.z)/lengths[i-1]
        vx,vz = (c.x-b.x)/lengths[i],(c.z-b.z)/lengths[i]
        angle = atan2(ux*vz-uz*vx,ux*vx+uz*vz)
        if abs(angle) >= pi-1e-9:
            return reject('REVERSAL_REQUIRES_DIFFERENT_PATH', i)
        trims[i] = config.radius*tan(abs(angle)/2)
        angles[i] = angle
    for i,length in enumerate(lengths):
        if trims[i]+trims[i+1] > length+1e-12:
            return reject('ADJACENT_TANGENT_REGIONS_OVERLAP', i)
    points, records = [path[0]], []
    required = request.constraints.required_clearance
    # Both independent angular bounds ensure max chord and sagitta are respected.
    step_limit = min(2*acos(1-config.max_sagitta/config.radius),
                     2*asin(min(1.,config.max_chord/(2*config.radius))))
    if step_limit <= 0:
        return reject('SAMPLING_PRECISION_UNRESOLVABLE')
    for i,angle in angles.items():
        a,b,c = path[i-1:i+2]
        ux,uz = (b.x-a.x)/lengths[i-1],(b.z-a.z)/lengths[i-1]
        vx,vz = (c.x-b.x)/lengths[i],(c.z-b.z)/lengths[i]
        entry = Pose(x=b.x-trims[i]*ux,z=b.z-trims[i]*uz)
        end = Pose(x=b.x+trims[i]*vx,z=b.z+trims[i]*vz)
        sign = 1 if angle > 0 else -1
        center = Pose(x=entry.x-sign*config.radius*uz,z=entry.z+sign*config.radius*ux)
        n = max(1,ceil(abs(angle)/step_limit))
        if len(points)+n+2 > config.max_points:
            return reject('POINT_BUDGET', i)
        start_angle = atan2(entry.z-center.z,entry.x-center.x)
        arc = [entry]+[Pose(x=center.x+config.radius*cos(start_angle+angle*j/n),
                           z=center.z+config.radius*sin(start_angle+angle*j/n)) for j in range(1,n)]+[end]
        sagitta = config.radius*(1-cos(abs(angle)/(2*n)))
        sampled_length = sum(p.distance_to(q) for p,q in zip(arc,arc[1:]))
        if config.lookahead > config.max_lookahead_fraction*sampled_length:
            return reject('LOOKAHEAD_TOO_LONG_FOR_ARC', i)
        if not space.is_segment_valid(points[-1],entry,required):
            return reject('STRAIGHT_CONNECTOR_UNSAFE', i)
        if not all(space.is_segment_valid(p,q,required+sagitta+1e-10) for p,q in zip(arc,arc[1:])):
            return reject('ARC_SWEPT_CAPSULE_UNSAFE', i)
        if points[-1].distance_to(entry)>1e-12:
            points.append(entry)
        points.extend(arc[1:])
        records.append(ArcRecord(i,center,entry,end,config.radius,angle,trims[i],n,sagitta,sampled_length))
    if points[-1] != path[-1]:
        points.append(path[-1])
    curved = Path(tuple(points))
    checked = validate_path(curved,space,request.constraints,request=request)
    if not checked.valid:
        return reject('OUTPUT_PATH_INVALID: '+str(checked.message))
    return FilletResult(FilletStatus.ACCEPTED,curved,tuple(records))
