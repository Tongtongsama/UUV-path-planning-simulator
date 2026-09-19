"""Opt-in spatial guidance adapter; geometry does not own navigation phases."""
from dataclasses import dataclass
from math import atan2, hypot, sqrt

from core import Path, Pose, Twist, VehicleState
from controller import BodyGuidance3DOF
from guidance import GuidanceCommand, LOSConfig, PathGeometry, ProgressState, ProjectionConfig, compute_los


@dataclass(frozen=True)
class LOSNavigationAdapter:
    """Immutable settings; each Navigator.run owns fresh explicit progress state."""

    projection_config: ProjectionConfig
    los_config: LOSConfig

    def __post_init__(self):
        if not isinstance(self.projection_config,ProjectionConfig) or not isinstance(self.los_config,LOSConfig):
            raise TypeError('LOS adapter requires ProjectionConfig and LOSConfig')

    def initialize(self,path: Path,initial: VehicleState) -> ProgressState:
        """Bind progress to processed geometry without relocalizing at launch."""
        return ProgressState(PathGeometry(path),initial.pose)

    def update(self,state: VehicleState,progress: ProgressState,
               previous_pitch: float | None) -> GuidanceCommand:
        """Called exclusively during PATH_FOLLOWING, never startup or capture."""
        return compute_los(state.pose,progress,self.projection_config,self.los_config,previous_pitch)

    @staticmethod
    def to_body(command: GuidanceCommand) -> BodyGuidance3DOF:
        """Direct surge/pitch/q mapping; no extra position/depth correction."""
        if not isinstance(command,GuidanceCommand):raise TypeError('expected GuidanceCommand')
        return BodyGuidance3DOF(command.surge_speed_ref,command.pitch_ref,command.pitch_rate_ref)

    @staticmethod
    def terminal_reference(progress: ProgressState,timestamp: float,actual_pitch: float) -> VehicleState:
        """Finite endpoint, final tangent and zero speed, at the physical time."""
        g=progress.geometry
        pitch=actual_pitch
        if g.lengths:
            a,b=g.points[-2:]
            pitch=atan2(b.z-a.z,b.x-a.x)
        end=g.points[-1]
        return VehicleState(Pose(x=end.x,z=end.z,pitch=pitch),Twist.zero(),timestamp)


def spatial_metrics(steps,states,progress: ProgressState,dt: float) -> dict:
    """Common finite-polyline metrics; global distance here is diagnostic only."""
    g=progress.geometry
    def distance(p):
        values=[]
        for a,b,length in zip(g.points,g.points[1:],g.lengths):
            tx,tz=(b.x-a.x)/length,(b.z-a.z)/length
            arc=max(0.,min(length,(p.x-a.x)*tx+(p.z-a.z)*tz))
            values.append(hypot(p.x-a.x-arc*tx,p.z-a.z-arc*tz))
        return min(values) if values else hypot(p.x-g.points[0].x,p.z-g.points[0].z)
    distances=[distance(s.pose) for s in states]
    following=[i for i,s in enumerate(steps) if s.guidance_phase=='PATH_FOLLOWING']
    ids=sorted({i for k in following for i in (k,k+1)})
    phase_distances=[distances[i] for i in ids]
    commands=[s.guidance_command for s in steps if s.guidance_command is not None]
    pitch_steps=[abs((b.pitch_ref-a.pitch_ref+3.141592653589793)%(2*3.141592653589793)-3.141592653589793)
                 for a,b in zip(commands,commands[1:])]
    arcs=[s.path_progress_s for s in steps]
    capture=[s for s in steps if s.guidance_phase=='TERMINAL_CAPTURE']
    return dict(path_deviation_rmse=sqrt(sum(d*d for d in distances)/len(distances)),
        maximum_path_deviation=max(distances),
        path_following_deviation_rmse=sqrt(sum(d*d for d in phase_distances)/len(phase_distances)) if ids else None,
        path_following_maximum_deviation=max(phase_distances,default=None),
        path_following_duration=len(following)*dt,
        terminal_handoff_time=capture[0].time if capture else None,
        terminal_phase_duration=len(capture)*dt,
        progress_monotonicity_violations=sum(b<a for a,b in zip(arcs,arcs[1:])),
        segment_switch_count=sum(c.switch_count for c in commands),
        progress_clamp_count=sum(c.projection.progress_clamped for c in commands),
        maximum_pitch_reference_step=max(pitch_steps,default=0.),
        maximum_actual_pitch_rate=max(abs(s.twist.q) for s in states),
        final_path_progress=progress.committed_s)
