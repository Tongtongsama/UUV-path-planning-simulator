"""Opt-in, bounded terminal guidance. No planning or inner PID implementation."""
from dataclasses import dataclass
from math import atan2,cos,sin,hypot,pi
from numbers import Integral
from controller import BodyGuidance3DOF,wrap_to_pi
from core import Pose,VehicleState
from planner import PlanningSpace
from trajectory.generator import finite


@dataclass(frozen=True)
class TerminalCaptureConfig:
    """Conservative trial bounds, not vehicle-wide feasibility guarantees."""
    budget: float = 60.0
    max_attempts: int = 6
    maximum_pitch: float = pi/3
    maximum_reference_pitch_rate: float = .15
    maximum_actual_pitch: float = 70*pi/180
    maximum_actual_pitch_rate: float = .35
    correction_speed: float = .08
    brake_speed: float = .03
    brake_pitch_rate: float = .03
    brake_reverse_target: float = .15
    heading_tolerance: float = .08
    move_duration: float = 3.0

    def __post_init__(self):
        for name in self.__dataclass_fields__:
            value=getattr(self,name)
            if name=='max_attempts':
                if isinstance(value,bool) or not isinstance(value,Integral) or value<1:
                    raise ValueError('max_attempts must be positive integer')
            else:
                value=finite(value,name)
                if value<=0:raise ValueError(f'{name} must be positive')
                object.__setattr__(self,name,value)
        if not self.maximum_pitch<self.maximum_actual_pitch<pi/2:
            raise ValueError('require reference pitch < actual guard < pi/2')
        if self.maximum_reference_pitch_rate>=self.maximum_actual_pitch_rate:
            raise ValueError('reference rate must be below actual rate guard')


@dataclass(frozen=True)
class CaptureDecision:
    """Effective target is separate from the original timed reference."""
    phase: str
    target: BodyGuidance3DOF | None
    failure: str | None = None


class TerminalCapturePolicy:
    """Brake first, rate-limited reorientation, committed low-speed correction."""
    def __init__(self,config=TerminalCaptureConfig()):
        if not isinstance(config,TerminalCaptureConfig):raise TypeError('invalid capture config')
        self.config=config
        self.reset()

    def reset(self):
        self.started=None;self.phase='INACTIVE';self.attempts=0
        self.pitch=None;self.heading=None;self.direction=1;self.move_start=None

    def update(self,state: VehicleState,reference: VehicleState,goal: Pose,dt: float,
               reference_end: float,qualified: bool,space: PlanningSpace,clearance: float) -> CaptureDecision:
        """Return no override before reference end or initially qualified arrival."""
        c=self.config
        dt=finite(dt,'dt')
        if dt<=0:raise ValueError('dt must be positive')
        if not isinstance(qualified,bool):raise TypeError('qualified must be bool')
        if state.timestamp<reference_end:return CaptureDecision('TIMED_REFERENCE',None)
        if self.started is None:
            if qualified:return CaptureDecision('ORIGINAL_SETTLE',None)
            self.started=state.timestamp;self.phase='BRAKE'
            self.pitch=reference.pose.pitch;self.heading=self.pitch
        if state.timestamp-self.started+dt>c.budget+1e-12:
            return CaptureDecision('FAILED',None,'capture time budget exhausted')
        if abs(state.pose.pitch)>c.maximum_actual_pitch or abs(state.twist.q)>c.maximum_actual_pitch_rate:
            return CaptureDecision('FAILED',None,'actual attitude/rate exceeds capture guard')
        if abs(self.pitch)>c.maximum_pitch:
            return CaptureDecision('FAILED',None,'entry reference pitch outside capture range')
        speed=hypot(state.twist.u,state.twist.w)
        dx,dz=goal.x-state.pose.x,goal.z-state.pose.z
        if qualified:
            self.phase='SETTLE';self.heading=self.pitch
        elif self.phase=='SETTLE':
            self.phase='BRAKE';self.heading=self.pitch
        if self.phase=='BRAKE' and speed<=c.brake_speed and abs(state.twist.q)<=c.brake_pitch_rate:
            if self.attempts>=c.max_attempts:
                return CaptureDecision('FAILED',None,'capture attempt budget exhausted')
            options=[]
            bearing=atan2(dz,dx)
            for direction in (1,-1):
                heading=max(-c.maximum_pitch,min(c.maximum_pitch,wrap_to_pi(bearing if direction==1 else bearing+pi)))
                vx,vz=direction*cos(heading),direction*sin(heading)
                progress=dx*vx+dz*vz
                endpoint=Pose(x=state.pose.x+vx*min(.3,max(.05,progress)),z=state.pose.z+vz*min(.3,max(.05,progress)))
                if progress>0 and space.is_segment_valid(state.pose,endpoint,clearance):
                    options.append((progress,-abs(heading-state.pose.pitch),heading,direction))
            if not options:return CaptureDecision('FAILED',None,'no admissible collision-free local correction segment')
            _,_,self.heading,self.direction=max(options)
            self.attempts+=1;self.phase='ALIGN'
        if self.phase=='ALIGN' and abs(wrap_to_pi(self.heading-state.pose.pitch))<=c.heading_tolerance and abs(state.twist.q)<=c.brake_pitch_rate:
            self.phase='MOVE';self.move_start=state.timestamp
        if self.phase=='MOVE' and state.timestamp-self.move_start>=c.move_duration:
            self.phase='BRAKE';self.heading=self.pitch
        previous=self.pitch
        delta=max(-c.maximum_reference_pitch_rate*dt,min(c.maximum_reference_pitch_rate*dt,self.heading-previous))
        self.pitch=previous+delta
        u=self.direction*c.correction_speed if self.phase=='MOVE' else -max(-c.brake_reverse_target,min(c.brake_reverse_target,state.twist.u))
        return CaptureDecision(self.phase,BodyGuidance3DOF(u,self.pitch,delta/dt))
