"""Optional, bounded physical alignment with a paused reference clock."""
from dataclasses import dataclass
from math import hypot, pi
from controller import BodyGuidance3DOF, wrap_to_pi
from trajectory.generator import finite
from .terminal_capture import CaptureDecision


@dataclass(frozen=True)
class StartupConfig:
    budget: float = 20.0
    maximum_pitch: float = 70*pi/180
    maximum_actual_pitch: float = 75*pi/180
    maximum_actual_pitch_rate: float = .35
    maximum_aligned_release_surge: float = 1.0
    reference_rate: float = .15
    pitch_tolerance: float = .03
    speed_tolerance: float = .03
    rate_tolerance: float = .02
    settle_time: float = .5
    maximum_drift: float = .10

    def __post_init__(self):
        for key in self.__dataclass_fields__:
            value = finite(getattr(self, key), key)
            if value <= 0:
                raise ValueError(f'{key} must be positive')
            object.__setattr__(self, key, value)
        if not self.maximum_pitch < self.maximum_actual_pitch < pi/2:
            raise ValueError('require startup reference pitch < actual guard < pi/2')
        if self.reference_rate >= self.maximum_actual_pitch_rate:
            raise ValueError('reference rate must be below actual rate guard')


class ControlledStartupPolicy:
    """Brake, slew actual vehicle to first heading, qualify, then release clock.

    No pose teleportation, controller reset at release, or position outer loop.
    Actual-motion safety is always enforced by Navigator.
    """
    def __init__(self, config=StartupConfig()):
        if not isinstance(config, StartupConfig):
            raise TypeError('invalid startup config')
        self.config = config
        self.reset()

    def reset(self):
        self.origin = None
        self.pitch = None
        self.held = 0.
        self.previous_qualified = False
        self.released = False
        self.braked = False

    def update(self, state, heading, dt):
        c = self.config
        dt = finite(dt, 'dt')
        heading = finite(heading, 'heading')
        if dt <= 0:
            raise ValueError('dt must be positive')
        if self.released:
            return CaptureDecision('RELEASED', None)
        initial_call=self.origin is None
        if initial_call:
            self.origin, self.pitch = state, state.pose.pitch
        elapsed = state.timestamp-self.origin.timestamp
        if abs(heading)>c.maximum_pitch:
            return CaptureDecision('FAILED', None, 'startup heading outside configured envelope')
        if abs(state.pose.pitch)>c.maximum_actual_pitch or abs(state.twist.q)>c.maximum_actual_pitch_rate:
            return CaptureDecision('FAILED', None, 'startup actual attitude/rate guard')
        heading_error=abs(wrap_to_pi(heading-state.pose.pitch))
        aligned_motion=(heading_error<=c.pitch_tolerance and abs(state.twist.q)<=c.rate_tolerance
            and abs(state.twist.w)<=c.speed_tolerance
            and -c.speed_tolerance<=state.twist.u<=c.maximum_aligned_release_surge)
        if initial_call and aligned_motion:
            self.released=True
            return CaptureDecision('RELEASED_ALIGNED',None)
        if hypot(state.pose.x-self.origin.pose.x, state.pose.z-self.origin.pose.z)>c.maximum_drift:
            return CaptureDecision('FAILED', None, 'startup drift budget exhausted')
        quiet = hypot(state.twist.u, state.twist.w)<=c.speed_tolerance and abs(state.twist.q)<=c.rate_tolerance
        self.braked = self.braked or quiet
        qualified = quiet and heading_error<=c.pitch_tolerance
        self.held = self.held+dt if qualified and self.previous_qualified else 0.
        self.previous_qualified = qualified
        if self.held+1e-12>=c.settle_time:
            self.released = True
            return CaptureDecision('RELEASED', None)
        if elapsed+dt>c.budget+1e-12:
            return CaptureDecision('FAILED', None, 'startup time budget exhausted')
        delta = max(-c.reference_rate*dt, min(c.reference_rate*dt, heading-self.pitch)) if self.braked else 0.
        self.pitch += delta
        return CaptureDecision('ALIGN' if self.braked else 'BRAKE',
            BodyGuidance3DOF(-max(-.15, min(.15, state.twist.u)), self.pitch, delta/dt))
