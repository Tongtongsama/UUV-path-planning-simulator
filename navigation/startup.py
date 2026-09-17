"""Optional, bounded physical alignment with a paused reference clock."""
from dataclasses import dataclass
from math import hypot, pi
from controller import BodyGuidance3DOF, wrap_to_pi
from trajectory.generator import finite
from .terminal_capture import CaptureDecision


@dataclass(frozen=True)
class StartupConfig:
    budget: float = 20.0
    maximum_pitch: float = pi/3
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
        if self.maximum_pitch >= pi/2:
            raise ValueError('startup pitch must be below pi/2')


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
        if self.origin is None:
            self.origin, self.pitch = state, state.pose.pitch
        elapsed = state.timestamp-self.origin.timestamp
        if abs(heading)>c.maximum_pitch or abs(state.pose.pitch)>70*pi/180 or abs(state.twist.q)>.35:
            return CaptureDecision('FAILED', None, 'startup attitude/rate guard')
        if hypot(state.pose.x-self.origin.pose.x, state.pose.z-self.origin.pose.z)>c.maximum_drift:
            return CaptureDecision('FAILED', None, 'startup drift budget exhausted')
        quiet = hypot(state.twist.u, state.twist.w)<=c.speed_tolerance and abs(state.twist.q)<=c.rate_tolerance
        self.braked = self.braked or quiet
        qualified = quiet and abs(wrap_to_pi(heading-state.pose.pitch))<=c.pitch_tolerance
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
