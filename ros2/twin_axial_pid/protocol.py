"""Predeclared C1/C2 targets and supplemental allocation recovery experiment."""
import math
from controller.guidance import BodyGuidance3DOF

CASES=('speed_step','pitch_positive','pitch_negative','allocation_recovery')
STEP_TIME=2.0

def duration(case):
    if case not in CASES: raise ValueError('Unknown case')
    return 40.0 if case=='allocation_recovery' else 32.0

def target(case,t):
    if case not in CASES: raise ValueError('Unknown case')
    if t<STEP_TIME: return BodyGuidance3DOF(0,0)
    if case=='allocation_recovery':
        return BodyGuidance3DOF(.6,math.pi/3) if t<10 else BodyGuidance3DOF(.4,0)
    angle=20 if case=='pitch_positive' else -20 if case=='pitch_negative' else 0
    return BodyGuidance3DOF(.4,math.radians(angle))
