"""Smooth segmented current schedule for the G12 current-aware revision."""
import math

F2=(0.0,0.05)
T0=60.0; T1=75.0
CENTER=105.0; HALF=30.0
EPS=0.01


def _u(t):
    return (float(t)-CENTER)/HALF


def _gate(t):
    # Smooth algebraic gate, no branch/clamp signals in xdyn expressions.
    u=_u(t)
    return 0.5*(1.0+u/math.sqrt(u*u+EPS))


def current(t):
    return (0.05*_gate(t), F2[1])


def true_current(t):
    return current(t)


def estimated_current(t):
    return current(t)


def expression_x():
    return "0.05*0.5*(1+((t-105.0)/30.0)/sqrt(((t-105.0)/30.0)^2+0.01))"


def expression_z():
    return "0.05"


def manifest():
    return {'schedule':'smooth algebraic gate', 'center_s':CENTER, 'half_width_s':HALF, 'definition':'margin schedule: smooth algebraic ramp 75..135 s',
            'epsilon':EPS, 'start_planning_xz_mps':[0.0,0.05], 'end_planning_xz_mps':[0.05,0.05],
            'true_current':'c_true(t)=current(t)',
            'controller_current':'c_ctrl(t)=current(t)',
            'estimated_current':'c_est(t)=current(t); no estimation error injected'}
