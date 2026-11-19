"""Measured time-series current (two NOAA samples) with delayed onset.

Window: 05:15-05:21 UTC on 2026-11-17, resampled to 0-150 s.
The source east component is linearly interpolated and zero current is
applied until a 75-135 s smooth onset. North is retained in metadata but is
not represented in the 2D (x,z) correction.
"""
import math
E0=0.03249865584940287
E1=-0.06668881840790987
DURATION=150.0
CENTER=105.0; HALF=30.0; EPS=0.01
SOURCE_ID='NOAA-COOPS-pm0101-bin1-2026-11-17T05:15-05:21Z'

def _gate(t):
    u=(float(t)-CENTER)/HALF
    return 0.5*(1.0+u/math.sqrt(u*u+EPS))

def _linear(t):
    return E0+(E1-E0)*(float(t)/DURATION)

def current(t):
    return (_linear(t)*_gate(t),0.0)

def true_current(t): return current(t)
def estimated_current(t): return current(t)

def expression_x():
    return f"({E0}+({E1}-{E0})*(t/{DURATION}))*0.5*(1+((t-105)/30)/sqrt(((t-105)/30)^2+0.01))"
def expression_z(): return "0.0"
def manifest():
    return {'source_type':'measured','source_id':SOURCE_ID,
            'window_utc':['2026-11-17T05:15:00+00:00','2026-11-17T05:21:00+00:00'],
            'resampled_to_s':[0.0,DURATION],'extrapolation':'not clamped; mission must end within 0..150 s','east_mps':[E0,E1],'north_mps':[0.005730389863008704,-0.029691774944533404],
            'up_mps':[0.0,0.0],'ramp':'75..135 s smooth gate',
            'true_current':'c_true(t)=linear measured east * gate',
            'controller_current':'c_ctrl(t)=c_true(t)',
            'estimated_current':'c_est(t)=c_true(t); no estimator error',
            'projection':'north is metadata only; 2D x/z correction uses east and zero up'}
