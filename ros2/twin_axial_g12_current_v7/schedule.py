"""Measured 3-sample current profile with delayed onset (v7 smoke)."""
import math
TIMES=[0.0,75.0,150.0]
EAST=[0.03249865584940287,-0.06668881840790987,-0.13457663743136464]
NORTH=[0.005730389863008704,-0.029691774944533404,0.03858922981437992]
CENTER=105.0; HALF=30.0; EPS=0.01
SOURCE_ID='NOAA-COOPS-pm0101-bin1-2026-11-17T05:15-05:27Z'

def _gate(t):
    u=(float(t)-CENTER)/HALF
    return 0.5*(1.0+u/math.sqrt(u*u+EPS))

def _linear(t):
    x=float(t)
    if x<=TIMES[0]: return EAST[0]
    if x>=TIMES[-1]: return EAST[-1]
    for i in range(len(TIMES)-1):
        if TIMES[i]<=x<=TIMES[i+1]:
            u=(x-TIMES[i])/(TIMES[i+1]-TIMES[i])
            return EAST[i]+u*(EAST[i+1]-EAST[i])
    return EAST[-1]

def current(t): return (_linear(t)*_gate(t),0.0)
def true_current(t): return current(t)
def estimated_current(t): return current(t)

def _hinge(ti):
    # smooth max(0,t-ti), normalized to zero at t=0
    return f"0.5*((t-{ti})+sqrt((t-{ti})^2+0.01))-0.5*((-{ti})+sqrt(({ti})^2+0.01))"

def expression_x():
    h=[_hinge(t) for t in TIMES]
    terms=[f"({EAST[0]})"]
    for i in range(len(TIMES)-1):
        m=(EAST[i+1]-EAST[i])/(TIMES[i+1]-TIMES[i])
        terms.append(f"+({m})*(({h[i]})-({h[i+1]}))")
    linear=''.join(terms)
    return f"({linear})*0.5*(1+((t-105)/30)/sqrt(((t-105)/30)^2+0.01))"

def expression_z(): return "0.0"
def manifest():
    return {'source_type':'measured','source_id':SOURCE_ID,
            'window_utc':['2026-11-17T05:15:00+00:00','2026-11-17T05:27:00+00:00'],
            'resampled_to_s':TIMES,'east_mps':EAST,'north_mps':NORTH,'up_mps':[0.0,0.0,0.0],
            'ramp':'75..135 s smooth gate','true_current':'c_true=piecewise measured east * gate',
            'controller_current':'c_ctrl=c_true','estimated_current':'c_est=c_true; no estimator error',
            'projection':'north metadata only; 2D x/z uses east and zero up'}
