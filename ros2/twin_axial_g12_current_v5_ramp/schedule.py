"""Measured weak current with a delayed smooth onset (diagnostic)."""
import math
END=0.03249865584940287
CENTER=105.0; HALF=30.0; EPS=0.01
SOURCE_ID='NOAA-COOPS-pm0101-bin1-2026-11-17T05:15Z'

def _gate(t):
    u=(float(t)-CENTER)/HALF
    return 0.5*(1.0+u/math.sqrt(u*u+EPS))

def current(t):
    return (END*_gate(t), 0.0)

def true_current(t): return current(t)
def estimated_current(t): return current(t)
def expression_x():
    return f"{END}*0.5*(1+((t-105)/30)/sqrt(((t-105)/30)^2+0.01))"
def expression_z(): return "0.0"
def manifest():
    return {'source_type':'measured','source_id':SOURCE_ID,'time_utc':'2026-11-17T05:15:00+00:00',
            'east_end_mps':END,'ramp':'75..135 s','spatial_support':'single station, uniform',
            'true_current':'c_true(t)=ramped measured sample','controller_current':'c_ctrl(t)=c_true(t)',
            'estimated_current':'c_est(t)=c_true(t); no estimator error'}
