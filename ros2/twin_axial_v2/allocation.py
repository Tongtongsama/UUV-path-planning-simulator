"""Two axial thrusters: no direct heave authority. Forces N, moment Nm."""
import math
from .model import KEYS, LEVER

def allocate(request):
    if set(request) != {'X', 'M'}:
        raise ValueError('Exactly X and M are required; direct heave is not supported')
    for value in request.values():
        if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
            raise ValueError('Finite numeric wrench required')
    x, m = float(request['X']), float(request['M'])
    raw = ((x-m/LEVER)/2, (x+m/LEVER)/2)
    if not all(math.isfinite(v) for v in raw): raise ValueError('Allocation overflow')
    peak = max(map(abs, raw))
    alpha = min(1.0, 15.0/peak) if peak else 1.0
    thrust = dict(zip(KEYS, (v*alpha for v in raw)))
    return {'requested_wrench': {'X': x, 'M': m},
            'unlimited_thrust_N': dict(zip(KEYS, raw)), 'alpha': alpha,
            'allocated_commands': thrust, 'allocated_wrench': wrench(thrust)}

def wrench(thrust):
    upper, lower = (thrust[k] for k in KEYS)
    return {'X': upper+lower, 'Z': 0.0, 'M': LEVER*(lower-upper)}
