"""Representation-only lower-bound correction; expiry limits remain unchanged."""
import math


def command_age(sim_time, stamp):
    if not math.isfinite(sim_time) or not math.isfinite(stamp):
        raise ValueError('Nonfinite command time')
    age = sim_time - stamp
    # At the 300 s mission scale this is at most ~2.3e-13 seconds.
    # Restrict to the mission clock domain; never normalize sentinel timestamps.
    if 0 <= sim_time <= 301 and 0 <= stamp <= 301:
        if -4 * max(math.ulp(sim_time), math.ulp(stamp)) <= age < 0:
            return 0.0
    return age
