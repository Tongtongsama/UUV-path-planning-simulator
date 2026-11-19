"""Measured uniform-current adapter for v5 (single CO-OPS sample).

The values are a measured NOAA CO-OPS pm0101 bin-1 vector at 2026-11-17 23:12 UTC.
This is a spatially uniform P1/v5 smoke input, not a full current field.
"""
EAST= 0.05000000000000000
UP= 0.0
SOURCE_ID='NOAA-COOPS-pm0101-bin1-2026-11-17T11:18Z-clamped-0.05'


def current(t):
    return (EAST, UP)


def true_current(t):
    return current(t)


def estimated_current(t):
    return current(t)


def expression_x():
    return repr(EAST)


def expression_z():
    return repr(UP)


def manifest():
    return {'source_type':'measured','source_id':SOURCE_ID,
            'time_utc':'2026-11-17T11:18:00+00:00','east_mps':EAST,'up_mps':UP,
            'spatial_support':'single station, spatially uniform P1 smoke only',
            'true_current':'c_true=measured sample','controller_current':'c_ctrl=measured sample',
            'estimated_current':'c_est=measured sample; no estimator error in this v5 interface smoke'}
