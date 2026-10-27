"""Fixed no-current bench; these are not obstacle-navigation criteria."""
import math
from controller.guidance import BodyGuidance3DOF

MOTION_CASES=('straight','depth_up','depth_down')
FAULT_CASES=('state_loss','stale_state','command_loss')
CASES=MOTION_CASES+FAULT_CASES
FAULT_TIME=8.0
OUTER={'depth_kp_rad_per_m':.35,'vertical_speed_kd_rad_per_mps':.8,
       'maximum_pitch_rad':math.radians(25),'surge_target_mps':.4}
CRITERIA={'depth_settling_s':40.,'depth_band_m':.1,'depth_final_mae_m':.05,
          'depth_overshoot_m':.2,'straight_depth_error_m':.1,
          'speed_band_mps':.04,'speed_final_mae_mps':.02,
          'final_pitch_deg':2.,'fault_max_after_stamp_s':.30}

def duration(case):
    if case not in CASES: raise ValueError('Unknown case')
    return 12. if case in FAULT_CASES else 42. if case=='straight' else 68.

def desired_depth(case,t):
    if case not in CASES: raise ValueError('Unknown case')
    if t<8: return 30.
    return 28. if case=='depth_up' else 32. if case=='depth_down' else 30.

def guidance(case,record):
    t=record['elapsed_end']; s=record['solver_state_ned_frd']
    theta=record['pitch_nose_up_rad']
    depth=desired_depth(case,t)
    vertical_up=s['u']*math.sin(theta)-s['w']*math.cos(theta)
    raw=OUTER['depth_kp_rad_per_m']*(s['z']-depth)-OUTER['vertical_speed_kd_rad_per_mps']*vertical_up
    pitch=max(-OUTER['maximum_pitch_rad'],min(OUTER['maximum_pitch_rad'],raw))
    u=OUTER['surge_target_mps'] if t>=2 else 0.
    return BodyGuidance3DOF(u,pitch),{'desired_depth_m':depth,'actual_depth_m':s['z'],
        'depth_error_up_m':s['z']-depth,'vertical_up_mps':vertical_up,
        'raw_pitch_rad':raw,'limited_pitch_rad':pitch}
