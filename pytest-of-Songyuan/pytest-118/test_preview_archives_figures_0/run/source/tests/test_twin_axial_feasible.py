"""Independent revision regression: geometric/physical bounds and fail-closed input."""
import math
import numpy as np
import pytest
from ros2.twin_axial_feasible.guidance import build_path,executable_path,Guide,CONFIG

OBSTACLES=[[6,7,-20,-7],[13,14,-13,0]]

def test_original_steep_shortcut_rejected_in_f2():
    with pytest.raises(ValueError,match='no cruise trim'):
        executable_path([[9,-6],[11.5,-14.5]],[0,.05])

def test_geometry_and_trim_for_all_initials():
    from ros2.twin_axial_navigation.mission import INITIALS
    for x,z,_ in INITIALS.values():
        p,g=build_path(OBSTACLES,(x,z))
        assert g['planning_clearance_m']>=1.5
        assert g['boundary_clearance_m']>=1.5
        f=executable_path(p,[0,.05])
        assert f['cruise_trim_pitch_headroom_deg']>0
        assert f['cruise_trim_max_thrust_N']<15
        assert np.array_equal(p[0],[x,z]) and np.array_equal(p[-1],[18,-10])

def test_current_requires_more_nose_down_trim():
    path=[[0,0],[1,-2]]
    assert executable_path(path,[0,.05])['cruise_trim_pitch_headroom_deg'] < executable_path(path,[0,0])['cruise_trim_pitch_headroom_deg']

def test_flow_drift_compensation_sign_and_rate_bound():
    g=Guide([[0,0],[10,0]],[0,.05])
    for i in range(80):
        target,nav=g.update([0,0,0,.2,-.05,0],2+i*.05)
        assert abs(target.q)<=CONFIG['pitch_rate_limit']+1e-12
        assert abs(target.pitch)<=math.radians(70)
        assert 0<=target.u<=.6
    assert target.pitch<0 # upward drift demands a nose-down correction

def test_invalid_feedback_fails_closed():
    g=Guide([[0,0],[10,0]],[0,.05])
    with pytest.raises(ValueError,match='Invalid feedback'):g.update([0,0,float('nan'),0,0,0],.05)
    g.update([0,0,0,0,0,0],.05)
    with pytest.raises(ValueError,match='Noncontiguous'):g.update([0,0,0,0,0,0],.15)

def test_unsupported_environment_is_not_silently_replaced():
    with pytest.raises(ValueError,match='topology'):build_path([[6,7,-19,-7],[13,14,-13,0]])

def test_existing_output_is_never_overwritten(tmp_path):
    from ros2.twin_axial_feasible.python_trial import run
    marker=tmp_path/'original.txt';marker.write_text('preserve')
    with pytest.raises(FileExistsError):run(tmp_path)
    assert marker.read_text()=='preserve'
