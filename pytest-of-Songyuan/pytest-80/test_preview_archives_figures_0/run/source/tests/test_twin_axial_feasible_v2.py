import math
import numpy as np
import pytest
from ros2.twin_axial_feasible_v2.guidance import Guide, build_path


def test_path_is_unchanged_for_every_initial_condition():
    from ros2.twin_axial_feasible.guidance import build_path as original
    from ros2.twin_axial_navigation.mission import INITIALS
    obstacles=[[6,7,-20,-7],[13,14,-13,0]]
    for x,z,_ in INITIALS.values():
        assert np.array_equal(build_path(obstacles,(x,z))[0],original(obstacles,(x,z))[0])


def test_no_acceleration_until_measured_attitude_is_stable():
    g=Guide([[2,-10],[4.5,-4.5]],[0,0])
    for i in range(1,401):
        target,nav=g.update([2,-10,0,0,0,0],i*.05)
        assert target.u==0 and not nav['launch_ready']
    theta=math.atan2(5.5,2.5)
    for i in range(401,410):
        target,nav=g.update([2,-10,theta,0,0,0],i*.05)
        assert target.u==0
    for i in range(410,430):
        target,nav=g.update([2,-10,theta,0,0,0],i*.05)
    assert target.u>0 and nav['launch_ready']


def test_attitude_rate_interrupts_alignment_dwell():
    g=Guide([[2,-10],[4.5,-4.5]],[0,0]);theta=math.atan2(5.5,2.5)
    for i in range(1,401):
        target,nav=g.update([2,-10,theta,0,0,.1],i*.05)
        assert not nav['launch_ready'] and target.u==0


def test_real_gate_rejects_incomplete_matrix(tmp_path):
    from ros2.twin_axial_feasible_v2.verify_python import gate
    with pytest.raises(ValueError,match='All F0/F1/F2'):gate(tmp_path)


def test_acceptance_logic_is_unchanged():
    from pathlib import Path
    root=Path(__file__).resolve().parents[1]/'ros2'
    assert (root/'twin_axial_feasible/acceptance.py').read_text()==(root/'twin_axial_feasible_v2/acceptance.py').read_text()
