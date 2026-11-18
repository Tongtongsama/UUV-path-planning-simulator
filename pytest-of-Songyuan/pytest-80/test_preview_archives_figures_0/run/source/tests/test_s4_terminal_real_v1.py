import copy
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch
import pytest
from ros2.twin_axial_s4_terminal_real_v1 import mission,gate,trial
from ros2.twin_axial_s4_planning_v1.study import SPEC
from ros2.twin_axial_navigation.mission import environment_from_world,obstacle_extents

def test_full_five_obstacle_world_and_frozen_model():
    for case in ('P0','P2','P4'):
        xml=mission.world_xml(case)
        assert obstacle_extents(environment_from_world(xml))==SPEC['obstacles_xmin_xmax_zmin_zmax']
        assert 'MinimalScene' in xml

def test_mission_rejects_modified_plan():
    p=mission.make_plan('P0','S4','F2');p['path'][-1][0]+=.01
    with pytest.raises(ValueError):mission.Mission(p,mission.world_xml('P0'))

def test_generic_or_fake_proof_rejected():
    with pytest.raises(Exception):gate.guard(object(),runtime=False)

def test_disallowed_case_cannot_prepare_or_start():
    with patch.object(gate,'guard'),patch.object(trial,'prepare') as prepare:
        with pytest.raises(ValueError,match='Unregistered'):
            trial.run_trial(SimpleNamespace(gate_proof=object(),gui=True,scene='S4',flow='F0',case='P1'))
        prepare.assert_not_called()

def test_acceptance_only_changes_mission_import():
    root=Path(__file__).resolve().parents[1]
    expected=(root/'ros2/twin_axial_g12_link_v6/acceptance.py').read_text(encoding='utf-8').replace('from ros2.twin_axial_g12_real_v3.mission import','from .mission import')
    assert (root/'ros2/twin_axial_s4_terminal_real_v1/acceptance.py').read_text(encoding='utf-8')==expected
