import math
from pathlib import Path
import numpy as np
import pytest
from ros2.twin_axial_feasible_matrix import guidance,scenes

def test_guide_and_configuration_are_original_objects():
    from ros2.twin_axial_feasible_v2.guidance import Guide,CONFIG
    assert guidance.Guide is Guide and guidance.CONFIG is CONFIG

def test_original_s2_path_is_identical():
    from ros2.twin_axial_feasible_v2.guidance import build_path
    for x,z,_ in scenes.INITIALS.values():
        assert np.array_equal(guidance.build_path(scenes.SCENES['S2'],(x,z))[0],build_path(scenes.SCENES['S2'],(x,z))[0])

@pytest.mark.parametrize('scene',scenes.SCENES)
def test_sdf_obstacles_match_documented_scene(scene):
    world=scenes.environment_from_world(scenes.world_xml('P0',scene,True))
    assert scenes.obstacle_extents(world)==scenes.SCENES[scene]

def test_s3_bound_excludes_steady_f0_and_f2_with_current_pitch_limit():
    bound=guidance.bottleneck(scenes.SCENES['S3'])
    assert bound['minimum_descent_course_deg']==pytest.approx(72.412046226)
    # Even at maximum allowed speed, upward current reduces ground descent.
    theta=-math.radians(70)
    max_f2=-math.degrees(theta-math.atan2(-.05*math.cos(theta),.6))
    assert max_f2<70<bound['minimum_descent_course_deg']
    for flow in ('F0','F2'):assert not scenes.preflight('S3','P0',flow)['passed']
    assert scenes.preflight('S3','P0','F1')['passed']

def test_rejected_path_records_no_dynamic_trajectory(tmp_path):
    from ros2.twin_axial_feasible_matrix.python_trial import run
    from ros2.twin_axial_feasible_matrix.verify_python import analyze
    folder=tmp_path/'run';r=run(folder,'P0','F0','S3')
    assert not r['passed'] and not r['dynamics_started']
    assert not (folder/'states.jsonl').exists()
    assert analyze(folder)['checks']['reproduced_rejection']
    (folder/'report.json').write_text('{}')
    assert not analyze(folder)['passed']

def test_real_gate_rejects_missing_matrix(tmp_path):
    from ros2.twin_axial_feasible_matrix.verify_python import gate
    with pytest.raises(ValueError,match='Complete S1/S2/S3'):gate(tmp_path)

def test_numeric_acceptance_unchanged_except_scene_allowlist():
    root=Path(__file__).resolve().parents[1]/'ros2'
    original=(root/'twin_axial_feasible_v2/acceptance.py').read_text()
    adapted=(root/'twin_axial_feasible_matrix/acceptance.py').read_text()
    assert adapted.replace(",'S3/F0','S3/F1','S3/F2'",'')==original
