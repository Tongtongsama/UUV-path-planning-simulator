"""The S3 transient study must stay bounded, uniform and read-only in its checks."""
import pytest

from ros2.twin_axial_s3_transient import study
from ros2.twin_axial_s3_transient.study import (
    GROUPS, FIXED, case_checks, config_for_group, patched_config,
)


def test_screen_is_at_most_twelve_unique_uniform_groups():
    assert len(GROUPS) == 12
    assert len({group["name"] for group in GROUPS}) == 12
    assert len({(group["cruise_speed"], group["lookahead"]) for group in GROUPS}) == 12
    for group in GROUPS:
        config = config_for_group(group)
        for key, value in FIXED.items():
            assert config[key] == value
        assert config["cruise_speed"] == group["cruise_speed"]
        assert config["lookahead"] == group["lookahead"]
        assert config["pitch_limit_deg"] == 70.0
        assert "flow" not in config


def test_config_patch_restores_global_configuration_on_error():
    before = dict(study.base_guidance.CONFIG)
    with pytest.raises(RuntimeError):
        with patched_config({"cruise_speed": 0.99, "lookahead": 9.0}):
            assert study.base_guidance.CONFIG["cruise_speed"] == 0.99
            assert study.base_guidance.CONFIG["lookahead"] == 9.0
            raise RuntimeError("stop")
    assert study.base_guidance.CONFIG == before


def test_scene_thresholds_and_controller_are_frozen():
    assert study.SCENES["S3"] == [[6, 7, -20, -5.5], [13, 14, -14.5, 0]]
    assert study.THRESHOLDS == {
        "safety_clearance_m": 0.9, "rms_m": 0.35, "max_error_m": 0.8,
        "saturation_fraction": 0.2, "timeout_s": 300.0, "pitch_deg": 75.0,
        "q_radps": 0.8, "speed_mps": 1.5,
    }
    assert study.CONTROLLER_SHA256 == (
        "9f3a02e4e68a27b21e2acad4f50fea2d4a221cae7e8696a808265467e189e29f")


def test_case_checks_use_the_unchanged_limits():
    metrics = {"arrival_time": 100.0, "min_clearance": 0.9, "rms_error": 0.35,
               "max_error": 0.8, "saturation_fraction": 0.2}
    assert all(case_checks(metrics, True, None).values())
    assert not case_checks({**metrics, "min_clearance": 0.899}, True, None)["safety"]
    assert not case_checks({**metrics, "rms_error": 0.351}, True, None)["rms"]
    assert not case_checks({**metrics, "max_error": 0.801}, True, None)["max_error"]
    assert not case_checks({**metrics, "saturation_fraction": 0.201}, True, None)["saturation"]
    assert not case_checks({**metrics, "arrival_time": None}, False, "safety")["arrived"]
    assert not case_checks({**metrics, "arrival_time": 301.0}, True, None)["timeout"]
