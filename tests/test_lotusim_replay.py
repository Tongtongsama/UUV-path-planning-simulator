from math import pi, sqrt

import pytest

from ros2.lotusim_replay.lotusim_replay.core import (
    interpolate,
    load_verified_payload,
    lotusim_pose,
    select_arms,
    select_scene,
)


def test_zero_pitch_pose_preserves_enu_position():
    pose = lotusim_pose([2.0, 4.0, -6.0, 0.0, 0.1, 0.2, 0.3])
    assert (pose.source_time, pose.x, pose.y, pose.z) == (2.0, 4.0, 0.0, -6.0)
    assert (pose.qx, pose.qy, pose.qz, pose.qw) == (0.0, 0.0, 0.0, 1.0)


def test_positive_nose_up_pitch_maps_frd_to_flu():
    pose = lotusim_pose([0.0, 0.0, 0.0, pi / 2])
    assert pose.qy == pytest.approx(-sqrt(0.5))
    assert pose.qw == pytest.approx(sqrt(0.5))


def test_unwrapped_pitch_is_not_clamped():
    pose = lotusim_pose([0.0, 0.0, 0.0, 5 * pi])
    assert pose.qy == pytest.approx(-1.0)
    assert pose.qw == pytest.approx(0.0, abs=1e-12)


def test_interpolation_clamps_ends_and_interpolates_pose():
    samples = [[1.0, 0.0, -1.0, 0.0], [3.0, 4.0, -5.0, 2.0]]
    assert interpolate(samples, 0.0) is samples[0]
    assert interpolate(samples, 4.0) is samples[1]
    assert interpolate(samples, 2.0) == [2.0, 2.0, -3.0, 1.0]


def test_scene_and_arm_selection_are_strict():
    payload = {"scenes": [{"name": "case", "arms": {"P1": {}, "APPROACH": {}}}]}
    scene = select_scene(payload, "case")
    assert list(select_arms(scene, ["approach"])) == ["APPROACH"]
    with pytest.raises(ValueError):
        select_scene(payload, "missing")
    with pytest.raises(ValueError):
        select_arms(scene, ["P1", "P1"])


def test_nonfinite_pose_is_rejected():
    with pytest.raises(ValueError):
        lotusim_pose([0.0, float("nan"), 0.0, 0.0])


def test_repository_frozen_payload_passes_existing_evidence_verifier():
    payload = load_verified_payload(__import__("pathlib").Path(__file__).parents[1])
    scene = select_scene(payload, "double_baffle_detour")
    assert len(scene["arms"]["APPROACH"]["samples"]) > 1
    assert payload["schema"] == 1

