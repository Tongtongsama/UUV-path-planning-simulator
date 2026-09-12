"""Regression tests for public validation input and outcome contracts."""
import pytest
from core import Path, Pose, VehicleState
from planner import (PathValidationResult, PlanningConstraints, PlanningRequest,
                     validate_path, validate_request)


class IndependentSpace:
    """Structural protocol implementation without inheritance or Environment."""
    def contains(self, pose):
        return True

    def clearance(self, pose):
        return float("inf")

    def is_pose_valid(self, pose, required_clearance=0.0):
        return True

    def is_segment_valid(self, start, end, required_clearance=0.0):
        return True


class IncompleteSpace:
    def is_pose_valid(self, pose, required_clearance=0.0):
        raise AssertionError("must reject incomplete protocol before querying")


@pytest.mark.parametrize("space", [None, object(), IncompleteSpace()])
@pytest.mark.parametrize("operation", ["request", "path"])
def test_invalid_space_raises_type_error(space, operation):
    constraints = PlanningConstraints(0)
    with pytest.raises(TypeError, match="space must satisfy PlanningSpace"):
        if operation == "request":
            validate_request(PlanningRequest(VehicleState.zero(), Pose(), constraints), space)
        else:
            validate_path(Path.from_list([Pose()]), space, constraints)


def test_structural_space_is_accepted():
    constraints = PlanningConstraints(0)
    space = IndependentSpace()
    assert validate_request(PlanningRequest(VehicleState.zero(), Pose(), constraints), space) is None
    assert validate_path(Path.from_list([Pose()]), space, constraints).valid


@pytest.mark.parametrize("value", [0, 1, None, "True"])
def test_valid_requires_bool(value):
    with pytest.raises(TypeError, match="valid must be bool"):
        PathValidationResult(value)


@pytest.mark.parametrize("field", ["invalid_waypoint_index", "invalid_segment_index"])
@pytest.mark.parametrize("value", [True, False, 1.5, "0"])
def test_indices_require_nonboolean_integers(field, value):
    with pytest.raises(TypeError):
        PathValidationResult(False, **{field: value})


@pytest.mark.parametrize("field", ["invalid_waypoint_index", "invalid_segment_index"])
def test_index_range_and_success_consistency(field):
    with pytest.raises(ValueError):
        PathValidationResult(False, **{field: -1})
    with pytest.raises(ValueError):
        PathValidationResult(True, **{field: 0})
    assert getattr(PathValidationResult(False, **{field: 0}), field) == 0


def test_mutually_exclusive_indices():
    with pytest.raises(ValueError, match="mutually exclusive"):
        PathValidationResult(False, invalid_waypoint_index=0, invalid_segment_index=0)


@pytest.mark.parametrize("message", [1, False, [], {}])
def test_message_type(message):
    with pytest.raises(TypeError, match="message"):
        PathValidationResult(False, message=message)


def test_results_without_indices_remain_valid():
    assert PathValidationResult(True).valid
    assert PathValidationResult(True, message="checked").valid
    assert not PathValidationResult(False, message="maximum path length exceeded").valid
