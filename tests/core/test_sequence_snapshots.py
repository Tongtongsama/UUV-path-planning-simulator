"""Caller-owned containers cannot invalidate constructed Core sequences."""
from dataclasses import FrozenInstanceError

import pytest

from core import Path, Pose, Trajectory, VehicleState


@pytest.mark.parametrize("factory, field, values", [
    (Path, "poses", [Pose(), Pose(x=1)]),
    (Trajectory, "states", [VehicleState.zero(), VehicleState.zero(timestamp=1)]),
])
@pytest.mark.parametrize("mutation", ["clear", "append", "replace", "reverse"])
def test_direct_constructor_snapshots_caller_list(factory, field, values, mutation):
    source = list(values)
    instance = factory(source)
    expected = tuple(source)
    original_hash = hash(instance)
    if mutation == "clear":
        source.clear()
    elif mutation == "append":
        source.append(source[0])  # Duplicate timestamp would invalidate Trajectory.
    elif mutation == "replace":
        source[1] = None
    else:
        source.reverse()
    assert getattr(instance, field) == expected
    assert isinstance(getattr(instance, field), tuple)
    assert len(instance) == 2
    assert hash(instance) == original_hash
    with pytest.raises(FrozenInstanceError):
        setattr(instance, field, ())


@pytest.mark.parametrize("factory, values", [
    (Path, [Pose(), Pose(x=1)]),
    (Trajectory, [VehicleState.zero(), VehicleState.zero(timestamp=1)]),
])
def test_constructor_and_factory_agree(factory, values):
    assert factory(values) == factory(tuple(values)) == factory.from_list(values)
    assert factory(iter(values)) == factory(tuple(values))


@pytest.mark.parametrize("factory", [Path, Trajectory])
@pytest.mark.parametrize("source", [[], iter(())])
def test_empty_inputs_still_rejected(factory, source):
    with pytest.raises(ValueError):
        factory(source)


@pytest.mark.parametrize("factory", [Path, Trajectory])
def test_invalid_members_and_noniterables_still_rejected(factory):
    with pytest.raises(TypeError):
        factory([None])
    with pytest.raises(TypeError):
        factory(None)


@pytest.mark.parametrize("times", [(0,0), (1,0), (0,float("nan")), (0,float("inf"))])
def test_snapshot_does_not_bypass_timestamp_validation(times):
    with pytest.raises(ValueError):
        Trajectory([VehicleState.zero(timestamp=t) for t in times])
