import csv
from pathlib import Path
import pytest
from ros2.twin_axial_current_data_v1 import (CurrentMetadata, SyntheticScheduleProvider,
    MeasuredCSVProvider, EstimatedProvider, PairedCurrent)


def measured_meta():
    return CurrentMetadata("measured", "adcp-001", "test-license", "field ADCP observation",
                           "MSL", "UTC", "surface")


def test_synthetic_is_explicitly_synthetic():
    p = SyntheticScheduleProvider(CurrentMetadata("synthetic_schedule", "v4-f2",
        "project-internal", "synthetic smooth v4 F2 schedule", "MSL", "UTC"))
    s = p.at(105.0)
    assert s.is_synthetic and not s.is_measured and not s.is_estimated
    assert s.east_mps == pytest.approx(0.025)
    assert s.up_mps == pytest.approx(0.05)


def test_measured_csv_interpolates_and_labels(tmp_path):
    path = tmp_path/'current.csv'
    with path.open('w', newline='') as f:
        w=csv.writer(f); w.writerow(['time_s','east_mps','north_mps','up_mps','uncertainty_mps'])
        w.writerow([0,0.0,0.0,0.05,0.01]); w.writerow([10,0.1,0.0,0.05,0.02])
    p=MeasuredCSVProvider(path, measured_meta())
    s=p.at(5)
    assert s.is_measured and not s.is_synthetic
    assert s.east_mps==pytest.approx(0.05)
    assert s.uncertainty_mps==pytest.approx(0.02)
    with pytest.raises(ValueError):
        p.at(11)


def test_measured_requires_provenance():
    with pytest.raises(ValueError):
        MeasuredCSVProvider('x.csv', CurrentMetadata("measured", "id", "", "field", "MSL"))


def test_estimated_is_not_truth():
    e=EstimatedProvider('est-1', lambda t: (0.01,0.0,0.05), 0.02, 'adcp-001')
    s=e.at(1.0)
    assert s.is_estimated and not s.is_measured and not s.is_synthetic
    assert s.source_id=='est-1'


def test_paired_keeps_true_and_estimate_separate():
    meta=CurrentMetadata("synthetic_schedule", "v4-f2", "project-internal",
                         "synthetic v4 F2 schedule", "MSL", "UTC")
    truth=SyntheticScheduleProvider(meta)
    est=EstimatedProvider('est-1', lambda t: truth.at(t).as_enu(), 0.01, 'v4-f2')
    pair=PairedCurrent(truth,est)
    assert pair.true_at(105).is_synthetic
    assert pair.estimated_at(105).is_estimated
    assert pair.true_at(105).source_id != pair.estimated_at(105).source_id


def test_synthetic_cannot_claim_measured_provenance():
    with pytest.raises(ValueError):
        CurrentMetadata("synthetic_schedule", "v4", "x", "field measured", "MSL").validate()
