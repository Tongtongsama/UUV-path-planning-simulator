from pathlib import Path
import json
import pytest
from ros2.twin_axial_current_data_v1 import CurrentMetadata, MeasuredCSVProvider

ROOT = Path(__file__).resolve().parents[1]
P1 = ROOT/'artifacts/current_data_p1_20261118_01'

def test_p1_measured_provider_is_measured():
    raw=json.loads((P1/'metadata.json').read_text(encoding='utf-8'))
    meta=CurrentMetadata('measured',raw['source_id'],raw['license'],raw['provenance'],raw['datum'],raw['time_reference'],raw['vertical_reference'])
    provider=MeasuredCSVProvider(P1/'pm0101_measured_uniform.csv',meta)
    s=provider.at(0.0)
    assert s.is_measured and not s.is_synthetic and not s.is_estimated
    assert s.source_id=='NOAA-COOPS-pm0101-bin1-20261117'
    assert abs(s.east_mps)<2 and abs(s.north_mps)<2
    with pytest.raises(ValueError):
        provider.at(10**7)
