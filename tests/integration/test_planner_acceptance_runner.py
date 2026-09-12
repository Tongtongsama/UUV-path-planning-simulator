"""Acceptance expectations must fail visibly; plots cannot mask invalid paths."""
import copy
import json
import pytest
from core import Path, Pose
from planner import PlanningResult, PlanningStatus
from validation.planner_acceptance import DEFAULT_CONFIG, evaluate_case, run_acceptance


@pytest.fixture
def config():
    return json.loads(DEFAULT_CONFIG.read_text(encoding="utf-8"))


@pytest.mark.parametrize("index", [0,1,2])
def test_frozen_scenarios_match_expectations(config,index):
    record,_,_,_=evaluate_case(config,config["cases"][index])
    assert record["accepted"] and record["deterministic"]
    if index == 2:
        assert record["path"] is None and record["validation"] is None
    else:
        assert record["validation"]["valid"]


def test_status_mismatch_is_rejected(config):
    case=copy.deepcopy(config["cases"][0])
    case["expected_status"]="NO_PATH"
    assert not evaluate_case(config,case)[0]["accepted"]


def test_invalid_success_path_is_rejected(config,monkeypatch):
    monkeypatch.setattr("validation.planner_acceptance.AStarPlanner.plan",
        lambda *args:PlanningResult(PlanningStatus.SUCCESS,"fake",Path.from_list([Pose(x=-1)])))
    assert not evaluate_case(config,config["cases"][0])[0]["accepted"]


def test_preview_archives_figures_source_and_no_false_test_claim(config,tmp_path):
    pytest.importorskip("matplotlib")
    output=tmp_path/"run"
    report=run_acceptance(output,run_tests=False)
    assert report["cases_accepted"] and not report["automated_acceptance"]
    assert not report["tests"]["executed"]
    for case in config["cases"]:
        assert (output/(case["name"]+".png")).read_bytes().startswith(b"\x89PNG")
        assert "<svg" in (output/(case["name"]+".svg")).read_text(encoding="utf-8")
    assert (output/"source/planner/algorithms/grid/astar.py").exists()
    assert "results.json" in json.loads((output/"manifest.json").read_text())
    with pytest.raises(FileExistsError):
        run_acceptance(output,run_tests=False)
