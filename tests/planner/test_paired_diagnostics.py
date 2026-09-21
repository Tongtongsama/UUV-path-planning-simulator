"""Pre-turn logging must precede segment switching, not substitute for entry matching."""
from types import SimpleNamespace as NS
from core import Path, Pose, Twist, VehicleState
from validation.planner_paired import turn_events, stable_audit


def test_lookahead_event_precedes_switch(monkeypatch):
    monkeypatch.setattr('validation.planner_paired.compact_control', lambda step: {'progress':step.path_progress_s})
    path = Path((Pose(x=0),Pose(x=1),Pose(x=1,z=1)))
    states = tuple(VehicleState(Pose(x=x),Twist(),timestamp=i*.05) for i,x in enumerate((.3,.5,.8,1.)))
    controls = [NS(guidance_phase='PATH_FOLLOWING',path_progress_s=s,
                   guidance_command=NS(segment_index=k,pitch_ref=angle),controller_diagnostics=None)
                for s,k,angle in ((.3,0,0),(.5,0,.2),(1.,1,.5))]
    result = NS(control_history=controls,state_history=states)
    event = turn_events(result,path,.5,.001)[0]
    assert event['preturn']['step'] == 1
    assert event['preturn']['state_before']['pose']['x'] == .5
    assert event['switch']['step'] == 2
    assert event['first_departing_command']['step'] == 1
    assert event['evidence_status'] == 'UNKNOWN'


def test_unreached_corner_keeps_null_events():
    path = Path((Pose(x=0),Pose(x=1),Pose(x=1,z=1)))
    event = turn_events(NS(control_history=[],state_history=[]),path,.5,.001)[0]
    assert event['preturn'] is event['switch'] is None
    assert event['evidence_status'] == 'UNKNOWN'


def test_geometry_digest_ignores_only_wall_clock():
    a = [{'planning_time':1.,'geometry':{'length':2.,'total_cost':3.}}]
    b = [{'planning_time':9.,'geometry':{'length':2.,'total_cost':3.}}]
    assert stable_audit(a) == stable_audit(b)
    b[0]['geometry']['total_cost'] = 4.
    assert stable_audit(a) != stable_audit(b)
