"""M3a/b/c fixed scenarios, structured failures and real closed-loop evidence."""
import argparse
from dataclasses import asdict, replace
import hashlib
import json
from math import isfinite
from pathlib import Path
import subprocess
import sys
import importlib.metadata
from datetime import datetime, timezone
import numpy as np
from core import Pose, Twist, VehicleState
from environment import Boundary, WorldModel, Obstacle, ConstantCurrent
from planner import AStarPlanner, AStarConfig, PlanningConstraints
from controller import CascadedPID3DOFController
from physics import Python3DOFBackend
from trajectory import TrajectoryConfig
from navigation import Navigator, NavigationRequest, NavigationConfig, NavigationStatus
from simulation.controller_config import load_baseline_controller_parameters
from simulation.vehicle_config import load_synthetic_parameters
from validation.planner_acceptance import ROOT, _git


def check_execution(result, request, config) -> dict[str,bool]:
    """Independently check saved time/control/termination evidence."""
    states=result.state_history; refs=result.reference_history; steps=result.control_history
    checks={"history_aligned":len(states)==len(refs)==len(steps)+1,
        "states_finite":all(np.all(np.isfinite(s.to_numpy())) and np.isfinite(s.timestamp) for s in states),
        "fixed_dt":all(abs(b.timestamp-a.timestamp-config.dt)<1e-9 for a,b in zip(states,states[1:])),
        "reference_time_aligned":all(s.timestamp==r.timestamp for s,r in zip(states,refs)),
        "applied_limits":all(abs(s.applied.tau_x)<=config.maximum_tau_x and abs(s.applied.tau_m)<=config.maximum_tau_m for s in steps),
        "inactive_inputs_zero":all(all(getattr(s.applied,k)==0 for k in ("tau_y","tau_z","tau_k","tau_n")) for s in steps)}
    final=states[-1]; world=request.environment; radius=request.planning_constraints.required_clearance
    if result.status is NavigationStatus.SUCCESS:
        checks["success_position_speed_rate"]=(np.hypot(final.pose.x-request.goal.x,final.pose.z-request.goal.z)<=config.goal_position_tolerance
            and np.hypot(final.twist.u,final.twist.w)<=config.goal_speed_tolerance and abs(final.twist.q)<=config.goal_pitch_rate_tolerance)
        checks["success_executed_safety"]=all(world.is_segment_valid(a.pose,b.pose,radius) for a,b in zip(states,states[1:]))
        held=0.
        for a,b in reversed(list(zip(states,states[1:]))):
            if not all(np.hypot(s.pose.x-request.goal.x,s.pose.z-request.goal.z)<=config.goal_position_tolerance and
                np.hypot(s.twist.u,s.twist.w)<=config.goal_speed_tolerance and abs(s.twist.q)<=config.goal_pitch_rate_tolerance for s in (a,b)): break
            held+=b.timestamp-a.timestamp
        checks["success_settle_hold"]=held+1e-9>=config.settle_time
    elif result.status is NavigationStatus.COLLISION:
        checks["collision_confirmed"]=bool(steps) and world.segment_clearance(states[-2].pose,final.pose)<=radius
    elif result.status is NavigationStatus.OUT_OF_BOUNDS:
        checks["boundary_failure_confirmed"]=bool(steps) and not world.boundary.contains_segment(states[-2].pose,final.pose,radius)
    elif result.status is NavigationStatus.TIMEOUT:
        checks["timeout_horizon"]=abs(final.timestamp-states[0].timestamp-config.max_duration)<1e-8
    return {k:bool(v) for k,v in checks.items()}


def clean(value):
    """JSON-safe evidence: infinities (no obstacles) become null, not JSON NaN."""
    if isinstance(value,dict): return {k:clean(v) for k,v in value.items()}
    if isinstance(value,(tuple,list)): return [clean(v) for v in value]
    if isinstance(value,np.generic): return clean(value.item())
    if isinstance(value,float) and not isfinite(value): return None
    if isinstance(value,NavigationStatus): return value.name
    return value


def build_cases(stage: str, settings: dict):
    """Known success baselines plus explicitly exploratory hard-turn cases."""
    v1=json.loads((ROOT/"config/scenarios/planner_acceptance_v1.json").read_text())
    v2=json.loads((ROOT/"config/scenarios/planner_acceptance_v2.json").read_text(encoding="utf-8"))
    cases=[]
    if stage in ("all","minimum"):
        for name,goal,pitch,speed,limits in (
            ("horizontal",(9,-5),0,0,None),("ascending",(9,-3),0,0,None),
            ("descending",(9,-7),0,0,None),("initial_pitch",(9,-5),.2,0,None),
            ("initial_speed",(9,-5),0,.8,None),("saturation",(9,-5),0,0,2.0)):
            cases.append((v1,{"name":name,"obstacles":[],"goal":goal,"pitch":pitch,"speed":speed,"force_limit":limits,
                              "must_succeed":name=="horizontal"},(0,0)))
    if stage in ("all","static"):
        names=("narrow_passage","detour","double_baffle_detour","edge_forced_detour","u_trap_detour","zigzag_maze_detour")
        lookup={c["name"]:(suite,c) for suite in (v1,v2) for c in suite["cases"]}
        for name in names:
            suite,case=lookup[name]
            cases.append((suite,{**case,"must_succeed":name=="narrow_passage"},(0,0)))
    if stage in ("all","current"):
        for velocity in settings["current_comparisons"]:
            case=next(c for c in v1["cases"] if c["name"]=="narrow_passage")
            cases.append((v1,{**case,"name":f"current_{velocity[0]:+.2f}_{velocity[1]:+.2f}",
                             "must_succeed":velocity[1]==0},velocity))
    return cases


def main() -> None:
    """Run selected stages twice, archive full histories, plots and optional GIF."""
    from visualization.navigation import plot_navigation, animate_navigation
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output",type=Path,required=True)
    parser.add_argument("--stage",choices=("minimum","static","current","all"),default="all")
    parser.add_argument("--animate",action="store_true")
    args=parser.parse_args(); args.output.mkdir(parents=True,exist_ok=False)
    settings=json.loads((ROOT/"config/navigation_v08.json").read_text())
    summary=[]
    controller_parameters=load_baseline_controller_parameters()
    physics_parameters=load_synthetic_parameters()
    for suite,case,current in build_cases(args.stage,settings):
        config=NavigationConfig(**settings["navigation"])
        if case.get("force_limit"): config=replace(config,maximum_tau_x=case["force_limit"])
        world=WorldModel(Boundary.rectangle(**suite["boundary"]),[Obstacle.rectangle(**o) for o in case["obstacles"]],ConstantCurrent(current))
        goal=Pose(x=case["goal"][0],z=case["goal"][1]) if "goal" in case else Pose(**suite["goal"])
        initial=VehicleState(Pose(**suite["start"],pitch=case.get("pitch",0)),Twist(u=case.get("speed",0)))
        req=NavigationRequest(initial,goal,world,PlanningConstraints(**suite["constraints"]))
        navigator=Navigator(AStarPlanner(AStarConfig(**suite["astar"])),CascadedPID3DOFController(controller_parameters),
            Python3DOFBackend(physics_parameters),config,TrajectoryConfig(**settings["trajectory"]))
        a=navigator.run(req); b=navigator.run(req)
        deterministic=(a.status==b.status and a.state_history==b.state_history and a.reference_history==b.reference_history
                       and a.control_history==b.control_history and a.metrics==b.metrics)
        accepted=deterministic and (not case["must_succeed"] or a.status is NavigationStatus.SUCCESS)
        # Exploratory collision/timeout is measured, not labelled mission success.
        accepted=accepted and a.status in (NavigationStatus.SUCCESS,NavigationStatus.COLLISION,NavigationStatus.OUT_OF_BOUNDS,NavigationStatus.TIMEOUT)
        checks=check_execution(a,req,config)
        accepted=accepted and all(checks.values())
        evidence={"name":case["name"],"stage":args.stage,"status":a.status.name,"message":a.message,
            "must_succeed":case["must_succeed"],"evidence_accepted":accepted,"deterministic":deterministic,"checks":checks,
            "failed_command":asdict(a.failed_command) if a.failed_command else None,
            "failed_applied":asdict(a.failed_applied) if a.failed_applied else None,"failed_current":a.failed_current,
            "scene":{**case,"boundary":suite["boundary"]},"initial_state":asdict(initial),"goal":asdict(goal),
            "navigation_config":asdict(config),"trajectory_config":settings["trajectory"],"astar_config":suite["astar"],
            "planning_constraints":suite["constraints"],"current":current,"controller_config":asdict(controller_parameters),
            "physics_config":(ROOT/"config/vehicles/uuv_3dof_synthetic_v1.yaml").read_text(),
            "metrics":a.metrics,"state_history":[asdict(s) for s in a.state_history],
            "reference_history":[asdict(s) for s in a.reference_history],"control_history":[asdict(s) for s in a.control_history],
            "trajectory":[asdict(s) for s in a.trajectory] if a.trajectory else None,
            "planner":{"status":a.planning_result.status.name,"time":a.planning_result.planning_time,
                       "expanded_nodes":a.planning_result.expanded_nodes,"path":[asdict(p) for p in a.planning_result.path] if a.planning_result.path else None}}
        (args.output/(case["name"]+".json")).write_text(json.dumps(clean(evidence),indent=2,allow_nan=False),encoding="utf-8")
        plot_navigation(a,req,{**case,"boundary":suite["boundary"]},config,args.output/case["name"])
        if args.animate and case["name"]=="narrow_passage":
            animate_navigation(a,req,{**case,"boundary":suite["boundary"]},config,args.output/"narrow_passage.gif")
        summary.append({"name":case["name"],"status":a.status.name,"accepted":accepted,"deterministic":deterministic,"metrics":a.metrics})
        print(f"{case['name']}: {a.status.name}, t={a.metrics['simulation_duration']:.2f}, error={a.metrics['terminal_position_error']:.3f}, accepted={accepted}",flush=True)
    command=[sys.executable,"-m","pytest","tests","-q","-rs","-p","no:cacheprovider"]
    tests=subprocess.run(command,cwd=ROOT,capture_output=True,text=True,encoding="utf-8",errors="replace")
    (args.output/"pytest.log").write_text(tests.stdout+tests.stderr,encoding="utf-8")
    hashes={}
    for folder in ("navigation","core","planner","trajectory","controller","physics","environment","tests","visualization","validation"):
        for source in (ROOT/folder).rglob("*.py"):
            hashes[source.relative_to(ROOT).as_posix()]=hashlib.sha256(source.read_bytes()).hexdigest()
    report={"date":datetime.now(timezone.utc).isoformat(),"settings":settings,"python":sys.version,"executable":sys.executable,
        "git_head":_git("rev-parse","HEAD"),"git_status":_git("status","--short"),"source_sha256":hashes,
        "packages":{p:importlib.metadata.version(p) for p in ("numpy","shapely","matplotlib","pytest")},
        "tests":{"command":command,"exit_code":tests.returncode},"cases":summary,
        "automated_acceptance":tests.returncode==0 and all(s["accepted"] for s in summary)}
    (args.output/"summary.json").write_text(json.dumps(clean(report),indent=2,allow_nan=False),encoding="utf-8")
    manifest={p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in args.output.iterdir() if p.is_file()}
    (args.output/"manifest.json").write_text(json.dumps(manifest,indent=2),encoding="utf-8")
    raise SystemExit(0 if report["automated_acceptance"] else 1)


if __name__ == "__main__": main()
