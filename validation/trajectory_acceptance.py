"""Independent M2 invariant checks and reproducible pytest evidence bundle."""
import argparse
from dataclasses import asdict
from datetime import datetime, timezone
import hashlib
import importlib.metadata
import json
from math import atan2, hypot, isclose, isfinite, pi
from pathlib import Path as FilePath
import platform
import subprocess
import sys

from core import Path
from planner import AStarPlanner, validate_path
from trajectory import TrajectoryConfig, generate_trajectory
from validation.planner_acceptance import ROOT, build_case, _git


def check_generation(result, space, request, config, repeated) -> dict[str, bool]:
    """Recompute acceptance properties from output, separately from generator."""
    checks={}
    for name in ("raw_path","deduplicated_path","shortcut_path","sampled_path"):
        checks[name+"_valid"]=validate_path(getattr(result,name),space,request.constraints,request=request).valid
    t=result.trajectory
    checks["trajectory_geometry_valid"]=validate_path(Path.from_list([s.pose for s in t]),
        space,request.constraints,request=request).valid
    times=[s.timestamp for s in t]
    lengths=[hypot(b.pose.x-a.pose.x,b.pose.z-a.pose.z) for a,b in zip(t,t.states[1:])]
    checks["timestamps_finite"]=all(isfinite(v) for v in times)
    checks["timestamps_strictly_increasing"]=all(b>a for a,b in zip(times,times[1:]))
    checks["sample_spacing_within_limit"]=all(0<d<=config.sample_spacing+1e-10 for d in lengths)
    checks["duration_consistent"]=isclose(t.duration,result.sampled_path.length()/config.nominal_speed,rel_tol=1e-10,abs_tol=1e-10)
    checks["segment_timing_consistent"]=all(isclose(b.timestamp-a.timestamp,d/config.nominal_speed,
        rel_tol=1e-10,abs_tol=1e-10) for a,b,d in zip(t,t.states[1:],lengths))
    checks["reference_speed_nominal"]=all(isclose(s.twist.u,config.nominal_speed,rel_tol=0,abs_tol=1e-12) for s in t.states[:-1])
    checks["terminal_twist_zero"]=all(getattr(t[-1].twist,k)==0 for k in ("u","v","w","p","q","r"))
    checks["inactive_and_lateral_rates_zero"]=all(
        all(getattr(s.twist,k)==0 for k in ("v","w","p","q","r")) and
        all(getattr(s.pose,k)==0 for k in ("y","roll","yaw")) for s in t)
    checks["pitch_matches_segment"]=all(abs((a.pose.pitch-atan2(b.pose.z-a.pose.z,b.pose.x-a.pose.x)+pi)%(2*pi)-pi)<=1e-10
        for a,b in zip(t,t.states[1:]))
    checks["deterministic"]=result==repeated
    return checks


def run_acceptance(output: FilePath) -> dict:
    """Write a new M2 bundle; run scoped and full tests with this interpreter."""
    output=output.resolve()
    output.mkdir(parents=True,exist_ok=False)
    config=TrajectoryConfig()
    suites={v:json.loads((ROOT/f"config/scenarios/planner_acceptance_{v}.json").read_text(encoding="utf-8")) for v in ("v1","v2")}
    settings={"schema_version":1,"trajectory":asdict(config),"scenario_suites":suites,
              "tolerances":{"spacing_abs_m":1e-10,"duration_rel":1e-10,"duration_abs_s":1e-10,"pitch_abs_rad":1e-10}}
    (output/"config.json").write_text(json.dumps(settings,indent=2),encoding="utf-8")
    cases=[]
    for version,suite in suites.items():
        for case in suite["cases"]:
            space,request,astar=build_case(suite,case)
            plan=AStarPlanner(astar).plan(request,space)
            record={"suite":version,"name":case["name"],"planning_status":plan.status.name,
                    "checks":{"planning_status_expected":plan.status.name==case["expected_status"]},"trajectory":None}
            if plan.success:
                result=generate_trajectory(plan.path,space,request.constraints,config,request=request)
                repeated=generate_trajectory(plan.path,space,request.constraints,config,request=request)
                record["checks"].update(check_generation(result,space,request,config,repeated))
                record["trajectory"]=[asdict(s) for s in result.trajectory]
                record["paths"]={n:[asdict(p) for p in getattr(result,n)] for n in
                    ("raw_path","deduplicated_path","shortcut_path","sampled_path")}
                record["duration_seconds"]=result.trajectory.duration
            else:
                record["checks"]["unreachable_has_no_trajectory"]=plan.path is None and record["trajectory"] is None
            record["automated_acceptance"]=all(record["checks"].values())
            cases.append(record)
    tests={}
    for label,target in (("trajectory","tests/integration/test_trajectory_generation.py"),("full","tests")):
        command=[sys.executable,"-m","pytest",target,"-q","-rs","-p","no:cacheprovider"]
        completed=subprocess.run(command,cwd=ROOT,capture_output=True,text=True,encoding="utf-8",errors="replace")
        (output/f"pytest_{label}.log").write_text(completed.stdout+completed.stderr,encoding="utf-8")
        tests[label]={"command":command,"returncode":completed.returncode,"log":f"pytest_{label}.log"}
    # Hash the actual source state; include the dedicated test source in the bundle.
    source_hashes={}
    for folder in ("core","environment","planner","trajectory","validation","tests"):
        for p in sorted((ROOT/folder).rglob("*.py")):
            source_hashes[p.relative_to(ROOT).as_posix()]=hashlib.sha256(p.read_bytes()).hexdigest()
    test_source=ROOT/"tests/integration/test_trajectory_generation.py"
    (output/"test_trajectory_generation.py").write_bytes(test_source.read_bytes())
    packages={}
    for name in ("numpy","shapely","pytest","scipy"):
        try: packages[name]=importlib.metadata.version(name)
        except importlib.metadata.PackageNotFoundError: packages[name]=None
    report={"schema_version":1,"created_utc":datetime.now(timezone.utc).isoformat(),
        "python":sys.version,"executable":sys.executable,"platform":platform.platform(),
        "packages":packages,"git_head":_git("rev-parse","HEAD"),"git_status":_git("status","--short"),
        "source_sha256":source_hashes,"tests":tests,"cases":cases,
        "automated_acceptance":all(c["automated_acceptance"] for c in cases) and all(t["returncode"]==0 for t in tests.values())}
    (output/"results.json").write_text(json.dumps(report,indent=2,allow_nan=False),encoding="utf-8")
    (output/"summary.txt").write_text("\n".join(
        [f"M2 automated_acceptance={report['automated_acceptance']}"]+
        [f"{c['suite']}/{c['name']}: {c['planning_status']}; accepted={c['automated_acceptance']}" for c in cases]+
        [f"{k} pytest exit={v['returncode']}; log={v['log']}" for k,v in tests.items()]),encoding="utf-8")
    manifest={p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in sorted(output.iterdir()) if p.is_file()}
    (output/"manifest.json").write_text(json.dumps(manifest,indent=2),encoding="utf-8")
    return report


def main() -> None:
    """Exit nonzero if any scenario invariant or either pytest command fails."""
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output",type=FilePath,required=True)
    args=parser.parse_args()
    report=run_acceptance(args.output)
    print(f"M2 automated_acceptance={report['automated_acceptance']}")
    raise SystemExit(0 if report["automated_acceptance"] else 1)


if __name__ == "__main__":
    main()
