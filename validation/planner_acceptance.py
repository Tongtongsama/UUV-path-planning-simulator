"""Run the three M1 planning cases and archive reviewable evidence."""
import argparse
from dataclasses import asdict
from datetime import datetime, timezone
import hashlib
import importlib.metadata
import json
from pathlib import Path as FilePath
import platform
import shutil
import subprocess
import sys

from core import Pose, Twist, VehicleState
from environment import Boundary, Obstacle, WorldModel
from planner import (AStarConfig, AStarPlanner, EnvironmentPlanningSpace,
                     OccupancyGrid, PlanningConstraints, PlanningRequest, validate_path)

ROOT = FilePath(__file__).resolve().parents[1]
DEFAULT_CONFIG = ROOT / "config/scenarios/planner_acceptance_v1.json"


def build_case(config: dict, case: dict):
    """Construct the world and request from the same rectangular plot schema."""
    space = EnvironmentPlanningSpace(WorldModel(Boundary.rectangle(**config["boundary"]),
        [Obstacle.rectangle(**o) for o in case["obstacles"]]))
    request = PlanningRequest(VehicleState(Pose(**config["start"]),Twist.zero()),
        Pose(**config["goal"]),PlanningConstraints(**config["constraints"]))
    return space,request,AStarConfig(**config["astar"])


def evaluate_case(config: dict, case: dict, repeats: int = 2):
    """Compare deterministic fields, independently revalidate every success."""
    if isinstance(repeats,bool) or not isinstance(repeats,int) or repeats < 2:
        raise ValueError("repeats must be an integer >= 2")
    space,request,settings = build_case(config,case)
    results = [AStarPlanner(settings).plan(request,space) for _ in range(repeats)]
    signatures = [(r.status,r.path,r.path_cost,r.expanded_nodes) for r in results]
    checks = [None if r.path is None else validate_path(r.path,space,request.constraints,request=request)
              for r in results]
    deterministic = all(s == signatures[0] for s in signatures)
    accepted = deterministic and all(r.status.name == case["expected_status"] for r in results)
    accepted = accepted and all((r.success and c is not None and c.valid) or
        (not r.success and r.path is None) for r,c in zip(results,checks))
    r = results[0]
    record = {"name":case["name"],"expected_status":case["expected_status"],
        "status":r.status.name,"accepted":accepted,"deterministic":deterministic,
        "planning_times_seconds":[r.planning_time for r in results],
        "expanded_nodes":r.expanded_nodes,"path_length_m":r.path_cost,
        "path":[asdict(p) for p in r.path] if r.path is not None else None,
        "validation":asdict(checks[0]) if checks[0] is not None else None,"message":r.message}
    grid = OccupancyGrid(space,settings.resolution,request.constraints.required_clearance,settings.max_grid_nodes)
    return record,grid,request,r


def _git(*args: str) -> str:
    completed = subprocess.run(["git","-c",f"safe.directory={ROOT.as_posix()}",*args],
        cwd=ROOT,capture_output=True,text=True,encoding="utf-8",errors="replace")
    return completed.stdout.strip() if completed.returncode == 0 else "unavailable: " + completed.stderr.strip()


def run_acceptance(output: FilePath, config_path: FilePath = DEFAULT_CONFIG,
                   repeats: int = 2, run_tests: bool = True) -> dict:
    """Create a new evidence directory; refuse to overwrite an existing run."""
    config = json.loads(config_path.read_text(encoding="utf-8"))
    if config.get("schema_version") != 1:
        raise ValueError("unsupported acceptance schema")
    names = [case["name"] for case in config["cases"]]
    if not names or len(names) != len(set(names)) or any(not n or any(c not in "abcdefghijklmnopqrstuvwxyz_0123456789" for c in n) for n in names):
        raise ValueError("case names must be unique safe identifiers")
    output = output.resolve()
    for folder in ("core","environment","planner","controller","physics","trajectory","navigation","simulation","validation","visualization","tests","config"):
        if output.is_relative_to(ROOT/folder):
            raise ValueError("output must be outside source directories; use artifacts/planner/<run>")
    output.mkdir(parents=True,exist_ok=False)
    from visualization.planner_static import plot_planning_case
    (output/"config.json").write_text(json.dumps(config,indent=2),encoding="utf-8")
    # Preserve actual source, including uncommitted/untracked implementation.
    source_files = []
    for folder in ("core","environment","planner","controller","physics","trajectory","navigation","simulation","validation","visualization","tests"):
        source_files.extend((ROOT/folder).rglob("*.py"))
    source_files.extend(ROOT.glob("requirements*.txt"))
    source_files.extend([ROOT/"pytest.ini"])
    hashes = {}
    for source in sorted(source_files):
        relative = source.relative_to(ROOT)
        target = output/"source"/relative
        target.parent.mkdir(parents=True,exist_ok=True)
        shutil.copy2(source,target)
        hashes[relative.as_posix()] = hashlib.sha256(source.read_bytes()).hexdigest()
    for source in (ROOT/"config").rglob("*"):
        if source.is_file():
            target=output/"source"/source.relative_to(ROOT)
            target.parent.mkdir(parents=True,exist_ok=True)
            shutil.copy2(source,target)
            hashes[source.relative_to(ROOT).as_posix()]=hashlib.sha256(source.read_bytes()).hexdigest()
    records=[]
    for case in config["cases"]:
        record,grid,request,result=evaluate_case(config,case,repeats)
        plot_planning_case(config,case,grid,request,result,output/case["name"])
        records.append(record)
    test = {"executed":False,"returncode":None}
    if run_tests:
        command=[sys.executable,"-m","pytest","tests","-q","-rs","-p","no:cacheprovider"]
        completed=subprocess.run(command,cwd=ROOT,capture_output=True,text=True,encoding="utf-8",errors="replace")
        (output/"pytest.log").write_text(completed.stdout+completed.stderr,encoding="utf-8")
        test={"executed":True,"command":command,"returncode":completed.returncode}
    packages={}
    for name in ("numpy","shapely","PyYAML","matplotlib","pytest","scipy"):
        try: packages[name]=importlib.metadata.version(name)
        except importlib.metadata.PackageNotFoundError: packages[name]=None
    report={"schema_version":1,"created_utc":datetime.now(timezone.utc).isoformat(),
        "python":sys.version,"executable":sys.executable,"platform":platform.platform(),
        "packages":packages,"git_head":_git("rev-parse","HEAD"),
        "git_status":_git("status","--short"),"seed":None,
        "seed_note":"Deterministic A*; no random sampling","source_sha256":hashes,
        "tests":test,"cases":records,
        "cases_accepted":all(r["accepted"] for r in records),
        "automated_acceptance":all(r["accepted"] for r in records) and test["returncode"] == 0,
        "visual_review":"Required separately; automated success does not certify figure layout"}
    (output/"results.json").write_text(json.dumps(report,indent=2,allow_nan=False),encoding="utf-8")
    (output/"summary.txt").write_text("\n".join(
        f"{r['name']}: {r['status']}; accepted={r['accepted']}; length={r['path_length_m']}; expanded={r['expanded_nodes']}"
        for r in records)+f"\nTests: {test}\n",encoding="utf-8")
    artifacts={p.relative_to(output).as_posix():hashlib.sha256(p.read_bytes()).hexdigest()
               for p in sorted(output.rglob("*")) if p.is_file()}
    (output/"manifest.json").write_text(json.dumps(artifacts,indent=2),encoding="utf-8")
    return report


def main() -> None:
    """CLI entry point; nonzero exit on expectation, determinism or test failure."""
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output",type=FilePath,required=True)
    parser.add_argument("--config",type=FilePath,default=DEFAULT_CONFIG)
    parser.add_argument("--repeats",type=int,default=2)
    parser.add_argument("--skip-tests",action="store_true",help="Preview only; not full automated acceptance")
    args=parser.parse_args()
    report=run_acceptance(args.output,args.config,args.repeats,not args.skip_tests)
    for r in report["cases"]:
        print(f"{r['name']}: {r['status']}; accepted={r['accepted']}; length={r['path_length_m']}")
    ok=report["cases_accepted"] if args.skip_tests else report["automated_acceptance"]
    raise SystemExit(0 if ok else 1)


if __name__ == "__main__":
    main()
