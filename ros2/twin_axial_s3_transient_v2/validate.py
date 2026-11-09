"""Frozen G12 validation: S3/F0,F1,F2 P1-P4 and P0 repeat (15 Python trials)."""
import json
import math
import platform
import shutil
from datetime import datetime, timezone
from importlib.metadata import version
from pathlib import Path

import numpy as np

from core import Pose
from physics.integrators import RK4Integrator
from physics.uuv_3dof import Python3DOFDynamics
from simulation.vehicle_config import load_synthetic_parameters
from ros2.twin_axial_navigation.current_model import FLOWS as CURRENT_FLOWS
from ros2.twin_axial_navigation.mission import INITIALS, environment_from_world, segment_clearance
from ros2.twin_axial_pid.control import AllocatedPID
from ros2.twin_axial_s3_transient import study
from .cli import original_rates, read, sha, snapshot, verify_run, write

FLOWS = ("F0", "F1", "F2")
INITIAL_CASES = ("P0", "P1", "P2", "P3", "P4")
NEW_CASES = ("P1", "P2", "P3", "P4", "P0_repeat")
G12 = study.GROUPS[11]
assert G12["name"] == "G12"


def g12_config():
    with original_rates():
        return study.config_for_group(G12)


def _run_case(output, flow, case, source_sha256):
    """Same state/PID/allocator loop as the frozen study, with a case variable."""
    output = Path(output)
    output.mkdir(parents=True, exist_ok=False)
    initial_case = "P0" if case == "P0_repeat" else case
    x, z, angle = INITIALS[initial_case]
    state = np.array([x, z, math.radians(angle), 0, 0, 0.0])
    current = list(CURRENT_FLOWS[flow])
    rows = []
    error = None
    with original_rates():
        config = study.config_for_group(G12)
        with study.patched_config(config):
            points, geometry = study.build_path(study.SCENES["S3"], (x, z))
            try:
                steady = study.preflight("S3", initial_case, flow)
            except Exception as exc:
                steady = {"passed": False, "reason": str(exc)}
            guide = study.base_guidance.Guide(points, current)
            pid = AllocatedPID(math.radians(config["pitch_limit_deg"]))
            dynamics = Python3DOFDynamics(load_synthetic_parameters())
            integrator = RK4Integrator()
            world = environment_from_world(study.world_xml(initial_case, "S3"))
            previous = Pose(x=x, z=z)
            force = np.zeros(3)
            for index in range(6000):
                state = integrator.step(
                    lambda value: dynamics.derivatives(value, force, np.array(current)), state, .05)
                x, z, theta, u, w, q = map(float, state)
                t = round((index + 1) * .05, 8)
                point = Pose(x=x, z=z)
                clearance = segment_clearance(world, previous, point)
                previous = point
                target, navigation = guide.update(state, t)
                record = {"valid": True, "sequence": index, "elapsed_end": t,
                          "pitch_nose_up_rad": theta,
                          "solver_state_ned_frd": {"t": t, "y": x, "z": 20 - z,
                                                   "u": u, "w": w, "q": q}}
                _, control = pid.command(record, target, .05)
                rows.append({"t": t, "state": state.tolist(), "clearance": clearance,
                             "nav": navigation, "control": control,
                             "applied_force": force.tolist()})
                allocated = control["allocation"]["allocated_wrench"]
                force = np.array([allocated["X"], 0, allocated["M"]])
                if clearance < study.THRESHOLDS["safety_clearance_m"] - 1e-6:
                    error = "safety"
                    break
                if (abs(theta) > math.radians(study.THRESHOLDS["pitch_deg"])
                        or abs(q) > study.THRESHOLDS["q_radps"]
                        or max(abs(u), abs(w)) > study.THRESHOLDS["speed_mps"]):
                    error = "envelope"
                    break
                if guide.arrived:
                    break
    arrived = bool(guide.arrived)
    metrics = study._metrics(rows, arrived)
    checks = study.case_checks(metrics, arrived, error)
    report = {
        "passed": all(checks.values()),
        "backend": "Python 3-DoF reference; no LOTUSim/xdyn validation",
        "scene": "S3", "case": initial_case, "flow": flow, "group": "G12",
        "run_case": case,
        "parameters": config,
        "checks": checks, "metrics": metrics,
        "steady_preflight": {"passed": steady.get("passed"),
                             "reason": steady.get("reason"),
                             "bottleneck": steady.get("bottleneck"),
                             "steady_trim": steady.get("steady_trim")},
        "error": error,
        "failure": None if error is None else {
            "t": rows[-1]["t"], "position_planning_xz_m": rows[-1]["state"][:2],
            "clearance_m": rows[-1]["clearance"], "step": len(rows) - 1},
        "source_sha256": source_sha256,
        "controller_sha256": study.CONTROLLER_SHA256,
    }
    plan = {"path": points.tolist(), "config": config, "geometry": geometry,
            "case": initial_case, "scene": "S3", "flow": flow, "group": "G12",
            "run_case": case, "steady_preflight": report["steady_preflight"]}
    write(output / "plan.json", plan)
    with (output / "states.jsonl").open("w", encoding="utf-8") as stream:
        for row in rows:
            stream.write(json.dumps(row) + "\n")
    write(output / "report.json", report)
    manifest = {path.relative_to(output).as_posix(): sha(path)
                for path in sorted(output.rglob("*"))
                if path.is_file() and path.name != "manifest.json"}
    write(output / "manifest.json", manifest)
    print(json.dumps({"passed": report["passed"], "flow": flow, "run_case": case,
                      "checks": checks, "error": error}), flush=True)
    return report


def _repeatability(reference, repeat):
    def rows(folder):
        return [json.loads(line) for line in
                (Path(folder) / "states.jsonl").read_text(encoding="utf-8").splitlines() if line]
    a, b = rows(reference), rows(repeat)
    shared = sorted({round(row["t"], 6) for row in a} & {round(row["t"], 6) for row in b})
    by_a = {round(row["t"], 6): row for row in a}
    by_b = {round(row["t"], 6): row for row in b}
    delta = max(math.dist(by_a[t]["state"][:2], by_b[t]["state"][:2]) for t in shared) if shared else None
    return {
        "passed": bool(shared) and len(shared) >= min(len(a), len(b)) - 1
                  and delta is not None and delta <= .02,
        "max_position_difference_m": delta,
        "matched_steps": len(shared),
        "terminal_time_difference_s": abs(a[-1]["t"] - b[-1]["t"]),
    }


def validate(output, screen):
    output = Path(output)
    if output.exists():
        raise FileExistsError(output)
    output.mkdir(parents=True)
    screen = Path(screen).resolve()
    config = g12_config()
    source_sha256 = snapshot(output)
    write(output / "environment.json", {
        "python": platform.python_version(), "platform": platform.platform(),
        "packages": {name: version(name) for name in ("numpy", "shapely", "PyYAML")}})
    reference = {}
    for flow in FLOWS:
        folder = output / "G12" / flow
        folder.mkdir(parents=True)
        source = screen / "G12" / flow
        verified = verify_run(source, replay=True)
        assert verified["passed"], f"G12/{flow}/P0 reference is not passed"
        shutil.copytree(source, folder / "P0")
        reference[flow] = {"source": str(source), "manifest_sha256": sha(source / "manifest.json"),
                           "verified": verified}
    write(output / "preregistration.json", {
        "created_utc": datetime.now(timezone.utc).isoformat(),
        "candidate": "G12", "configuration": config, "flows": list(FLOWS),
        "cases": list(NEW_CASES), "new_dynamic_trials": 15,
        "reference_runs": {flow: reference[flow]["source"] for flow in FLOWS},
        "source_sha256": source_sha256,
        "screen_sha256": sha(screen / "summary.json"),
        "thresholds": study.THRESHOLDS,
        "stop": "15 trials only; no full matrix; no per-flow tuning",
        "s3_remains_nominal": True, "real_started": False,
    })
    runs = {}
    for flow in FLOWS:
        runs[flow] = {"P0": reference[flow]["verified"]}
        for case in NEW_CASES:
            folder = output / "G12" / flow / case
            _run_case(folder, flow, case, source_sha256)
            runs[flow][case] = verify_run(folder, replay=True)
        runs[flow]["repeatability"] = _repeatability(
            output / "G12" / flow / "P0", output / "G12" / flow / "P0_repeat")
    summary = {"candidate": "G12", "configuration": config, "flows": {}}
    all_robust = True
    for flow in FLOWS:
        initial = [runs[flow][case] for case in INITIAL_CASES]
        successful = sum(row["passed"] for row in initial)
        all_safe = all(row["checks"]["safety"] for row in runs[flow].values()
                       if isinstance(row, dict) and "checks" in row)
        repeat = runs[flow]["repeatability"]
        min_clear = min(row["metrics"]["min_clearance"] for row in initial)
        max_rms = max(row["metrics"]["rms_error"] for row in initial)
        max_error = max(row["metrics"]["max_error"] for row in initial)
        robust = successful == 5 and all_safe and repeat["passed"]
        all_robust &= robust
        summary["flows"][flow] = {
            "P0": runs[flow]["P0"], "P1": runs[flow]["P1"], "P2": runs[flow]["P2"],
            "P3": runs[flow]["P3"], "P4": runs[flow]["P4"],
            "P0_repeat": runs[flow]["P0_repeat"],
            "successful_initials": successful, "all_safe": all_safe,
            "repeatability": repeat, "min_clearance": min_clear,
            "max_rms": max_rms, "max_error": max_error, "robust": robust,
        }
    worst_clear = min(row["min_clearance"] for row in summary["flows"].values())
    worst_rms = max(row["max_rms"] for row in summary["flows"].values())
    worst_error = max(row["max_error"] for row in summary["flows"].values())
    # Full 54 also contains S1 and S2.  G12 has only been screened on S3.
    s1_s2_validated = False
    decision = {
        "s3_g12_robust": bool(all_robust),
        "ready_for_full_54": bool(all_robust and s1_s2_validated),
        "s1_s2_g12_validated": s1_s2_validated,
        "full_matrix_started": False,
        "s3_remains_nominal": True,
        "worst_min_clearance_m": worst_clear,
        "worst_rms_m": worst_rms,
        "worst_max_error_m": worst_error,
        "reason": ("S3 的五初态与重复性尚未全部通过，不能进入完整矩阵。"
                   if not all_robust else
                   "S3 的五初态与重复性通过，但完整 54 条还包含 S1/S2；G12 尚未在这两个场景筛选，"
                   "而且 F2 净距裕量需要保留。因此不直接启动完整矩阵，先补 S1/S2 的 G12 检查。"),
        "next_stage": ("先修 S3 失败初态或重新筛选；保持 S3 名义定位。"
                       if not all_robust else
                       "冻结 G12；用同一组参数补 S1/S2 的 G12 筛选（每场景三流五初态+重复），"
                       "再决定是否建立完整 54 条 Python 矩阵。"),
    }
    summary["worst"] = {"min_clearance_m": worst_clear, "rms_m": worst_rms,
                        "max_error_m": worst_error}
    summary["decision"] = decision
    write(output / "summary.json", summary)
    write(output / "decision.json", decision)
    lines = ["# G12 frozen validation", "",
             "S3/P0 reference copied from the frozen screen; 15 new Python trials.",
             "| flow | P0 | P1 | P2 | P3 | P4 | repeat | robust |",
             "| --- | --- | --- | --- | --- | --- | --- | --- |"]
    for flow, row in summary["flows"].items():
        marks = ["PASS" if row[case]["passed"] else "FAIL"
                 for case in (*INITIAL_CASES, "P0_repeat")]
        lines.append(f"| {flow} | " + " | ".join(marks) + f" | {row['robust']} |")
    lines += ["", f"S3 G12 robust: **{decision['s3_g12_robust']}**",
              f"Ready for full 54: **{decision['ready_for_full_54']}**", "",
              decision["reason"], "", decision["next_stage"]]
    (output / "summary.md").write_text("\n".join(lines) + "\n", encoding="utf-8")
    print(json.dumps(decision, indent=2), flush=True)
    return summary
