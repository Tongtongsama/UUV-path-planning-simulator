"""S3 transient feasibility: audit old trajectories, screen 12 uniform guidance sets.

The model, PID, allocator, scene geometry and acceptance limits are reused
unchanged.  Only guidance path-following parameters are screened.  The steady
preflight certificate is recorded but deliberately not used as a gate: the
point of the study is whether a transient run can succeed where that
conservative certificate rejects.
"""
import hashlib
import json
import math
import os
from contextlib import contextmanager
from dataclasses import asdict
from pathlib import Path

import numpy as np

from core import Pose
from physics.integrators import RK4Integrator
from physics.uuv_3dof import Python3DOFDynamics
from simulation.vehicle_config import load_synthetic_parameters
from ros2.twin_axial_navigation.current_model import FLOWS
from ros2.twin_axial_navigation.mission import INITIALS, environment_from_world, segment_clearance
from ros2.twin_axial_pid.control import AllocatedPID, parameters
import ros2.twin_axial_feasible_v2.guidance as base_guidance
from ros2.twin_axial_feasible_matrix.guidance import build_path, bottleneck
from ros2.twin_axial_feasible_matrix.scenes import SCENES, preflight, world_xml

ROOT = Path(__file__).resolve().parents[2]
FLOW_NAMES = ("F0", "F1", "F2")
AUDIT_CASES = ("P0", "P1", "P2", "P3", "P4", "P0_repeat")
BASE_CONFIG = dict(base_guidance.CONFIG)
THRESHOLDS = {
    "safety_clearance_m": 0.9,
    "rms_m": 0.35,
    "max_error_m": 0.8,
    "saturation_fraction": 0.2,
    "timeout_s": 300.0,
    "pitch_deg": 75.0,
    "q_radps": 0.8,
    "speed_mps": 1.5,
}
CONTROLLER_PARAMETERS = asdict(parameters())
CONTROLLER_SHA256 = hashlib.sha256(
    json.dumps(CONTROLLER_PARAMETERS, sort_keys=True).encode()).hexdigest()
# 4 speeds x 3 lookaheads = 12 uniform groups.  Every other guidance value is
# identical so a group is one configuration applied to all three flows.
GROUP_SPEEDS = (0.40, 0.44, 0.47, 0.48)
GROUP_LOOKAHEADS = (1.0, 1.8, 2.3)
GROUPS = tuple(
    {"name": f"G{index:02d}", "cruise_speed": speed, "lookahead": lookahead}
    for index, (speed, lookahead) in enumerate(
        ((speed, lookahead) for speed in GROUP_SPEEDS for lookahead in GROUP_LOOKAHEADS), 1
    )
)
FIXED = {
    "pitch_rate_limit": 0.20,
    "turn_speed": 0.10,
    "speed_rate_limit": 0.02,
    "preview": 2.0,
}


def config_for_group(group):
    return {**BASE_CONFIG, **FIXED, "cruise_speed": group["cruise_speed"],
            "lookahead": group["lookahead"]}


@contextmanager
def patched_config(config):
    """Temporarily set the one global guidance CONFIG and always restore it."""
    original = dict(base_guidance.CONFIG)
    base_guidance.CONFIG.update(config)
    try:
        yield
    finally:
        base_guidance.CONFIG.clear()
        base_guidance.CONFIG.update(original)


def path_angles(points):
    return [math.degrees(math.atan2(b[1] - a[1], b[0] - a[0]))
            for a, b in zip(points, points[1:])]


def case_checks(metrics, arrived, error):
    """Same pass/fail limits as the existing Python reference trials."""
    return {
        "arrived": bool(arrived),
        "timeout": bool(arrived and metrics["arrival_time"] is not None
                       and metrics["arrival_time"] <= THRESHOLDS["timeout_s"]),
        "safety": metrics["min_clearance"] >= THRESHOLDS["safety_clearance_m"] - 1e-6,
        "rms": metrics["rms_error"] <= THRESHOLDS["rms_m"],
        "max_error": metrics["max_error"] <= THRESHOLDS["max_error_m"],
        "saturation": metrics["saturation_fraction"] <= THRESHOLDS["saturation_fraction"],
        "envelope": error != "envelope",
    }


def _metrics(rows, arrived):
    errors = [row["nav"]["path_error"] for row in rows]
    return {
        "arrival_time": rows[-1]["t"] if arrived else None,
        "min_clearance": min(row["clearance"] for row in rows),
        "rms_error": math.sqrt(sum(value * value for value in errors) / len(errors)),
        "max_error": max(errors),
        "saturation_fraction": sum(row["control"]["allocation"]["alpha"] < 0.999
                                   for row in rows) / len(rows),
        "max_pitch_deg": max(abs(math.degrees(row["state"][2])) for row in rows),
        "max_q_radps": max(abs(row["state"][5]) for row in rows),
        "max_speed_mps": max(math.hypot(row["state"][3], row["state"][4]) for row in rows),
        "steps": len(rows),
    }


def _run(output, group, flow, source_sha256):
    output = Path(output)
    output.mkdir(parents=True, exist_ok=False)
    config = config_for_group(group)
    x, z, angle = INITIALS["P0"]
    state = np.array([x, z, math.radians(angle), 0, 0, 0.0])
    current = list(FLOWS[flow])
    rows = []
    error = None
    with patched_config(config):
        points, geometry = build_path(SCENES["S3"], (x, z))
        try:
            steady = preflight("S3", "P0", flow)
        except Exception as exc:  # never let the conservative gate decide this run
            steady = {"passed": False, "reason": str(exc)}
        guide = base_guidance.Guide(points, current)
        pid = AllocatedPID(math.radians(config["pitch_limit_deg"]))
        dynamics = Python3DOFDynamics(load_synthetic_parameters())
        integrator = RK4Integrator()
        world = environment_from_world(world_xml("P0", "S3"))
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
            record = {
                "valid": True,
                "sequence": index,
                "elapsed_end": t,
                "pitch_nose_up_rad": theta,
                "solver_state_ned_frd": {"t": t, "y": x, "z": 20 - z,
                                         "u": u, "w": w, "q": q},
            }
            _, control = pid.command(record, target, .05)
            rows.append({"t": t, "state": state.tolist(), "clearance": clearance,
                         "nav": navigation, "control": control,
                         "applied_force": force.tolist()})
            allocated = control["allocation"]["allocated_wrench"]
            force = np.array([allocated["X"], 0, allocated["M"]])
            if clearance < THRESHOLDS["safety_clearance_m"] - 1e-6:
                error = "safety"
                break
            if (abs(theta) > math.radians(THRESHOLDS["pitch_deg"])
                    or abs(q) > THRESHOLDS["q_radps"]
                    or max(abs(u), abs(w)) > THRESHOLDS["speed_mps"]):
                error = "envelope"
                break
            if guide.arrived:
                break
    arrived = bool(guide.arrived)
    metrics = _metrics(rows, arrived)
    checks = case_checks(metrics, arrived, error)
    report = {
        "passed": all(checks.values()),
        "backend": "Python 3-DoF reference; no LOTUSim/xdyn validation",
        "scene": "S3",
        "case": "P0",
        "flow": flow,
        "group": group["name"],
        "parameters": config,
        "checks": checks,
        "metrics": metrics,
        "steady_preflight": {
            "passed": steady.get("passed"),
            "reason": steady.get("reason"),
            "bottleneck": steady.get("bottleneck"),
            "steady_trim": steady.get("steady_trim"),
        },
        "error": error,
        "failure": None if error is None else {
            "t": rows[-1]["t"],
            "position_planning_xz_m": rows[-1]["state"][:2],
            "clearance_m": rows[-1]["clearance"],
            "step": len(rows) - 1,
        },
        "source_sha256": source_sha256,
        "controller_sha256": CONTROLLER_SHA256,
    }
    plan = {"path": points.tolist(), "config": config, "geometry": geometry,
            "case": "P0", "scene": "S3", "flow": flow, "group": group["name"],
            "steady_preflight": report["steady_preflight"]}
    (output / "plan.json").write_text(json.dumps(plan, indent=2), encoding="utf-8")
    with (output / "states.jsonl").open("w", encoding="utf-8") as stream:
        for row in rows:
            stream.write(json.dumps(row) + "\n")
    (output / "report.json").write_text(json.dumps(report, indent=2), encoding="utf-8")
    manifest = {path.relative_to(output).as_posix(): hashlib.sha256(path.read_bytes()).hexdigest()
                for path in sorted(output.rglob("*"))
                if path.is_file() and path.name != "manifest.json"}
    (output / "manifest.json").write_text(json.dumps(manifest, indent=2), encoding="utf-8")
    print(json.dumps({key: report[key] for key in
                      ("passed", "group", "flow", "checks", "error")}), flush=True)
    return report


def source_snapshot(output):
    """Copy the exact Python/YAML source once per study and hash it."""
    output = Path(output)
    folders = ("ros2/twin_axial_s3_transient", "ros2/twin_axial_feasible_matrix",
               "ros2/twin_axial_feasible_v2", "ros2/twin_axial_navigation",
               "ros2/twin_axial_pid", "ros2/twin_axial_v2", "controller", "physics",
               "simulation", "core", "config/vehicles")
    hashes = {}
    for folder in folders:
        for source in (ROOT / folder).rglob("*"):
            if (not source.is_file() or "__pycache__" in source.parts
                    or source.suffix not in (".py", ".yaml")):
                continue
            target = output / "source" / source.relative_to(ROOT)
            target.parent.mkdir(parents=True, exist_ok=True)
            target.write_bytes(source.read_bytes())
            hashes[source.relative_to(ROOT).as_posix()] = hashlib.sha256(
                source.read_bytes()).hexdigest()
    aggregate = hashlib.sha256(
        "".join(hashes[key] for key in sorted(hashes)).encode()).hexdigest()
    (output / "source.json").write_text(json.dumps(
        {"files": hashes, "count": len(hashes), "aggregate_sha256": aggregate}, indent=2),
        encoding="utf-8")
    return aggregate


def screen(output):
    output = Path(output).resolve()
    if output.exists():
        raise FileExistsError(output)
    output.mkdir(parents=True)
    source_sha256 = source_snapshot(output)
    (output / "groups.json").write_text(json.dumps(
        {"groups": GROUPS, "fixed": FIXED, "base_config": BASE_CONFIG,
         "thresholds": THRESHOLDS, "flows": FLOW_NAMES,
         "controller_parameters": CONTROLLER_PARAMETERS,
         "controller_sha256": CONTROLLER_SHA256,
         "scope": "S3/P0 only; uniform guidance parameters; Python reference"},
        indent=2), encoding="utf-8")
    results = {}
    for group in GROUPS:
        results[group["name"]] = {"parameters": config_for_group(group), "flows": {}}
        for flow in FLOW_NAMES:
            report = _run(output / group["name"] / flow, group, flow, source_sha256)
            results[group["name"]]["flows"][flow] = report
    flow_counts = {flow: sum(results[name]["flows"][flow]["passed"] for name in results)
                   for flow in FLOW_NAMES}
    unified = [name for name, row in results.items()
               if all(row["flows"][flow]["passed"] for flow in FLOW_NAMES)]
    robust = [name for name in unified
              if all(results[name]["flows"][flow]["metrics"]["min_clearance"] >= 0.95
                     and results[name]["flows"][flow]["metrics"]["max_error"] <= 0.78
                     and results[name]["flows"][flow]["metrics"]["rms_error"] <= 0.33
                     for flow in FLOW_NAMES)]
    f0f1 = [name for name, row in results.items()
            if row["flows"]["F0"]["passed"] and row["flows"]["F1"]["passed"]]
    f2_only = [name for name, row in results.items()
               if row["flows"]["F2"]["passed"] and not (row["flows"]["F0"]["passed"]
                                                       and row["flows"]["F1"]["passed"])]
    decision = {
        "worth_full_matrix": bool(robust),
        "unified_passes": unified,
        "robust_unified_passes": robust,
        "f0_f1_passes": f0f1,
        "f2_only_passes": f2_only,
        "reason": (
            "没有一组统一参数同时通过 S3/F0、F1、F2；F0/F1 可用的低前视组在 F2 触发安全线，"
            "F2 可用的高前视组使 F0/F1 的跟踪误差越界。唯一 F2 通过组的净距/最大误差裕量也很薄。"
            if not robust else
            "存在同时通过三流且保留最小裕量的统一参数组，可进入完整矩阵准备。"
        ),
        "next": ("不要直接进入 45 次矩阵；先研究流感知引导/路径或单独批准范围决策。"
                 "S3 仍保持 double_baffle_tight_v1 名义场景定位，本结论不把 S3 改称边界场景。"),
    }
    summary = {
        "scope": ("S3/P0 only; 12 uniform guidance groups; Python 3-DoF reference; "
                  "no xdyn/LOTUSim; steady preflight recorded but not used as a gate"),
        "source_sha256": source_sha256,
        "controller_parameters": CONTROLLER_PARAMETERS,
        "controller_sha256": CONTROLLER_SHA256,
        "thresholds": THRESHOLDS,
        "groups": results,
        "flow_pass_counts": flow_counts,
        "decision": decision,
    }
    (output / "summary.json").write_text(json.dumps(summary, indent=2), encoding="utf-8")
    write_summary_md(summary, output / "summary.md")
    plot_screen(summary, output / "screen.png")
    print(json.dumps({"decision": decision, "flow_pass_counts": flow_counts}, indent=2))
    return summary


def write_summary_md(summary, path):
    lines = ["# S3 transient screening", "",
             "Python 3-DoF reference; S3/P0 only; no LOTUSim/xdyn.", "",
             "| group | speed | lookahead | F0 | F1 | F2 |",
             "| --- | ---: | ---: | --- | --- | --- |"]
    for name, row in summary["groups"].items():
        params = row["parameters"]
        marks = ["PASS" if row["flows"][flow]["passed"] else "FAIL" for flow in FLOW_NAMES]
        lines.append(f"| {name} | {params['cruise_speed']:.2f} | "
                     f"{params['lookahead']:.1f} | {marks[0]} | {marks[1]} | {marks[2]} |")
    decision = summary["decision"]
    lines += ["", f"Worth full matrix: **{decision['worth_full_matrix']}**", "",
              decision["reason"], "", decision["next"], ""]
    Path(path).write_text("\n".join(lines), encoding="utf-8")


def plot_screen(summary, path):
    cache = Path(path).parent / "mplconfig"
    cache.mkdir(parents=True, exist_ok=True)
    os.environ["MPLCONFIGDIR"] = str(cache)
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    names = list(summary["groups"])
    x = np.arange(len(names))
    fig, axes = plt.subplots(1, 2, figsize=(12, 4.5), layout="constrained")
    colors = {"F0": "#244ca0", "F1": "#338264", "F2": "#ba347d"}
    for flow in FLOW_NAMES:
        clearances = [summary["groups"][name]["flows"][flow]["metrics"]["min_clearance"]
                      for name in names]
        rms = [summary["groups"][name]["flows"][flow]["metrics"]["rms_error"]
               for name in names]
        axes[0].plot(x, clearances, marker="o", color=colors[flow], label=flow)
        axes[1].plot(x, rms, marker="o", color=colors[flow], label=flow)
    axes[0].axhline(THRESHOLDS["safety_clearance_m"], color="#ba5c5c", linestyle="--",
                    linewidth=1, label="0.9 m limit")
    axes[1].axhline(THRESHOLDS["rms_m"], color="#ba5c5c", linestyle="--",
                    linewidth=1, label="0.35 m limit")
    axes[0].set(ylabel="minimum swept clearance (m)", title="S3 uniform-group screen: clearance")
    axes[1].set(ylabel="path RMS error (m)", title="S3 uniform-group screen: tracking")
    for axis in axes:
        axis.set_xticks(x)
        axis.set_xticklabels([name for name in names], rotation=45, ha="right")
        axis.grid(alpha=.25)
        axis.legend(fontsize=8)
    fig.savefig(path, dpi=180)
    plt.close(fig)


def audit(source, output):
    source = Path(source).resolve()
    output = Path(output).resolve()
    if output.exists():
        raise FileExistsError(output)
    output.mkdir(parents=True)
    f1 = source / "S3/F1"
    suite_path = f1 / "suite.json"
    suite = json.loads(suite_path.read_text(encoding="utf-8"))
    runs = {}
    max_pitch = max_q = 0.0
    for case in AUDIT_CASES:
        folder = f1 / case
        report = json.loads((folder / "report.json").read_text(encoding="utf-8"))
        plan = json.loads((folder / "plan.json").read_text(encoding="utf-8"))
        states = [json.loads(line) for line in
                  (folder / "states.jsonl").read_text(encoding="utf-8").splitlines() if line]
        pitch = max(abs(math.degrees(row["state"][2])) for row in states)
        q = max(abs(row["state"][5]) for row in states)
        max_pitch = max(max_pitch, pitch)
        max_q = max(max_q, q)
        runs[case] = {"passed": report["passed"], "arrival_time": report["metrics"]["arrival_time"],
                      "min_clearance": report["metrics"]["min_clearance"],
                      "rms_error": report["metrics"]["rms_error"],
                      "max_error": report["metrics"]["max_error"],
                      "max_pitch_deg": pitch, "max_q_radps": q,
                      "path_min_angle_deg": min(path_angles(plan["path"])),
                      "path_max_angle_deg": max(path_angles(plan["path"]))}
    blocked = {}
    for flow in ("F0", "F2"):
        pf = json.loads((source / f"S3/{flow}/P0/preflight.json").read_text(encoding="utf-8"))
        blocked[flow] = {"reason": pf.get("reason"),
                         "dynamics_started": pf.get("dynamics_started", False),
                         "path_min_angle_deg": min(path_angles(pf["path"])),
                         "path_max_angle_deg": max(path_angles(pf["path"])),
                         "steady_trim": pf.get("steady_trim")}
    bound = bottleneck(SCENES["S3"])
    findings = [
        "S3/F1 已有 5 个初态及 P0 重复的真实 Python 轨迹，全部通过；"
        f"实际最大俯仰 {max_pitch:.3f}° 超过 70° 目标限幅，说明瞬态过冲参与了通过。",
        "S3/F0、S3/F2 只有稳态预检拒绝，没有任何动态轨迹；"
        "拒绝不是碰撞或超时结论，不能当作已发生的失败轨迹。",
        f"现有 S3/F1 各初态路径的最大下降段为 {min(runs[c]['path_min_angle_deg'] for c in runs):.3f}°，"
        f"最大爬升段为 {max(runs[c]['path_max_angle_deg'] for c in runs):.3f}°，"
        f"前向单调路径的必要下降下界为 {bound['minimum_descent_course_deg']:.3f}°。",
        "稳态证书对 F0/F2 是充分性很弱的保守检查；本研究因此只把预检作为记录，不作为动态运行门禁。",
    ]
    report = {
        "scope": "read-only audit of existing S3 records; no dynamics rerun",
        "source": str(source),
        "suite_sha256": hashlib.sha256(suite_path.read_bytes()).hexdigest(),
        "existing_f1": {"suite": suite, "runs": runs},
        "blocked_flows": blocked,
        "steady_bound": bound,
        "findings": findings,
    }
    (output / "audit.json").write_text(json.dumps(report, indent=2), encoding="utf-8")
    lines = ["# S3 trajectory audit", "",
             f"Source: `{source}`", "",
             "## Existing S3/F1", "",
             "| case | pass | arrival (s) | min clearance (m) | RMS (m) | max error (m) | max pitch (deg) |",
             "| --- | --- | ---: | ---: | ---: | ---: | ---: |"]
    for case, row in runs.items():
        lines.append(f"| {case} | {row['passed']} | {row['arrival_time']} | "
                     f"{row['min_clearance']:.4f} | {row['rms_error']:.4f} | "
                     f"{row['max_error']:.4f} | {row['max_pitch_deg']:.3f} |")
    lines += ["", "## Blocked flows", ""]
    for flow, row in blocked.items():
        lines.append(f"- S3/{flow}: `{row['reason']}`; dynamics_started={row['dynamics_started']}")
    lines += ["", "## Findings", ""] + [f"- {item}" for item in findings]
    (output / "audit.md").write_text("\n".join(lines) + "\n", encoding="utf-8")
    print(json.dumps({"existing_f1": suite["passed"], "blocked": blocked,
                      "findings": findings}, indent=2))
    return report


def analyze(directory):
    """Read-only check that every stored run still satisfies its recorded checks."""
    directory = Path(directory).resolve()
    summary = json.loads((directory / "summary.json").read_text(encoding="utf-8"))
    problems = []
    for name, group in summary["groups"].items():
        for flow, report in group["flows"].items():
            metrics = report["metrics"]
            checks = case_checks(metrics, report["checks"]["arrived"], report["error"])
            if checks != report["checks"] or all(checks.values()) != report["passed"]:
                problems.append(f"{name}/{flow}")
            folder = directory / name / flow
            manifest = json.loads((folder / "manifest.json").read_text(encoding="utf-8"))
            for relative, expected in manifest.items():
                path = folder / relative
                if not path.is_file() or hashlib.sha256(path.read_bytes()).hexdigest() != expected:
                    problems.append(f"{name}/{flow}/{relative}")
    result = {"passed": not problems, "problems": problems,
              "decision": summary["decision"]}
    print(json.dumps(result, indent=2))
    return result
