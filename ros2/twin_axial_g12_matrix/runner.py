# Derived from the validated loop; only scene selection and failure-prefix recording differ.
"""Frozen G12 state loop generalized to all three unchanged scene geometries."""
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
from ros2.twin_axial_s3_transient_v2.cli import original_rates, read, sha, snapshot, verify_run, write

FLOWS = ("F0", "F1", "F2")
INITIAL_CASES = ("P0", "P1", "P2", "P3", "P4")
NEW_CASES = ("P1", "P2", "P3", "P4", "P0_repeat")
G12 = study.GROUPS[11]
assert G12["name"] == "G12"


def g12_config():
    with original_rates():
        return study.config_for_group(G12)


def run_case(output, scene, flow, case, source_sha256):
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
            points, geometry = study.build_path(study.SCENES[scene], (x, z))
            try:
                steady = study.preflight(scene, initial_case, flow)
            except Exception as exc:
                steady = {"passed": False, "reason": str(exc)}
            guide = study.base_guidance.Guide(points, current)
            pid = AllocatedPID(math.radians(config["pitch_limit_deg"]))
            dynamics = Python3DOFDynamics(load_synthetic_parameters())
            integrator = RK4Integrator()
            world = environment_from_world(study.world_xml(initial_case, scene))
            previous = Pose(x=x, z=z)
            force = np.zeros(3)
            state_stream = (output / "states.jsonl").open("w", encoding="utf-8", buffering=1)
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
                state_stream.write(json.dumps(rows[-1]) + "\n")
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
    state_stream.close()
    arrived = bool(guide.arrived)
    metrics = study._metrics(rows, arrived)
    checks = study.case_checks(metrics, arrived, error)
    report = {
        "passed": all(checks.values()),
        "backend": "Python 3-DoF reference; no LOTUSim/xdyn validation",
        "scene": scene, "case": initial_case, "flow": flow, "group": "G12",
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
            "case": initial_case, "scene": scene, "flow": flow, "group": "G12",
            "run_case": case, "steady_preflight": report["steady_preflight"]}
    write(output / "plan.json", plan)
    write(output / "report.json", report)
    manifest = {path.relative_to(output).as_posix(): sha(path)
                for path in sorted(output.rglob("*"))
                if path.is_file() and path.name != "manifest.json"}
    write(output / "manifest.json", manifest)
    print(json.dumps({"passed": report["passed"], "scene": scene, "flow": flow, "run_case": case,
                      "checks": checks, "error": error}), flush=True)
    return report


