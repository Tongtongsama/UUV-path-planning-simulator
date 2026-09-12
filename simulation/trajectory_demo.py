"""Generate and plot v1/v2 planner-to-trajectory reference scenarios."""
import argparse
from dataclasses import asdict
import json
from pathlib import Path

from planner import AStarPlanner
from trajectory import TrajectoryConfig, generate_trajectory
from validation.planner_acceptance import ROOT, build_case


def main() -> None:
    """Write all scenario records and PNG figures to a new directory."""
    from matplotlib.figure import Figure
    from matplotlib.backends.backend_agg import FigureCanvasAgg
    from matplotlib.patches import Rectangle
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output",type=Path,required=True)
    args=parser.parse_args()
    args.output.mkdir(parents=True,exist_ok=False)
    records=[]
    for version in ("v1","v2"):
        config=json.loads((ROOT/f"config/scenarios/planner_acceptance_{version}.json").read_text(encoding="utf-8"))
        for case in config["cases"]:
            space,request,settings=build_case(config,case)
            plan=AStarPlanner(settings).plan(request,space)
            if plan.status.name != case["expected_status"]:
                raise RuntimeError(f"Unexpected planner outcome: {case['name']}")
            record={"version":version,"case":case,"config":config,
                    "planning_status":plan.status.name,"trajectory":None}
            if not plan.success:
                records.append(record)
                continue
            result=generate_trajectory(plan.path,space,request.constraints,request=request)
            record.update(trajectory=[asdict(s) for s in result.trajectory],
                raw_path=[asdict(p) for p in result.raw_path],
                shortcut_path=[asdict(p) for p in result.shortcut_path],
                trajectory_config=asdict(TrajectoryConfig()),duration=result.trajectory.duration)
            records.append(record)
            fig=Figure(figsize=(10,6),layout="constrained")
            FigureCanvasAgg(fig)
            gs=fig.add_gridspec(2,2,width_ratios=[1.4,1])
            ax=fig.add_subplot(gs[:,0]); speed=fig.add_subplot(gs[0,1]); pitch=fig.add_subplot(gs[1,1])
            b=config["boundary"]
            for o in case["obstacles"]:
                ax.add_patch(Rectangle((o["x_min"],o["z_min"]),o["x_max"]-o["x_min"],o["z_max"]-o["z_min"],color="#697581"))
            for path,label,style,color in ((result.raw_path,"Raw A*","--","#8492a0"),
                (result.shortcut_path,"Safe shortcut","-","#1761b0")):
                ax.plot([p.x for p in path],[p.z for p in path],style,color=color,label=label)
            t=result.trajectory
            ax.scatter([s.pose.x for s in t],[s.pose.z for s in t],s=9,color="#11977b",label="Reference samples")
            ax.set(xlim=(b["x_min"],b["x_max"]),ylim=(b["z_min"],b["z_max"]),
                xlabel="East x (m)",ylabel="Up z (m)",aspect="equal")
            ax.legend(fontsize=8,loc="upper center",bbox_to_anchor=(.5,-.12))
            times=[s.timestamp for s in t]
            speed.step(times,[s.twist.u for s in t],where="post",color="#1761b0")
            pitch.step(times,[s.pose.pitch for s in t],where="post",color="#11977b")
            speed.set(xlabel="Time (s)",ylabel="Reference u (m/s)",ylim=(-.05,.6))
            pitch.set(xlabel="Time (s)",ylabel="Reference pitch (rad)")
            fig.suptitle(f"{case['name']} | {t.duration:.2f} s\nPiecewise-linear reference; corner attitude jumps are not smoothed",fontsize=12)
            fig.savefig(args.output/f"{case['name']}.png",dpi=160)
            fig.clear()
            print(f"{case['name']}: {len(plan.path)} -> {len(result.shortcut_path)} corners; {len(t)} states; {t.duration:.3f} s")
    (args.output/"results.json").write_text(json.dumps(records,indent=2,allow_nan=False),encoding="utf-8")


if __name__ == "__main__":
    main()
