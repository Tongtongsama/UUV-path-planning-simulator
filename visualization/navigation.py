"""Read-only closed-loop scientific figures and bounded-size GIF animation."""
from pathlib import Path
import numpy as np
from matplotlib.figure import Figure
from matplotlib.backends.backend_agg import FigureCanvasAgg
from matplotlib.patches import Rectangle, Circle
from matplotlib.animation import FuncAnimation, PillowWriter


def _world(ax, scene, request, goal_tolerance):
    b=scene["boundary"]
    for o in scene["obstacles"]:
        ax.add_patch(Rectangle((o["x_min"],o["z_min"]),o["x_max"]-o["x_min"],o["z_max"]-o["z_min"],color="#626d79"))
    ax.add_patch(Circle((request.goal.x,request.goal.z),goal_tolerance,color="#40a877",alpha=.3))
    ax.scatter([request.initial_state.pose.x],[request.initial_state.pose.z],color="#173958",marker="o",label="Start")
    ax.scatter([request.goal.x],[request.goal.z],color="#13946b",marker="*",s=90,label="Goal")
    ax.set(xlim=(b["x_min"],b["x_max"]),ylim=(b["z_min"],b["z_max"]),
           xlabel="East x (m)",ylabel="Up z (m)",aspect="equal")


def plot_navigation(result, request, scene: dict, config, output: Path) -> None:
    """Environment/reference/actual path, synchronous errors, pitch and controls."""
    fig=Figure(figsize=(12,8),layout="constrained"); FigureCanvasAgg(fig)
    grid=fig.add_gridspec(3,2,width_ratios=[1.15,1])
    ax=fig.add_subplot(grid[:,0]); err=fig.add_subplot(grid[0,1]); pitch=fig.add_subplot(grid[1,1]); ctrl=fig.add_subplot(grid[2,1])
    _world(ax,scene,request,config.goal_position_tolerance)
    if result.planning_result and result.planning_result.path:
        p=result.planning_result.path
        ax.plot([s.x for s in p],[s.z for s in p],"--",color="#9cabb6",label="Raw A*")
    if result.trajectory:
        ax.plot([s.pose.x for s in result.trajectory],[s.pose.z for s in result.trajectory],color="#168a9b",label="Reference")
    states=result.state_history; refs=result.reference_history
    ax.plot([s.pose.x for s in states],[s.pose.z for s in states],color="#dd702b",lw=2,label="Executed")
    last=states[-1].pose
    ax.arrow(last.x,last.z,.4*np.cos(last.pitch),.4*np.sin(last.pitch),width=.03,color="#b22d30",length_includes_head=True)
    ax.legend(fontsize=8,loc="upper center",bbox_to_anchor=(.5,-.1),ncol=2)
    time=[s.timestamp for s in states]
    if len(refs)==len(states):
        err.plot(time,[s.pose.x-r.pose.x for s,r in zip(states,refs)],label="x error")
        err.plot(time,[s.pose.z-r.pose.z for s,r in zip(states,refs)],label="z error")
        pitch.plot(time,[s.pose.pitch for s in states],label="Actual")
        pitch.plot(time,[r.pose.pitch for r in refs],"--",label="Reference")
    err.set(ylabel="Position error (m)"); pitch.set(ylabel="Pitch (rad)")
    for axis,limit,color in (("tau_x",config.maximum_tau_x,"#2369ac"),("tau_m",config.maximum_tau_m,"#bb6a24")):
        ctrl.plot([s.time for s in result.control_history],[getattr(s.commanded,axis) for s in result.control_history],":",color=color,label=axis+" commanded")
        ctrl.plot([s.time for s in result.control_history],[getattr(s.applied,axis) for s in result.control_history],color=color,label=axis+" applied")
        ctrl.axhline(limit,ls="--",color=color,alpha=.4); ctrl.axhline(-limit,ls="--",color=color,alpha=.4)
    ctrl.set(ylabel="Force (N) / moment (N m)")
    for a in (err,pitch,ctrl):
        a.set_xlabel("Simulation time (s)"); a.legend(fontsize=7,ncol=2); a.grid(alpha=.15)
    m=result.metrics
    current=request.environment.current_at(request.initial_state.pose,request.initial_state.timestamp)
    fig.suptitle(f"{scene['name']} | {result.status.name} | current {tuple(float(v) for v in current)} m/s\n"
        f"Final error {m['terminal_position_error']:.3f} m | elapsed {m['simulation_duration']:.2f} s | "
        f"saturation {m['saturation_fraction']:.1%}",fontsize=13)
    # Case identifiers may contain decimal points; append, never replace a suffix.
    fig.savefig(output.parent / (output.name + ".png"),dpi=140)
    fig.savefig(output.parent / (output.name + ".svg")); fig.clear()


def animate_navigation(result, request, scene: dict, config, output: Path) -> None:
    """Up to 100 uniformly selected frames; animation is illustrative only."""
    fig=Figure(figsize=(7,7),layout="constrained"); FigureCanvasAgg(fig)
    ax=fig.subplots(); _world(ax,scene,request,config.goal_position_tolerance)
    states=result.state_history; refs=result.reference_history
    if result.trajectory:
        ax.plot([s.pose.x for s in result.trajectory],[s.pose.z for s in result.trajectory],"--",color="#148e9d")
    line,=ax.plot([],[],color="#dd702b"); vehicle,=ax.plot([],[],color="#b42d30",lw=3)
    reference,=ax.plot([],[],"o",color="#148e9d"); title=ax.set_title("")
    frames=np.unique(np.linspace(0,len(states)-1,min(100,len(states)),dtype=int))
    def update(i):
        s=states[i]; p=s.pose
        line.set_data([v.pose.x for v in states[:i+1]],[v.pose.z for v in states[:i+1]])
        vehicle.set_data([p.x-.2*np.cos(p.pitch),p.x+.2*np.cos(p.pitch)],
                         [p.z-.2*np.sin(p.pitch),p.z+.2*np.sin(p.pitch)])
        error=0.
        if len(refs)==len(states):
            r=refs[i].pose; reference.set_data([r.x],[r.z]); error=np.hypot(p.x-r.x,p.z-r.z)
        clearance=result.control_history[i-1].segment_clearance if i else request.environment.clearance(p)
        title.set_text(f"{scene['name']} | t={s.timestamp:.2f} s\nerror={error:.3f} m | clearance={clearance:.3f} m | "
                       f"{result.status.name if i==len(states)-1 else 'RUNNING'}")
        return line,vehicle,reference,title
    movie=FuncAnimation(fig,update,frames=frames,interval=100,blit=False)
    movie.save(output,writer=PillowWriter(fps=10)); fig.clear()
