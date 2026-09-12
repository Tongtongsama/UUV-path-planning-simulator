"""Headless scientific plots for rectangular A* acceptance scenarios."""
from pathlib import Path

from matplotlib.figure import Figure
from matplotlib.backends.backend_agg import FigureCanvasAgg
from matplotlib.patches import Polygon, Rectangle, Circle
from shapely.geometry import box


def plot_planning_case(config: dict, case: dict, grid, request, result,
                       output_stem: Path) -> None:
    """Render PNG/SVG from the exact scenario data used to build the world.

    This renderer supports the rectangular acceptance schema, not arbitrary
    WorldModel shapes. Inflation is display geometry; queries remain authoritative.
    """
    figure = Figure(figsize=(9, 7.5), layout="constrained")
    FigureCanvasAgg(figure)
    ax = figure.subplots()
    b = config["boundary"]
    x0,x1,z0,z1 = b["x_min"],b["x_max"],b["z_min"],b["z_max"]
    radius = request.constraints.required_clearance
    ax.add_patch(Rectangle((x0,z0),x1-x0,z1-z0,fill=False,color="#263442",lw=1.8,label="Boundary"))
    if radius:
        ax.add_patch(Rectangle((x0+radius,z0+radius),x1-x0-2*radius,z1-z0-2*radius,
            fill=False,ls="--",color="#ba6a1c",lw=1,label="Boundary clearance"))
    for i,o in enumerate(case["obstacles"]):
        geometry = box(o["x_min"],o["z_min"],o["x_max"],o["z_max"])
        ax.add_patch(Polygon(list(geometry.buffer(radius).exterior.coords),
            color="#edb96b",alpha=.45,label="Obstacle inflation" if i==0 else None))
        ax.add_patch(Polygon(list(geometry.exterior.coords),color="#555d66",
            label="Obstacle" if i==0 else None))
    for occupied, color, marker, label in ((False,"#adbbc8",".","Free lattice node"),
                                           (True,"#b76a62","x","Blocked lattice node")):
        points = [grid.index_to_pose((i,j)) for i in range(grid.shape[0])
            for j in range(grid.shape[1]) if grid.occupied[i][j] == occupied]
        ax.scatter([p.x for p in points],[p.z for p in points],s=8,marker=marker,
                   color=color,linewidths=.6,label=label,zorder=2)
    start,goal = request.start.pose,request.goal
    ax.add_patch(Circle((goal.x,goal.z),request.constraints.goal_tolerance,
                       facecolor="#67bc92",alpha=.3,label="Goal tolerance",zorder=3))
    if result.path is not None:
        ax.plot([p.x for p in result.path],[p.z for p in result.path],"o-",
            color="#155fbc",ms=3,lw=2,label="A* path (validated, unprocessed)",zorder=4)
    ax.scatter([start.x],[start.z],s=100,marker="o",color="#172f50",label="Start",zorder=5)
    ax.scatter([goal.x],[goal.z],s=150,marker="*",color="#098650",label="Goal centre",zorder=5)
    length = "no path" if result.path is None else f"length {result.path_cost:.3f} m"
    ax.set_title(f"{case['name'].replace('_',' ').title()} | {result.status.name}\n"
                 f"{length} | expanded {result.expanded_nodes} | clearance {radius:.2f} m",fontsize=12)
    ax.set(xlabel="World x / East (m)",ylabel="World z / Up (m)",
           xlim=(x0-.35,x1+.35),ylim=(z0-.35,z1+.35),aspect="equal")
    ax.legend(loc="upper center",bbox_to_anchor=(.5,-.1),ncol=3,fontsize=8,frameon=False)
    figure.savefig(output_stem.with_suffix(".png"),dpi=180)
    figure.savefig(output_stem.with_suffix(".svg"))
    figure.clear()
