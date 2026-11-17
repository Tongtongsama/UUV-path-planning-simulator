"""Scene adapter only; retain feasible_v2 guidance and its fixed configuration."""
import math
import numpy as np
from shapely.geometry import LineString, box
from ros2.twin_axial_feasible_v2.guidance import CONFIG, Guide, executable_path
from ros2.twin_axial_feasible_v2.guidance import build_path as alternating_path

def bottleneck(obstacles, clearance=1.5):
    a,b=obstacles;dx=b[0]-a[1];drop=a[3]-b[2]
    angle=math.atan2(drop,dx)+math.asin(2*clearance/math.hypot(dx,drop))
    return dict(horizontal_gap_m=dx,vertical_overlap_m=drop,
                clearance_m=clearance,minimum_descent_course_deg=math.degrees(angle),
                scope='Necessary bound for forward-monotone bounded-slope paths, not all transient maneuvers')

def build_path(obstacles,start=(2,-10),goal=(18,-10)):
    if len(obstacles)==2:return alternating_path(obstacles,start,goal)
    if len(obstacles)!=1 or obstacles[0][2]!=-20:raise ValueError('Unsupported obstacle topology')
    a=obstacles[0];top=a[3]+2.5
    anchors=np.array([start,[a[0]-1.5,top],[a[1]+1.5,top],goal],float);pts=[anchors[0]]
    for i in range(1,len(anchors)-1):
        prev,p,nxt=anchors[i-1:i+2]
        cut=min(CONFIG['corner_trim'],.4*np.linalg.norm(p-prev),.4*np.linalg.norm(nxt-p))
        entry=p+(prev-p)*cut/np.linalg.norm(p-prev);leave=p+(nxt-p)*cut/np.linalg.norm(nxt-p)
        pts.append(entry)
        for t in np.linspace(0,1,31)[1:]:pts.append((1-t)**2*entry+2*t*(1-t)*p+t*t*leave)
    pts.append(anchors[-1]);dense=[pts[0]]
    for x,y in zip(pts,pts[1:]):
        for t in np.linspace(0,1,max(2,math.ceil(np.linalg.norm(y-x)/.05)+1))[1:]:dense.append(x+t*(y-x))
    dense=np.array(dense);line=LineString(dense)
    clearance=min(line.distance(box(o[0],o[2],o[1],o[3])) for o in obstacles)
    boundary=float(np.min(np.column_stack([dense[:,0],20-dense[:,0],dense[:,1]+20,-dense[:,1]])))
    if min(clearance,boundary)<1.5-1e-9:raise ValueError('Candidate planning clearance failed')
    return dense,dict(planning_clearance_m=clearance,boundary_clearance_m=boundary)
