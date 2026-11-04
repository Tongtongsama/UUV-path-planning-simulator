"""S2 geometry-derived candidate, feasibility screening and preview guidance."""
import math
import numpy as np
from shapely.geometry import LineString, box
from controller.guidance import BodyGuidance3DOF

CONFIG=dict(revision='feasible_v1',pitch_limit_deg=70.,lookahead=1.0,
            turn_speed=.12,cruise_speed=.4,preview=2.0,corner_trim=1.5,
            pitch_rate_limit=.12,speed_rate_limit=.04)

def executable_path(points,current,speed=None):
    """Check segment trim at cruise; full transient feasibility requires a run."""
    speed=CONFIG['cruise_speed'] if speed is None else speed
    if not 0<speed<=.6:raise ValueError('Cruise outside existing speed range')
    pts=np.asarray(points,float);current=np.asarray(current,float)
    if pts.ndim!=2 or pts.shape[1]!=2 or len(pts)<2 or not np.isfinite(pts).all():raise ValueError('Invalid path')
    limits=math.radians(CONFIG['pitch_limit_deg']);required=[];forces=[]
    for delta in np.diff(pts,axis=0):
        if np.linalg.norm(delta)<1e-10:raise ValueError('Zero length segment')
        chi=math.atan2(delta[1],delta[0])
        def course(theta):
            cw=current[0]*math.sin(theta)-current[1]*math.cos(theta)
            return theta-math.atan2(cw,speed)
        if not course(-limits)<=chi<=course(limits):
            raise ValueError(f'Path segment {math.degrees(chi):.3f} deg has no cruise trim within 70 deg')
        lo,hi=-limits,limits
        for _ in range(50):
            theta=(lo+hi)/2
            if course(theta)<chi:lo=theta
            else:hi=theta
        ur=speed-current[0]*math.cos(theta)-current[1]*math.sin(theta)
        X=4*ur+ur*abs(ur);M=6*math.sin(theta)
        thrust=max(abs(X/2-2*M),abs(X/2+2*M))
        if thrust>15:raise ValueError('Cruise trim exceeds unchanged thruster limit')
        required.append(abs(math.degrees(theta)));forces.append(thrust)
    return dict(cruise_trim_pitch_headroom_deg=70-max(required),
                cruise_trim_max_thrust_N=max(forces),
                scope='Segment steady trim only; not transient navigation certification')

def build_path(obstacles, start=(2,-10), goal=(18,-10)):
    """Limited to the documented two alternating rectangular-baffle topology.

    This is an explicitly scoped candidate generator, not a general replacement A*.
    Geometry and full segment clearance are checked before any controller starts.
    """
    if len(obstacles)!=2: raise ValueError('Requires two alternating baffles')
    a,b=obstacles
    if not (a[2]==-20 and b[3]==0 and a[1]<b[0]):raise ValueError('Unsupported topology')
    midpoint=np.array([(a[1]+b[0])/2,(a[3]+b[2])/2])
    radius=1.55; delta=np.array([b[0]-a[1],b[2]-a[3]])
    slope=math.atan2(-delta[1],delta[0])+math.asin(2*radius/np.linalg.norm(delta))
    top=a[3]+2.5;bottom=b[2]-2.5
    topx=midpoint[0]-(top-midpoint[1])/math.tan(slope)
    bottomx=midpoint[0]+(midpoint[1]-bottom)/math.tan(slope)
    anchors=np.array([start,[a[0]-1.5,top],[topx,top],[bottomx,bottom],[b[1]+1.5,bottom],goal],float)
    pts=[anchors[0]]
    for i in range(1,len(anchors)-1):
        prev,p,nxt=anchors[i-1:i+2]
        cut=min(CONFIG['corner_trim'],.4*np.linalg.norm(p-prev),.4*np.linalg.norm(nxt-p))
        entry=p+(prev-p)*cut/np.linalg.norm(prev-p)
        leave=p+(nxt-p)*cut/np.linalg.norm(nxt-p)
        pts.append(entry)
        for t in np.linspace(0,1,31)[1:]:pts.append((1-t)**2*entry+2*t*(1-t)*p+t*t*leave)
    pts.append(anchors[-1]);pts=np.array(pts)
    # Resample uniformly; retain original vertices so no unchecked shortcut appears.
    dense=[pts[0]]
    for x,y in zip(pts,pts[1:]):
        for t in np.linspace(0,1,max(2,math.ceil(np.linalg.norm(y-x)/.05)+1))[1:]:dense.append(x+t*(y-x))
    dense=np.array(dense); line=LineString(dense)
    clear=min(line.distance(box(o[0],o[2],o[1],o[3])) for o in obstacles)
    boundary=float(np.min(np.column_stack([dense[:,0],20-dense[:,0],dense[:,1]+20,-dense[:,1]])))
    if min(clear,boundary)<1.5-1e-9:raise ValueError(f'Candidate planning clearance {clear}, boundary {boundary}')
    return dense,dict(planning_clearance_m=clear,boundary_clearance_m=boundary)

class Guide:
    def __init__(self,points,current):
        self.points=np.array(points);self.delta=np.diff(self.points,axis=0)
        self.length=np.linalg.norm(self.delta,axis=1);self.s=np.r_[0,np.cumsum(self.length)]
        self.tangent=self.delta/self.length[:,None];self.current=np.array(current)
        self.progress=0.;self.pitch=0.;self.speed=0.;self.hold=None;self.arrived=False
        self.last_t=None

    def point(self,s):
        j=min(len(self.length)-1,max(0,int(np.searchsorted(self.s,s,side='right')-1)))
        return self.points[j]+self.tangent[j]*min(self.length[j],max(0,s-self.s[j])),j

    def update(self,state,t):
        if len(state)!=6 or not np.isfinite(state).all() or not math.isfinite(t):raise ValueError('Invalid feedback')
        if self.last_t is not None and abs(t-self.last_t-.05)>1e-6:raise ValueError('Noncontiguous feedback time')
        self.last_t=t
        pos=np.array(state[:2]);theta,u,w,q=state[2:]
        fractions=np.clip(np.sum((pos-self.points[:-1])*self.delta,axis=1)/self.length**2,0,1)
        projected=self.points[:-1]+fractions[:,None]*self.delta
        dist=np.linalg.norm(projected-pos,axis=1)
        arcs=self.s[:-1]+fractions*self.length
        eligible=(arcs>=self.progress-.2)&(arcs<=self.progress+3.)
        ids=np.where(eligible)[0];j=int(ids[np.argmin(dist[ids])])
        self.progress=max(self.progress,float(arcs[j]))
        distance=float(np.linalg.norm(pos-self.points[-1]))
        self.hold=(t if self.hold is None else self.hold) if distance<=.5 else None
        self.arrived=self.arrived or (self.hold is not None and t-self.hold>=2-1e-9)
        target,k=self.point(self.progress+CONFIG['lookahead'])
        chi=math.atan2(*(target-pos)[::-1])
        headings=[math.atan2(*self.tangent[self.point(self.progress+d)[1]][::-1]) for d in np.linspace(0,CONFIG['preview'],12)]
        bend=max(headings)-min(headings)
        speed=CONFIG['turn_speed']+(CONFIG['cruise_speed']-CONFIG['turn_speed'])*max(0,1-bend/.35)
        # Measured transverse motion includes current and turn-induced sideslip.
        # Desired ground direction chi = theta - atan2(w,u).
        current_w=self.current[0]*math.sin(theta)-self.current[1]*math.cos(theta)
        residual_w=w-current_w
        pitch=chi+math.atan2(current_w+residual_w,max(.08,u))
        uncompensated_pitch=pitch
        pitch=max(-math.radians(70),min(math.radians(70),pitch))
        if distance<1.5:speed=min(speed,max(.08,.25*distance))
        if t<2 or self.arrived:speed=0.;pitch=0.
        if distance<.3:speed=0.;pitch=0.
        dp=max(-CONFIG['pitch_rate_limit']*.05,min(CONFIG['pitch_rate_limit']*.05,pitch-self.pitch))
        self.pitch+=dp
        self.speed+=max(-CONFIG['speed_rate_limit']*.05,min(CONFIG['speed_rate_limit']*.05,speed-self.speed))
        return BodyGuidance3DOF(self.speed,self.pitch,dp/.05),dict(progress_s=self.progress,
            desired_course=chi,target_speed=self.speed,target_pitch=self.pitch,target_q=dp/.05,
            current_xz=self.current.tolist(),current_body_w=current_w,residual_body_w=residual_w,
            uncompensated_limit_pitch=uncompensated_pitch,pitch_limited=abs(uncompensated_pitch)>math.radians(70),
            bend_preview=bend,path_error=float(dist.min()),goal_distance=distance)
