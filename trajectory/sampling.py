"""Reference selection policy outside Core, callable by Simulation."""
from bisect import bisect_right
from core import Pose, Trajectory, Twist, VehicleState
from .generator import finite


def sample_reference(trajectory: Trajectory, time: float) -> VehicleState:
    """Sample a generated polyline: linear position, outgoing-segment attitude.

    At internal knots use the outgoing state. Before start and at/after end,
    hold endpoint pose with zero velocity. q is zero on open straight segments;
    corner attitude jumps are explicit, not finite-rate rotation commands.
    This policy is for generate_trajectory outputs, not arbitrary smooth data.
    """
    if not isinstance(trajectory,Trajectory):
        raise TypeError("trajectory must be Trajectory")
    time=finite(time,"time")
    if time < trajectory.start_time:
        return VehicleState(trajectory[0].pose,Twist.zero(),time)
    if time >= trajectory.end_time:
        return VehicleState(trajectory[-1].pose,Twist.zero(),time)
    index=bisect_right([s.timestamp for s in trajectory],time)-1
    a,b=trajectory[index],trajectory[index+1]
    f=(time-a.timestamp)/(b.timestamp-a.timestamp)
    pose=Pose(x=a.pose.x+f*(b.pose.x-a.pose.x),z=a.pose.z+f*(b.pose.z-a.pose.z),pitch=a.pose.pitch)
    return VehicleState(pose,a.twist,time)
