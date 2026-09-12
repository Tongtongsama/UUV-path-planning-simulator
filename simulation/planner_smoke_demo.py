"""Reproducible A* obstacle detour and unreachable-goal smoke cases."""
from core import Pose, Twist, VehicleState
from environment import Boundary, Obstacle, WorldModel
from planner import (AStarPlanner, EnvironmentPlanningSpace, PlanningConstraints,
                     PlanningRequest, PlanningStatus, validate_path)


def main() -> None:
    """Plan and validate two static x-z scenarios without trajectory generation."""
    request = PlanningRequest(VehicleState(Pose(x=1,z=-5),Twist.zero()),
                              Pose(x=9,z=-5),PlanningConstraints(.2))
    for blocked in (False,True):
        boundary = Boundary.rectangle(x_min=0,x_max=10,z_min=-10,z_max=0)
        obstacle = Obstacle.rectangle("wall",x_min=4.9,x_max=5.1,
            z_min=-10 if blocked else -7,z_max=0 if blocked else -3)
        space = EnvironmentPlanningSpace(WorldModel(boundary,[obstacle]))
        result = AStarPlanner().plan(request,space)
        expected = PlanningStatus.NO_PATH if blocked else PlanningStatus.SUCCESS
        if result.status is not expected:
            raise RuntimeError(f"Unexpected status: {result}")
        if result.success and not validate_path(result.path,space,request.constraints,request=request).valid:
            raise RuntimeError("Invalid output path")
        print(f"{'blocked' if blocked else 'detour'}: {result.status.name}, "
              f"length={result.path_cost}, expanded={result.expanded_nodes}")


if __name__ == "__main__":
    main()
