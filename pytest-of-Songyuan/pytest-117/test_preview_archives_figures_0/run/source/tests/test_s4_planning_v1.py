import numpy as np
from core import Pose
from environment import Boundary, Obstacle, WorldModel
from planner import EnvironmentPlanningSpace, PlanningConstraints, validate_path
from ros2.twin_axial_navigation.mission import environment_from_world, obstacle_extents
from ros2.twin_axial_s4_planning_v1.study import world, world_xml, smooth, path_of, SPEC

def test_s4_geometry_matches_runner_sdf():
    assert obstacle_extents(environment_from_world(world_xml())) == SPEC['obstacles_xmin_xmax_zmin_zmax']
    assert world().boundary.bounds == (0,-20,20,0)

def test_unsafe_corner_is_retained():
    w=WorldModel(Boundary.rectangle(x_min=-10,x_max=10,z_min=-10,z_max=10),
                 [Obstacle.rectangle('box',x_min=0,x_max=3,z_min=0,z_max=3)])
    space=EnvironmentPlanningSpace(w); limits=PlanningConstraints(0,0)
    # Corner cuts through the box: endpoints alone cannot validate a shortcut.
    raw=path_of([[-2,1],[-1,-1],[1,-2],[4,-2]])
    result,audit=smooth(raw,space,limits,None)
    assert validate_path(result,space,limits).valid
    assert np.allclose([result[0].x,result[0].z],[-2,1])
    assert np.allclose([result[-1].x,result[-1].z],[4,-2])

def test_rounding_rejects_segment_cutting_obstacle():
    w=WorldModel(Boundary.rectangle(x_min=-10,x_max=10,z_min=-10,z_max=10),
                 [Obstacle.rectangle('box',x_min=.01,x_max=3,z_min=.01,z_max=3)])
    space=EnvironmentPlanningSpace(w); limits=PlanningConstraints(0,0)
    raw=path_of([[0,4],[0,0],[4,0]])
    result,audit=smooth(raw,space,limits,None)
    assert audit[0]['smoothed'] is False
    assert len(audit[0]['attempts']) == 4
    assert validate_path(result,space,limits).valid

def test_free_corner_rounds_without_moving_endpoints():
    w=WorldModel(Boundary.rectangle(x_min=-10,x_max=10,z_min=-10,z_max=10),[])
    result,audit=smooth(path_of([[-4,0],[0,0],[0,4]]),EnvironmentPlanningSpace(w),PlanningConstraints(.7,.8),None)
    assert audit[0]['smoothed']
    assert (result[0].x,result[0].z)==(-4,0)
    assert (result[-1].x,result[-1].z)==(0,4)
