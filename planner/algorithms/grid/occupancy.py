"""Finite world x-z lattice with footprint-aware node occupancy."""
from math import floor, ceil, isfinite
from numbers import Integral
from core import Pose
from planner._checks import scalar, check_pose
from planner.space import BoundedPlanningSpace

Index = tuple[int, int]


class OccupancyGrid:
    """Nodes at lower bound + index * resolution; storage order [ix][iz].

    Occupancy certifies nodes, not whole cells or edges. Non-multiple upper
    bounds leave a final strip connected through the last lattice nodes.
    The environment must remain static throughout construction and search.
    """
    def __init__(self, space: BoundedPlanningSpace, resolution: float,
                 required_clearance: float = 0.0, max_nodes: int = 250000) -> None:
        if not isinstance(space, BoundedPlanningSpace):
            raise TypeError("space must satisfy BoundedPlanningSpace")
        self.resolution = scalar(resolution, "resolution")
        if self.resolution == 0:
            raise ValueError("resolution must be positive")
        self.required_clearance = scalar(required_clearance, "required_clearance")
        if isinstance(max_nodes, bool) or not isinstance(max_nodes, Integral):
            raise TypeError("max_nodes must be integer")
        if max_nodes < 1:
            raise ValueError("max_nodes must be positive")
        bounds = tuple(space.bounds)
        if len(bounds) != 4:
            raise ValueError("bounds must contain four numbers")
        self.bounds = tuple(scalar(v, "bounds", False) for v in bounds)
        xmin, zmin, xmax, zmax = self.bounds
        if xmin >= xmax or zmin >= zmax:
            raise ValueError("bounds must have positive width and height")
        sizes = ((xmax-xmin)/self.resolution, (zmax-zmin)/self.resolution)
        if not all(isfinite(v) for v in sizes):
            raise OverflowError("grid size exceeds resource limit")
        self.shape = (floor(sizes[0])+1, floor(sizes[1])+1)
        if self.shape[0]*self.shape[1] > max_nodes:
            raise OverflowError("grid size exceeds max_grid_nodes")
        self.occupied = tuple(tuple(not space.is_pose_valid(
            self.index_to_pose((i, j)), self.required_clearance)
            for j in range(self.shape[1])) for i in range(self.shape[0]))

    def index_to_pose(self, index: Index) -> Pose:
        """Convert an in-range integer pair to a zero-pitch world Pose."""
        if len(index) != 2 or any(isinstance(v, bool) or not isinstance(v, Integral) for v in index):
            raise TypeError("index must contain two integers")
        i, j = index
        if not (0 <= i < self.shape[0] and 0 <= j < self.shape[1]):
            raise ValueError("grid index out of bounds")
        return Pose(x=self.bounds[0]+i*self.resolution,
                    z=self.bounds[1]+j*self.resolution)

    def world_to_index(self, pose: Pose) -> Index:
        """Nearest lattice node, ties upward; reject outside world bounds."""
        x, z = self._coordinates(pose)
        return (min(floor(x+.5), self.shape[0]-1),
                min(floor(z+.5), self.shape[1]-1))

    def _coordinates(self, pose: Pose) -> tuple[float, float]:
        check_pose(pose)
        xmin, zmin, xmax, zmax = self.bounds
        if not (xmin <= pose.x <= xmax and zmin <= pose.z <= zmax):
            raise ValueError("pose outside grid bounds")
        return ((pose.x-xmin)/self.resolution, (pose.z-zmin)/self.resolution)

    def endpoint_candidates(self, pose: Pose) -> tuple[Index, ...]:
        """Up to four surrounding nodes; callers must validate connectors."""
        x, z = self._coordinates(pose)
        return tuple(sorted({(min(i,self.shape[0]-1), min(j,self.shape[1]-1))
            for i in (floor(x), ceil(x)) for j in (floor(z), ceil(z))}))

    def is_free(self, index: Index) -> bool:
        """Whether a valid node satisfies the configured clearance."""
        self.index_to_pose(index)
        return not self.occupied[index[0]][index[1]]

    def neighbors(self, index: Index, connectivity: int = 8) -> tuple[Index, ...]:
        """Free 4/8-connected nodes; diagonals cannot cut occupied corners."""
        self.index_to_pose(index)
        if isinstance(connectivity, bool) or connectivity not in (4, 8):
            raise ValueError("connectivity must be 4 or 8")
        result = []
        i, j = index
        for di, dj in ((-1,0),(0,-1),(0,1),(1,0),(-1,-1),(-1,1),(1,-1),(1,1)):
            if connectivity == 4 and di and dj:
                continue
            node = (i+di,j+dj)
            if not (0 <= node[0] < self.shape[0] and 0 <= node[1] < self.shape[1]):
                continue
            if not self.is_free(node):
                continue
            if di and dj and (not self.is_free((i+di,j)) or not self.is_free((i,j+dj))):
                continue
            result.append(node)
        return tuple(result)
