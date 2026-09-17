"""Mission configuration, terminal outcomes and synchronized evidence."""
from dataclasses import dataclass
from enum import Enum, auto
from math import isclose, isfinite
from core import ControlInput, Pose, Trajectory, VehicleState
from environment import WorldModel, NoCurrent, ConstantCurrent
from planner import PlanningConstraints, PlanningRequest, PlanningResult
from trajectory.generator import finite
from controller import ControllerDiagnostics3DOF,BodyGuidance3DOF


class NavigationStatus(Enum):
    """Terminal outcome; safety violations take priority over goal arrival."""
    SUCCESS=auto()
    PLANNING_FAILED=auto()
    TRAJECTORY_FAILED=auto()
    COLLISION=auto()
    TIMEOUT=auto()
    OUT_OF_BOUNDS=auto()
    NUMERICAL_FAILURE=auto()
    CONTROLLER_FAILURE=auto()
    ENVIRONMENT_FAILURE=auto()
    CAPTURE_FAILED=auto()
    STARTUP_FAILED=auto()


@dataclass(frozen=True)
class NavigationRequest:
    """Initial physical state, geometric goal, static world and footprint."""
    initial_state: VehicleState
    goal: Pose
    environment: WorldModel
    planning_constraints: PlanningConstraints

    def __post_init__(self) -> None:
        PlanningRequest(self.initial_state,self.goal,self.planning_constraints)
        if not isinstance(self.environment,WorldModel):
            raise TypeError("environment must be WorldModel")
        if not isinstance(self.environment.current,(NoCurrent,ConstantCurrent)):
            raise ValueError("v0.8 accepts only NoCurrent or ConstantCurrent")


@dataclass(frozen=True)
class NavigationConfig:
    """Fixed-step execution; defaults are synthetic experiment settings."""
    dt: float = .05
    max_duration: float = 120.0
    goal_position_tolerance: float = .25
    goal_speed_tolerance: float = .1
    goal_pitch_rate_tolerance: float = .1
    settle_time: float = .5
    maximum_tau_x: float = 20.0
    maximum_tau_m: float = 8.0

    def __post_init__(self) -> None:
        for name in self.__dataclass_fields__:
            v=finite(getattr(self,name),name)
            if v < 0 or (name in ("dt","max_duration","maximum_tau_x","maximum_tau_m") and v==0):
                raise ValueError(f"{name} outside range")
            object.__setattr__(self,name,v)
        ratio=self.max_duration/self.dt
        if not isfinite(ratio) or not isclose(ratio,round(ratio),rel_tol=0,abs_tol=1e-8) or ratio<1 or ratio>1000000:
            raise ValueError("max_duration must be an integer number of dt steps, at most 1000000")


@dataclass(frozen=True)
class NavigationStep:
    """One completed transition; control/reference/current apply at time_k."""
    time: float
    commanded: ControlInput
    applied: ControlInput
    current: tuple[float,float]
    position_error: float
    pitch_error: float
    velocity_error: float
    segment_clearance: float
    in_bounds: bool
    collision: bool
    clipped: bool
    at_limit: bool
    limits_aware: bool = False
    controller_diagnostics: ControllerDiagnostics3DOF | None = None
    capture_phase: str = 'DISABLED'
    effective_guidance: BodyGuidance3DOF | None = None
    startup_phase: str = 'DISABLED'
    reference_time: float | None = None


@dataclass(frozen=True)
class NavigationResult:
    """Finite history only; N transitions have N+1 states and references."""
    status: NavigationStatus
    planning_result: PlanningResult | None
    trajectory: Trajectory | None
    state_history: tuple[VehicleState,...]
    reference_history: tuple[VehicleState,...]
    control_history: tuple[NavigationStep,...]
    metrics: dict[str,float | int | bool | None]
    message: str | None = None
    failed_command: ControlInput | None = None
    failed_applied: ControlInput | None = None
    failed_current: tuple[float,float] | None = None

    @property
    def final_state(self) -> VehicleState:
        """Last finite, timestamp-valid state, including detected collision endpoint."""
        return self.state_history[-1]
