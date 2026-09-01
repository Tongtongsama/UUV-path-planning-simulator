"""Deterministic integration smoke scenario for the Core data model.

This module deliberately contains only mock/stub behaviour.  It validates the
interfaces that future planner, trajectory-generation, controller, physics,
and logger modules will use without implementing any of those subsystems.

Frame conventions made explicit by the scenario:
    * Pose is expressed in the ENU inertial/world frame (z points up).
    * Twist and ControlInput use the SNAME body-fixed frame (w points down).
    * The kinematic stub performs an explicitly labelled body-to-world mapping.
"""

from __future__ import annotations

from dataclasses import dataclass
from math import cos, sin

from core import ControlInput, Path, Pose, Trajectory, Twist, VehicleState


@dataclass(frozen=True)
class CoreIntegrationResult:
    """Immutable record returned by :func:`run_core_integration_demo`."""

    path: Path
    trajectory: Trajectory
    controls: tuple[ControlInput, ...]
    history: tuple[VehicleState, ...]


def mock_generate_trajectory(path: Path, time_step: float = 1.0) -> Trajectory:
    """Assign timestamps and simple body-frame reference velocities to a path."""
    if not isinstance(path, Path):
        raise TypeError(f"Expected Path, got {type(path).__name__}")
    if time_step <= 0.0:
        raise ValueError("time_step must be positive")

    states: list[VehicleState] = []
    for index, pose in enumerate(path):
        if index < len(path) - 1:
            next_pose = path[index + 1]
            east_rate = (next_pose.x - pose.x) / time_step
            north_rate = (next_pose.y - pose.y) / time_step

            # ENU horizontal velocity -> SNAME body velocity at the waypoint yaw.
            body_u = cos(pose.yaw) * east_rate + sin(pose.yaw) * north_rate
            body_v = -sin(pose.yaw) * east_rate + cos(pose.yaw) * north_rate

            # ENU z is up while SNAME body w is down, hence the minus sign.
            body_w = -(next_pose.z - pose.z) / time_step
            yaw_rate = (next_pose.yaw - pose.yaw) / time_step
            reference_twist = Twist(u=body_u, v=body_v, w=body_w, r=yaw_rate)
        else:
            reference_twist = Twist.zero()

        states.append(
            VehicleState(
                pose=pose,
                twist=reference_twist,
                timestamp=index * time_step,
            )
        )

    return Trajectory.from_list(states)


def mock_controller(
    current: VehicleState,
    reference: VehicleState,
    position_gain: float = 0.5,
    attitude_gain: float = 0.5,
) -> ControlInput:
    """Produce a complete body-frame generalized-force command.

    The proportional mapping is intentionally a mock controller.  It exists
    only to prove that a future controller can consume VehicleState objects and
    produce a six-component ControlInput without embedding actuator policy.
    """
    east_error = reference.pose.x - current.pose.x
    north_error = reference.pose.y - current.pose.y
    yaw = current.pose.yaw

    body_x_error = cos(yaw) * east_error + sin(yaw) * north_error
    body_y_error = -sin(yaw) * east_error + cos(yaw) * north_error
    body_z_down_error = -(reference.pose.z - current.pose.z)

    return ControlInput(
        tau_x=position_gain * body_x_error,
        tau_y=position_gain * body_y_error,
        tau_z=position_gain * body_z_down_error,
        tau_k=attitude_gain * (reference.pose.roll - current.pose.roll),
        tau_m=attitude_gain * (reference.pose.pitch - current.pose.pitch),
        tau_n=attitude_gain * (reference.pose.yaw - current.pose.yaw),
    )


def kinematic_physics_stub(
    current: VehicleState,
    control: ControlInput,
    time_step: float = 1.0,
) -> VehicleState:
    """Create the next immutable state using a deterministic unit-mass stub.

    This is not a hydrodynamic model.  Generalized force is treated as unit
    acceleration, Euler rates are integrated directly, and body linear
    velocity is rotated into ENU using yaw only.  The mapping is intentionally
    visible so it cannot be mistaken for the future Physics Engine.
    """
    if time_step <= 0.0:
        raise ValueError("time_step must be positive")

    next_twist = Twist(
        u=current.twist.u + control.tau_x * time_step,
        v=current.twist.v + control.tau_y * time_step,
        w=current.twist.w + control.tau_z * time_step,
        p=current.twist.p + control.tau_k * time_step,
        q=current.twist.q + control.tau_m * time_step,
        r=current.twist.r + control.tau_n * time_step,
    )

    yaw = current.pose.yaw
    east_rate = cos(yaw) * next_twist.u - sin(yaw) * next_twist.v
    north_rate = sin(yaw) * next_twist.u + cos(yaw) * next_twist.v

    next_pose = Pose(
        x=current.pose.x + east_rate * time_step,
        y=current.pose.y + north_rate * time_step,
        # Pose.z is ENU-up; Twist.w is SNAME body-down.
        z=current.pose.z - next_twist.w * time_step,
        roll=current.pose.roll + next_twist.p * time_step,
        pitch=current.pose.pitch + next_twist.q * time_step,
        yaw=current.pose.yaw + next_twist.r * time_step,
    )
    return VehicleState(
        pose=next_pose,
        twist=next_twist,
        timestamp=current.timestamp + time_step,
    )


def build_demo_path() -> Path:
    """Return the deterministic three-waypoint geometric reference."""
    return Path.from_list(
        [
            Pose.from_position_and_yaw(0.0, 0.0, 0.0, 0.0),
            Pose.from_position_and_yaw(2.0, 1.0, -0.5, 0.25),
            Pose.from_position_and_yaw(4.0, 2.0, -1.0, 0.50),
        ]
    )


def run_core_integration_demo(time_step: float = 1.0) -> CoreIntegrationResult:
    """Run the complete deterministic Core interface-validation pipeline."""
    path = build_demo_path()
    trajectory = mock_generate_trajectory(path, time_step=time_step)
    initial = VehicleState.zero(timestamp=trajectory.start_time)

    history = [initial]
    controls: list[ControlInput] = []
    current = initial
    for reference in trajectory.states[1:]:
        control = mock_controller(current, reference)
        current = kinematic_physics_stub(current, control, time_step=time_step)

        # Logger-style in-memory records retain immutable state snapshots.
        controls.append(control)
        history.append(current)

    return CoreIntegrationResult(
        path=path,
        trajectory=trajectory,
        controls=tuple(controls),
        history=tuple(history),
    )


def main() -> None:
    """Execute the scenario and print a compact human-readable summary."""
    result = run_core_integration_demo()
    print("Core integration demo completed")
    print(f"Path: {len(result.path)} waypoints, length={result.path.length():.3f} m")
    print(
        f"Trajectory: {len(result.trajectory)} states, "
        f"duration={result.trajectory.duration:.3f} s"
    )
    print(f"Controls: {len(result.controls)}")
    print(f"History snapshots: {len(result.history)}")
    print(f"Final state: {result.history[-1]}")


if __name__ == "__main__":
    main()

