from controller import CascadedPID3DOFController
from core import Pose, Twist, VehicleState
from simulation.controller_config import load_baseline_controller_parameters


def test_reverse_command_reverses_depth_to_pitch_correction():
    controller = CascadedPID3DOFController(load_baseline_controller_parameters())
    current = VehicleState(Pose(x=2.0, z=-2.0), Twist.zero(), 0.0)
    reference = VehicleState(Pose(x=0.0, z=0.0), Twist.zero(), 0.0)
    control = controller.step(current, reference, 0.1)
    assert control.tau_x < 0.0
    assert control.tau_m < 0.0
