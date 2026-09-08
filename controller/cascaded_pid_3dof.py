"""Underactuated surge/pitch cascaded PID controller for Control v0.5."""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

from controller._validation import finite_scalar, validate_state
from controller.angles import wrap_to_pi
from controller.parameters import CascadedPID3DOFParameters
from core import ControlInput, VehicleState


@dataclass(frozen=True)
class ControllerMemory3DOF:
    surge_error_integral: float = 0.0
    pitch_error_integral: float = 0.0


def _conditional_integral(
    previous: float,
    error: float,
    dt: float,
    integral_limit: float,
    proportional_and_derivative: float,
    integral_gain: float,
    output_limit: float,
) -> float:
    candidate = float(np.clip(previous + error * dt, -integral_limit, integral_limit))
    candidate_raw = proportional_and_derivative + integral_gain * candidate
    if abs(candidate_raw) <= output_limit:
        return candidate
    if candidate_raw > output_limit and error < 0.0:
        return candidate
    if candidate_raw < -output_limit and error > 0.0:
        return candidate
    return previous


class CascadedPID3DOFController:
    """Stateful baseline producing only admissible ``tau_x`` and ``tau_m``."""

    def __init__(self, parameters: CascadedPID3DOFParameters) -> None:
        if not isinstance(parameters, CascadedPID3DOFParameters):
            raise TypeError("parameters must be CascadedPID3DOFParameters")
        self._parameters = parameters
        self._surge_error_integral = 0.0
        self._pitch_error_integral = 0.0

    @property
    def parameters(self) -> CascadedPID3DOFParameters:
        return self._parameters


    @property
    def memory(self) -> ControllerMemory3DOF:
        return ControllerMemory3DOF(
            surge_error_integral=self._surge_error_integral,
            pitch_error_integral=self._pitch_error_integral,
        )

    def reset(self) -> None:
        self._surge_error_integral = 0.0
        self._pitch_error_integral = 0.0

    def step(
        self,
        current: VehicleState,
        reference: VehicleState,
        dt: float,
    ) -> ControlInput:
        validate_state(current, name="current")
        validate_state(reference, name="reference")
        step_size = finite_scalar(dt, name="dt")
        if step_size <= 0.0:
            raise ValueError("dt must be strictly positive")

        p = self._parameters
        east_error = reference.pose.x - current.pose.x
        depth_error = reference.pose.z - current.pose.z
        forward_error = (
            math.cos(current.pose.pitch) * east_error
            + math.sin(current.pose.pitch) * depth_error
        )
        surge_reference = float(
            np.clip(
                reference.twist.u + p.position_to_surge_gain * forward_error,
                -p.maximum_surge_reference,
                p.maximum_surge_reference,
            )
        )
        surge_error = surge_reference - current.twist.u

        travel_direction = 1.0 if surge_reference >= 0.0 else -1.0
        pitch_correction = float(
            np.clip(
                travel_direction * p.depth_to_pitch_gain * depth_error,
                -p.maximum_pitch_correction,
                p.maximum_pitch_correction,
            )
        )
        pitch_reference = reference.pose.pitch + pitch_correction
        pitch_error = wrap_to_pi(pitch_reference - current.pose.pitch)
        pitch_rate_error = reference.twist.q - current.twist.q

        surge_pd = p.surge_kp * surge_error
        next_surge_integral = _conditional_integral(
            self._surge_error_integral,
            surge_error,
            step_size,
            p.surge_integral_limit,
            surge_pd,
            p.surge_ki,
            p.maximum_tau_x,
        )
        pitch_pd = p.pitch_kp * pitch_error + p.pitch_rate_kd * pitch_rate_error
        next_pitch_integral = _conditional_integral(
            self._pitch_error_integral,
            pitch_error,
            step_size,
            p.pitch_integral_limit,
            pitch_pd,
            p.pitch_ki,
            p.maximum_tau_m,
        )

        tau_x = float(
            np.clip(
                surge_pd + p.surge_ki * next_surge_integral,
                -p.maximum_tau_x,
                p.maximum_tau_x,
            )
        )
        tau_m = float(
            np.clip(
                pitch_pd + p.pitch_ki * next_pitch_integral,
                -p.maximum_tau_m,
                p.maximum_tau_m,
            )
        )
        if not np.isfinite(tau_x) or not np.isfinite(tau_m):
            raise ValueError("controller output must be finite")

        self._surge_error_integral = next_surge_integral
        self._pitch_error_integral = next_pitch_integral
        return ControlInput(tau_x=tau_x, tau_m=tau_m)
