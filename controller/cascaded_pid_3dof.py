"""Underactuated surge/pitch cascaded PID controller for Control v0.5."""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

from controller._validation import finite_scalar, validate_state
from controller.angles import wrap_to_pi
from controller.parameters import CascadedPID3DOFParameters
from controller.limits import ControlLimits3DOF
from controller.guidance import BodyGuidance3DOF
from core import ControlInput, VehicleState


@dataclass(frozen=True)
class ControllerMemory3DOF:
    surge_error_integral: float = 0.0
    pitch_error_integral: float = 0.0


@dataclass(frozen=True)
class ControllerDiagnostics3DOF:
    """One successful step: combined raw requests versus public clipped output."""
    timestamp: float
    effective_limits: ControlLimits3DOF
    forward_error: float
    depth_error: float
    surge_target: float
    travel_direction: float
    depth_correction: float
    pitch_target: float
    pitch_error: float
    surge_pd: float
    pitch_pd: float
    restoring_feedforward: float
    integral_before: ControllerMemory3DOF
    integral_after: ControllerMemory3DOF
    requested: ControlInput
    output: ControlInput
    guidance_mode: str = 'trajectory'


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
        self._last_diagnostics = None

    @property
    def parameters(self) -> CascadedPID3DOFParameters:
        return self._parameters


    @property
    def memory(self) -> ControllerMemory3DOF:
        return ControllerMemory3DOF(
            surge_error_integral=self._surge_error_integral,
            pitch_error_integral=self._pitch_error_integral,
        )

    @property
    def last_diagnostics(self) -> ControllerDiagnostics3DOF | None:
        """Immutable last successful step, cleared on reset or a failed attempt."""
        return self._last_diagnostics

    def reset(self) -> None:
        self._surge_error_integral = 0.0
        self._pitch_error_integral = 0.0
        self._last_diagnostics = None

    def step(
        self,
        current: VehicleState,
        reference: VehicleState,
        dt: float,
    ) -> ControlInput:
        """Legacy interface using configured limits; feedforward defaults to zero."""
        return self.step_with_limits(current,reference,dt,ControlLimits3DOF(
            self._parameters.maximum_tau_x,self._parameters.maximum_tau_m))

    def step_with_limits(self, current: VehicleState, reference: VehicleState,
                         dt: float, limits: ControlLimits3DOF) -> ControlInput:
        """Combine feedback/FF before clipping and conditional integration."""
        return self._step(current,reference,dt,limits,None)

    def step_guidance(self,current: VehicleState,target: BodyGuidance3DOF,
                      dt: float,limits: ControlLimits3DOF) -> ControlInput:
        """Direct inner-loop target; outer position/depth corrections are disabled."""
        if not isinstance(target,BodyGuidance3DOF):
            self._last_diagnostics=None
            raise TypeError('target must be BodyGuidance3DOF')
        return self._step(current,None,dt,limits,target)

    def _step(self,current,reference,dt,limits,target):
        self._last_diagnostics = None
        if not isinstance(limits,ControlLimits3DOF):
            raise TypeError('limits must be ControlLimits3DOF')
        validate_state(current, name="current")
        if target is None:
            validate_state(reference, name="reference")
        step_size = finite_scalar(dt, name="dt")
        if step_size <= 0.0:
            raise ValueError("dt must be strictly positive")

        p = self._parameters
        effective=ControlLimits3DOF(min(p.maximum_tau_x,limits.maximum_tau_x),
                                   min(p.maximum_tau_m,limits.maximum_tau_m))
        before=self.memory
        east_error = reference.pose.x - current.pose.x if target is None else 0.0
        depth_error = reference.pose.z - current.pose.z if target is None else 0.0
        forward_error = (
            math.cos(current.pose.pitch) * east_error
            + math.sin(current.pose.pitch) * depth_error
        )
        surge_reference = float(
            np.clip(
                reference.twist.u + p.position_to_surge_gain * forward_error if target is None else target.u,
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
        pitch_reference = reference.pose.pitch + pitch_correction if target is None else target.pitch
        pitch_error = wrap_to_pi(pitch_reference - current.pose.pitch)
        pitch_rate_error = (reference.twist.q if target is None else target.q) - current.twist.q

        surge_pd = p.surge_kp * surge_error
        next_surge_integral = _conditional_integral(
            self._surge_error_integral,
            surge_error,
            step_size,
            p.surge_integral_limit,
            surge_pd,
            p.surge_ki,
            effective.maximum_tau_x,
        )
        pitch_pd = p.pitch_kp * pitch_error + p.pitch_rate_kd * pitch_rate_error
        feedforward=p.restoring_pitch_coefficient*math.sin(current.pose.pitch)
        next_pitch_integral = _conditional_integral(
            self._pitch_error_integral,
            pitch_error,
            step_size,
            p.pitch_integral_limit,
            pitch_pd+feedforward,
            p.pitch_ki,
            effective.maximum_tau_m,
        )

        tau_x = float(
            np.clip(
                surge_pd + p.surge_ki * next_surge_integral,
                -effective.maximum_tau_x,
                effective.maximum_tau_x,
            )
        )
        tau_m = float(
            np.clip(
                (pitch_pd+feedforward) + p.pitch_ki * next_pitch_integral,
                -effective.maximum_tau_m,
                effective.maximum_tau_m,
            )
        )
        requested=ControlInput(tau_x=surge_pd+p.surge_ki*next_surge_integral,
                               tau_m=(pitch_pd+feedforward)+p.pitch_ki*next_pitch_integral)
        if not np.all(np.isfinite(requested.to_numpy())) or not np.isfinite(tau_x) or not np.isfinite(tau_m):
            raise ValueError("controller output must be finite")

        self._surge_error_integral = next_surge_integral
        self._pitch_error_integral = next_pitch_integral
        output=ControlInput(tau_x=tau_x,tau_m=tau_m)
        self._last_diagnostics=ControllerDiagnostics3DOF(current.timestamp,effective,
            forward_error,depth_error,surge_reference,travel_direction,pitch_correction,pitch_reference,
            pitch_error,surge_pd,pitch_pd,feedforward,before,self.memory,requested,output,
            'trajectory' if target is None else 'direct_body_guidance')
        return output
