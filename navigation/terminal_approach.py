"""Bounded, opt-in zero-current approach experiment; not a feasibility oracle.

Rollouts copy the entire controller (including integral memory), physics and
progress. Live execution uses the same feedback kernel, never predicted controls.
The legacy PATH_FOLLOWING progress is frozen; approach owns a separate cursor.
"""
from copy import deepcopy
from dataclasses import asdict, dataclass
from math import hypot, isfinite
from time import perf_counter

import numpy as np
from controller import BodyGuidance3DOF, wrap_to_pi
from guidance import compute_los
from physics.backend import extract_reduced_state


@dataclass(frozen=True)
class TerminalApproachConfig:
    """Preregistered experimental limits; no physical feasibility guarantee."""
    evaluation_arc: float = 4.
    evaluation_interval: float = 2.
    max_evaluations: int = 16
    horizon: float = 30.
    speeds: tuple = (.15, .10, .05)
    brake_distances: tuple = (.35, .20)
    entry_speed: float = .03
    entry_rate: float = .03
    entry_pitch_error: float = .15
    hold_steps: int = 3
    prediction_margin: float = .005
    state_error_limit: float = 1e-6

    def __post_init__(self):
        for name in self.__dataclass_fields__:
            value = getattr(self, name)
            if name in ('speeds', 'brake_distances'):
                value = tuple(value)
                if not value: raise ValueError('empty candidates')
                object.__setattr__(self, name, value)
            else: value = (value,)
            if any(isinstance(v, bool) or not isinstance(v, (int, float)) or not isfinite(v) or v <= 0 for v in value):
                raise ValueError('positive finite configuration required: '+name)
        for name in ('max_evaluations', 'hold_steps'):
            if not isinstance(getattr(self, name), int): raise ValueError('integer required: '+name)


@dataclass
class ApproachCursor:
    """Private approach progress and irreversible brake subphase."""
    progress: object
    pitch: float | None
    braking: bool = False
    held: int = 0


class TerminalApproachPolicy:
    """Finite candidate prediction with actual-state guarded handoff."""
    def __init__(self, config=TerminalApproachConfig()):
        if not isinstance(config, TerminalApproachConfig): raise TypeError('invalid approach config')
        self.config = config
        self.reset()

    def reset(self):
        self.events = []
        self.evaluations = 0
        self.last_evaluation = None
        self.active = False
        self.cursor = None
        self.predicted = []
        self.index = 0
        self.handoff = None
        self.prediction_steps = 0
        self.wall_seconds = 0.

    def kernel(self, state, cursor, adapter, goal, capture, speed, brake_distance):
        """Identical pure-state feedback in rollout and execution, no PID calls."""
        command = compute_los(state.pose, cursor.progress, adapter.projection_config,
                              adapter.los_config, cursor.pitch)
        cursor.progress = command.projection.next_state
        cursor.pitch = command.pitch_ref
        terminal = adapter.terminal_reference(cursor.progress, state.timestamp, state.pose.pitch)
        c = self.config
        distance = hypot(goal.x-state.pose.x, goal.z-state.pose.z)
        g = cursor.progress.geometry
        on_final = cursor.progress.segment_index == len(g.lengths)-1
        if on_final and distance <= brake_distance: cursor.braking = True
        admissible = (cursor.braking and command.terminal_ready
            and hypot(state.twist.u, state.twist.w) <= c.entry_speed
            and abs(state.twist.q) <= min(c.entry_rate, capture.maximum_actual_pitch_rate)
            and abs(state.pose.pitch) <= capture.maximum_actual_pitch
            and abs(terminal.pose.pitch) <= capture.maximum_pitch
            and abs(wrap_to_pi(state.pose.pitch-terminal.pose.pitch)) <= c.entry_pitch_error)
        cursor.held = cursor.held+1 if admissible else 0
        if cursor.braking:
            u = -max(-capture.brake_reverse_target, min(capture.brake_reverse_target, state.twist.u))
            target = BodyGuidance3DOF(u, terminal.pose.pitch, 0.)
        else:
            target = BodyGuidance3DOF(speed, command.pitch_ref, 0.)
        return target, cursor.held >= c.hold_steps

    def update(self, state, progress, previous_pitch, adapter, controller, physics,
               world, space, required, goal, capture, limits, dt, remaining_time):
        """Return (target, actual_handoff, failure); UNKNOWN is not infeasible."""
        c = self.config
        if np.any(np.asarray(world.current_at(state.pose, state.timestamp)) != 0):
            return None, False, 'APPROACH_OUT_OF_DOMAIN_CURRENT'
        if self.active:
            if self.index >= len(self.predicted):
                return None, False, 'APPROACH_PREDICTION_WINDOW_EXHAUSTED'
            expected = self.predicted[self.index]
            error = float(np.max(np.abs(extract_reduced_state(state)-extract_reduced_state(expected))))
            if error > c.state_error_limit:
                return None, False, 'APPROACH_PREDICTION_DEVIATION'
            target, ready = self.kernel(state, self.cursor, adapter, goal, capture, *self.action)
            if ready:
                self.handoff = dict(actual=asdict(state), predicted=asdict(expected), maximum_state_error=error,
                                    progress_s=self.cursor.progress.committed_s)
                return None, True, None
            self.index += 1
            return target, False, None
        remaining = progress.geometry.total_length-progress.committed_s
        if remaining > c.evaluation_arc: return None, False, None
        if self.evaluations >= c.max_evaluations or remaining <= 1e-12:
            return None, False, 'APPROACH_UNKNOWN_SEARCH_EXHAUSTED'
        if self.last_evaluation is not None and state.timestamp-self.last_evaluation < c.evaluation_interval-1e-9:
            return None, False, None
        self.last_evaluation = state.timestamp
        self.evaluations += 1
        event = dict(time=state.timestamp, state=asdict(state), memory=asdict(controller.memory),
                     remaining_arc=remaining, candidates=[], outcome='UNKNOWN')
        self.events.append(event)
        started = perf_counter()
        for speed in c.speeds:
            for brake_distance in c.brake_distances:
                cursor = ApproachCursor(deepcopy(progress), previous_pitch)
                shadow_controller, shadow_physics = deepcopy(controller), deepcopy(physics)
                predicted, x = [], state
                reason = 'UNKNOWN_HORIZON'
                for _ in range(int(min(c.horizon, remaining_time)/dt)):
                    predicted.append(x)
                    target, ready = self.kernel(x, cursor, adapter, goal, capture, speed, brake_distance)
                    if ready:
                        reason = 'ADMISSIBLE'
                        break
                    control = shadow_controller.step_guidance(x, target, dt, limits)
                    nxt = shadow_physics.step(x, control, world.current_at(x.pose, x.timestamp), dt)
                    self.prediction_steps += 1
                    if not space.is_segment_valid(x.pose, nxt.pose, required+c.prediction_margin):
                        reason = 'PREDICTED_UNSAFE'
                        break
                    # During braking do not cross beyond the final goal plane.
                    a, b = progress.geometry.points[-2:]
                    if cursor.braking and ((nxt.pose.x-goal.x)*(b.x-a.x)+(nxt.pose.z-goal.z)*(b.z-a.z)) > 0:
                        reason = 'PREDICTED_GOAL_PLANE_OVERSHOOT'
                        break
                    x = nxt
                event['candidates'].append(dict(speed=speed, brake_distance=brake_distance,
                    outcome=reason, steps=len(predicted), end=asdict(x)))
                if reason == 'ADMISSIBLE':
                    self.active = True
                    self.action = (speed, brake_distance)
                    self.cursor = ApproachCursor(deepcopy(progress), previous_pitch)
                    self.predicted = predicted
                    event['outcome'] = 'SELECTED'
                    event['predicted_handoff'] = asdict(x)
                    self.wall_seconds += perf_counter()-started
                    return self.update(state, progress, previous_pitch, adapter, controller, physics,
                        world, space, required, goal, capture, limits, dt, remaining_time)
        self.wall_seconds += perf_counter()-started
        return None, False, None
