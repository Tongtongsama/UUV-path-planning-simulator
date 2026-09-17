"""Executable design prototype only; production baseline is not modified.

The baseline equations are repeated here deliberately for an independent,
opt-in rehearsal of pre-limit feedforward and consistent conditional integration.
Zero-FF/configured-limit equivalence is regression tested.
"""
import math
import numpy as np
from core import ControlInput
from controller import CascadedPID3DOFController
from controller.cascaded_pid_3dof import _conditional_integral
from controller._validation import finite_scalar, validate_state
from controller.angles import wrap_to_pi


class FeedforwardPIDPrototype(CascadedPID3DOFController):
    """Known effective caps are supplied by the experiment, not queried from Physics."""
    def __init__(self, parameters, restoring_coefficient=0., applied_limits=None):
        super().__init__(parameters)
        self.coefficient=finite_scalar(restoring_coefficient,name='restoring_coefficient')
        if self.coefficient < 0:
            raise ValueError('restoring_coefficient must be nonnegative')
        limits=applied_limits or (parameters.maximum_tau_x,parameters.maximum_tau_m)
        if len(limits)!=2:
            raise ValueError('two applied limits required')
        limits=tuple(finite_scalar(x,name='applied_limit') for x in limits)
        if any(x<=0 for x in limits):
            raise ValueError('applied limits must be positive')
        self.limits=(min(limits[0],parameters.maximum_tau_x),min(limits[1],parameters.maximum_tau_m))
        self.records=[]

    def reset(self):
        super().reset()
        self.records.clear()

    def step_with_limits(self,current,reference,dt,limits):
        """Keep this historical prototype's own step under capability dispatch."""
        self.limits=(min(self.parameters.maximum_tau_x,limits.maximum_tau_x),
                     min(self.parameters.maximum_tau_m,limits.maximum_tau_m))
        return self.step(current,reference,dt)

    def step(self,current,reference,dt):
        validate_state(current,name='current')
        validate_state(reference,name='reference')
        dt=finite_scalar(dt,name='dt')
        if dt<=0:
            raise ValueError('dt must be positive')
        p=self.parameters
        dx,dz=reference.pose.x-current.pose.x,reference.pose.z-current.pose.z
        forward=math.cos(current.pose.pitch)*dx+math.sin(current.pose.pitch)*dz
        ur=float(np.clip(reference.twist.u+p.position_to_surge_gain*forward,-p.maximum_surge_reference,p.maximum_surge_reference))
        eu=ur-current.twist.u
        correction=float(np.clip((1 if ur>=0 else -1)*p.depth_to_pitch_gain*dz,-p.maximum_pitch_correction,p.maximum_pitch_correction))
        ep=wrap_to_pi(reference.pose.pitch+correction-current.pose.pitch)
        eq=reference.twist.q-current.twist.q
        ff=self.coefficient*math.sin(current.pose.pitch)
        up=p.surge_kp*eu
        pp=p.pitch_kp*ep+p.pitch_rate_kd*eq
        old_u,old_p=self._surge_error_integral,self._pitch_error_integral
        next_u=_conditional_integral(old_u,eu,dt,p.surge_integral_limit,up,p.surge_ki,self.limits[0])
        # The SAME ff+PD sum and effective limit determine both integration and output.
        next_p=_conditional_integral(old_p,ep,dt,p.pitch_integral_limit,pp+ff,p.pitch_ki,self.limits[1])
        raw_u=up+p.surge_ki*next_u
        raw_p=(pp+ff)+p.pitch_ki*next_p
        if not np.all(np.isfinite([raw_u,raw_p])):
            raise ValueError('nonfinite combined output')
        command=ControlInput(tau_x=float(np.clip(raw_u,-self.limits[0],self.limits[0])),
                             tau_m=float(np.clip(raw_p,-self.limits[1],self.limits[1])))
        self._surge_error_integral=next_u
        self._pitch_error_integral=next_p
        self.records.append(dict(ff=ff,pitch_pd=pp,pitch_error=ep,pitch_target=reference.pose.pitch+correction,
            old_pitch_integral=old_p,pitch_integral=next_p,
            raw_tau_x=raw_u,raw_tau_m=raw_p,applied_tau_x=command.tau_x,applied_tau_m=command.tau_m,
            effective_tau_x_limit=self.limits[0],effective_tau_m_limit=self.limits[1],
            pitch_integral_frozen=next_p==old_p and ep!=0,
            saturated_pitch=abs(raw_p)>self.limits[1]))
        return command
