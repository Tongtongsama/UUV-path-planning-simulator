"""Reuse baseline inner PID; back-calculate from the jointly allocated wrench.

Only this derived adapter owns/updates its inherited integrator memory. The
original controller implementation and all existing configurations are unchanged.
"""
import math
from dataclasses import asdict
from controller.cascaded_pid_3dof import CascadedPID3DOFController
from controller.parameters import CascadedPID3DOFParameters
from controller.guidance import BodyGuidance3DOF
from controller.limits import ControlLimits3DOF
from core import Pose, Twist, VehicleState
from ros2.twin_axial_v2.allocation import allocate


def parameters():
    # Dedicated fixed step-test candidate; not a change to baseline YAML.
    # Large internal bounds disable the baseline's independent wrench clipping;
    # physical limits are enforced exclusively by the original twin allocator.
    return CascadedPID3DOFParameters(
        position_to_surge_gain=0, depth_to_pitch_gain=0,
        maximum_surge_reference=.6, maximum_pitch_correction=math.pi/3,
        surge_kp=25, surge_ki=3, pitch_kp=20, pitch_ki=1, pitch_rate_kd=22,
        maximum_tau_x=1e6, maximum_tau_m=1e6,
        surge_integral_limit=4, pitch_integral_limit=1,
        restoring_pitch_coefficient=6)


class AllocatedPID(CascadedPID3DOFController):
    tracking_time_s=1.0

    def __init__(self):
        super().__init__(parameters())

    def command(self, record, target, dt=.05):
        if not record.get('valid') or not 0 < dt <= .1:
            self.reset()
            raise ValueError('Invalid feedback or controller timestep')
        if not 0 <= target.u <= .6 or abs(target.pitch)>math.pi/3:
            self.reset()
            raise ValueError('Target outside step-test envelope')
        s=record['solver_state_ned_frd']
        current=VehicleState(Pose(x=s['y'],z=-s['z'],pitch=record['pitch_nose_up_rad']),
                             Twist(u=s['u'],w=s['w'],q=s['q']),s['t'])
        before=asdict(self.memory)
        self.step_guidance(current,target,dt,ControlLimits3DOF(1e6,1e6))
        d=self.last_diagnostics
        raw={'X':d.requested.tau_x,'M':d.requested.tau_m}
        allocation=allocate(raw)
        allocated=allocation['allocated_wrench']
        candidate=asdict(self.memory)
        p=self.parameters
        corrections={'surge_error_integral':dt*(allocated['X']-raw['X'])/(p.surge_ki*self.tracking_time_s),
                     'pitch_error_integral':dt*(allocated['M']-raw['M'])/(p.pitch_ki*self.tracking_time_s)}
        self._surge_error_integral=max(-p.surge_integral_limit,min(p.surge_integral_limit,
            self._surge_error_integral+corrections['surge_error_integral']))
        self._pitch_error_integral=max(-p.pitch_integral_limit,min(p.pitch_integral_limit,
            self._pitch_error_integral+corrections['pitch_error_integral']))
        # Return raw wrench: the gateway independently repeats the same allocation.
        # The recorded transaction must confirm the actual applied thrusters.
        return raw,{'target':asdict(target),'dt':dt,'feedback_sequence':record['sequence'],
            'feedback_time':s['t'],'feedback_u':s['u'],'feedback_pitch':record['pitch_nose_up_rad'],
            'requested_wrench':raw,'allocation':allocation,'integral_before':before,
            'integral_candidate':candidate,'backcalculation':corrections,
            'integral_after':asdict(self.memory),'tracking_time_s':self.tracking_time_s}
