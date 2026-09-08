# Control v0.5 Validation

**Status:** Completed baseline validation  
**Configuration:** `cascaded_pid_3dof_baseline.yaml`  
**Physics model:** `uuv_3dof_synthetic_v1`

## Scope

This record verifies the software contract and deterministic closed-loop
behaviour of the synthetic Control v0.5 baseline. It is not a real-vehicle
performance claim, a formal nonlinear stability proof, or a controller-tuning
study.

## Unit and contract validation

The Controller tests cover angle wrapping, parameter validation, zero-error
output, control signs, pitch-rate damping, inactive output fields, saturation,
conditional anti-windup, integral limits, reset behaviour, transactional
memory on invalid calls, input immutability, deterministic execution, and the
forward/reverse depth-coupling convention.

Controller runtime code imports only Core and NumPy. YAML configuration is
loaded at the Simulation boundary.

## Closed-loop integration case

The deterministic demo starts at `(x,z)=(5,-15)` m and tracks the stationary
reference `(15,-5)` m using `dt=0.05` s for 1200 steps (60 s). It runs once
without current and once with constant world current `[0.2,0.0]` m/s.

| Case | Initial position error | Final position error | Final pitch | Maximum force/moment |
| --- | ---: | ---: | ---: | ---: |
| No current | 14.142 m | 1.143 m | 0.160 rad | 20.000 N / 8.000 N m |
| Constant current | 14.142 m | 1.144 m | 0.154 rad | 20.000 N / 8.000 N m |

Both cases reduce position error below 15% of its initial value. Every command
is finite, respects the configured limits, has exact-zero inactive components,
and is accepted by `Python3DOFBackend`. Repeated runs produce identical output.

Run the evidence locally with:

```bash
python -m simulation.control_tracking_demo
python -m pytest tests/controller tests/integration/test_control_physics_integration.py -q
```

## Interpretation and limitations

The initial saturation is expected for this deliberately simple reference
step. The result demonstrates interface compatibility, correct response signs,
bounded execution, disturbance tolerance for the documented constant-current
case, and substantial error reduction. It does not establish optimal transient
response, robustness outside the tested envelope, actuator feasibility, or
physical-vehicle accuracy.
