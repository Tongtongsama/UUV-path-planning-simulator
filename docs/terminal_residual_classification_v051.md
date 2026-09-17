# Formal-candidate low-speed detour terminal classification

Date: 2026-09-15. Diagnosis only; no terminal behavior changes.

Use the fixed v0.5.1 coefficient-6 candidate, original detour geometry, original
pitch=0/rest initial state, constant 0.2 or 0.3 m/s timing and the original 120 s
budget. Execution clearance remains 0.3 m. These fixtures have only approximately
27 mm and 8 mm measured obstacle slack: neither is a robust safe configuration.

## Aligned observations

Along/cross errors below are actual minus original reference projected into the
last segment's tangent/normal frame. Forward error is reference minus actual
projected on the **actual body heading**, a different projection. At final time
no extra controller call is performed: targets/integrals/control are taken from
the last completed control interval at 119.95 s, explicitly separate from final
state at 120 s. This avoids manufacturing final-time control evidence.

| Quantity | 0.2 m/s | 0.3 m/s |
| --- | ---: | ---: |
| Reference end | 47.16991 s | 31.44660 s |
| First recorded state at/after end | 47.20 s | 31.45 s |
| Along error then | +0.04153 m | +0.09903 m |
| Cross error then | −0.34946 m | −0.52839 m |
| Actual surge then | +0.19760 m/s | +0.28939 m/s |
| Surge target then | −0.00684 m/s | −0.01815 m/s |
| Applied surge then | −2.21276 N | −3.24538 N |
| Final along error | −0.01432 m | −0.03041 m |
| Final cross error | −0.28680 m | −0.41385 m |
| Final forward projection error | −5.29e−5 m | −1.81e−5 m |
| Final u | −6.56e−6 m/s | −2.24e−6 m/s |
| Final w | −1.45e−9 m/s | −6.79e−11 m/s |
| Final q | −4.30e−6 rad/s | −9.82e−7 rad/s |
| Last control surge target | −1.86e−5 m/s | −6.38e−6 m/s |
| Last effective pitch target | 0.50844 rad | 0.48519 rad |
| Last depth correction | −0.05016 rad | −0.07341 rad |
| Last pitch integral | 0.001952 | 0.000408 |
| Last FF moment | 2.92133 N m | 2.79833 N m |
| Last applied pitch moment | 2.92132 N m | 2.79833 N m |
| Post-end net displacement | 0.08394 m | 0.17284 m |
| Post-end travel distance | 0.46992 m | 0.72863 m |

## 1. The model can actively brake

At reference end u is positive and applied tau_x negative. The recorded integral
of `min(0, tau_x*u)*dt` after reference end is approximately −0.400 and −0.858 J.
This is the surge-axis actuator mechanical-work component, not total vehicle
energy or a sum mixing force and moment units. It demonstrates active negative
surge work in addition to passive damping. The selector permits negative surge
force; this capability belongs to the synthetic force-actuated backend, not to
an independently validated physical thruster.

Both cases do exhibit finite braking travel/overshoot. However they eventually
come almost to rest; lack of braking authority alone is not the explanation for
the final persistent position error.

## 2. Dominant final classification: near-static non-goal position residual

Last-10-second maximum body speeds are about 1.70e−5 and 5.87e−6 m/s. Final
forward projection is almost zero while the final-segment cross error is still
large. The old outer loop therefore requests almost zero surge even though the
2-D goal tolerance is not satisfied. With no direct heave force, the existing
terminal hold does not remove the remaining normal-direction displacement.

Effective pitch target includes a negative depth correction because surge target
is slightly negative. That direction is stable in these terminal intervals:
recorded surge-target sign changes after reference end are zero, both raw and
after excluding magnitudes <=1e−4 m/s. This is not evidence of frequent terminal
switching chatter. It does not rule out chatter in different initial conditions.

The small final pitch integrals and FF approximately matching applied moment
also show that the old uncompensated-restoring error is not the remaining main
mechanism. The residual is already substantial at reference end, originating
partly in preceding segment/turn tracking; it is not generated solely by braking.

## Next independent change

Prioritize a constrained terminal position-capture design that can address
normal-direction residual after sufficient braking, with explicit speed,
heading, clearance and finite-time-budget conditions. Do not restore the rejected
instantaneous turn-to-goal policy, assume arbitrary zero-speed station keeping,
or just extend timeout.

Synchronized final-segment deceleration remains a useful separate experiment
for reducing braking travel, but reducing speed alone cannot be claimed to
remove the measured near-static projection blind spot. Any future speed profile
must update reference position timing consistently. No corner slowdown or path
smoothing was added in this change.

Evidence: `artifacts/controller/v051_20260915/detour_0.2.json`,
`detour_0.3.json`, and `terminal_diagnosis.png/.svg` in that directory. The plot
uses the last segment frame over the whole timeline; before the final segment
these projections are not nearest-path cross-track distances.
