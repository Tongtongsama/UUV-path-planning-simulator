# Guidance source audit

Audit date: 2026-09-18. Scope: the eight requested source files and the two repository
licenses. All eight files were read in full from commit-pinned raw URLs. No MATLAB,
ROS 1 package, numerical upstream example or hardware was run. No source code copied
or adapted; reuse category is reference implementation / mathematical design reference.

MSS commit: `98970f71a21cfe81e7e29abdcc1bb6741789cddc`.
UUV Simulator commit: `bfb40cb153684a0703173117b6bbf4258e8e71c5`.
Evidence location: immutable file links below and this audit; no local upstream checkout.
These pins identify the inspected content, not a claim that they are current upstream HEAD.

## MSS — principal algorithm reference

[MIT license](https://github.com/cybergalactic/MSS/blob/98970f71a21cfe81e7e29abdcc1bb6741789cddc/LICENSE),
copyright Thor I. Fossen. Potential adaptations require preserved applicable notices,
explicit local attribution, modifications and independent tests; none have occurred here.

| File / inspected function | Verified behavior | Project decision / limitation |
| --- | --- | --- |
| [LOSchi.m](https://github.com/cybergalactic/MSS/blob/98970f71a21cfe81e7e29abdcc1bb6741789cddc/GNC/LOSchi.m), LOSchi | Tangent-frame error and LOS correction; switching by remaining along-track distance; persistent active waypoint; terminal bearing extends to a point at distance 1e10 | Reference LOS geometry; replace persistent state with explicit per-mission progress; finite endpoint belongs to Navigation. Despite the comment calling it adaptive, the inspected implementation contains no adaptive estimate. |
| [crosstrackWpt3D.m](https://github.com/cybergalactic/MSS/blob/98970f71a21cfe81e7e29abdcc1bb6741789cddc/GNC/crosstrackWpt3D.m), crosstrackWpt3D | NED path-frame transform using azimuth/elevation; unbounded along-track coordinate | Not a finite-segment projector. The printed R_z has negative sine in both off-diagonal entries: generally not orthogonal. Independently derive ENU planar geometry; do not port this matrix. |
| [projectToPath.m](https://github.com/cybergalactic/MSS/blob/98970f71a21cfe81e7e29abdcc1bb6741789cddc/GNC/projectToPath.m), projectToPath | One-segment backward search; local forward candidates; clipped finite projection mu; interpolated path parameter/tangent; closest candidate wins, ties retain first | Main local-projection reference. Code computes sMax=ps(k_old)+dsFwd: window uses path-parameter units, despite sample-count wording. Can search at least the next segment. No monotonic-s guarantee or reachability check. Self-near branches inside the window can still attract projection. |
| [ALOS3D.m](https://github.com/cybergalactic/MSS/blob/98970f71a21cfe81e7e29abdcc1bb6741789cddc/GNC/ALOS3D.m), ALOS3D and local proj | NED yaw/pitch commands, two adaptive states and projection-limited update; Euclidean waypoint capture; final horizontal bearing extension | Future 3D research only. No adaptive state in v1. Does not justify importing NED pitch signs into our ENU/SNAME reduced state. |
| [LOSobserver.m](https://github.com/cybergalactic/MSS/blob/98970f71a21cfe81e7e29abdcc1bb6741789cddc/GNC/LOSobserver.m), LOSobserver | Wrapped angle innovation; propagated angle and rate estimate; differentiator constant derived from gain | Later independent factor. Actual signature has five inputs; LOSchi/ALOS3D comments show an older six-argument call. An observer is not an explicit angular-rate bound. |

LOSchi parameter prose is stricter than its checks: zero Delta/R_switch and equality
with minimum segment length are not rejected by all corresponding comparisons.
Our contracts must validate strictly positive finite values explicitly.

## UUV Simulator — interface reference

[Apache-2.0 license](https://github.com/uuvsimulator/uuv_simulator/blob/bfb40cb153684a0703173117b6bbf4258e8e71c5/LICENSE).
The inspected files carry UUV Simulator Authors copyright notices. No local import,
runtime dependency or adopted parameter; source/NOTICE obligations must be reviewed
again if future implementation copies code.

Base path: `uuv_control/uuv_trajectory_control/src/`.

| File / inspected members | Verified behavior | Project decision / limitation |
| --- | --- | --- |
| [uuv_waypoints/waypoint.py](https://github.com/uuvsimulator/uuv_simulator/blob/bfb40cb153684a0703173117b6bbf4258e8e71c5/uuv_control/uuv_trajectory_control/src/uuv_waypoints/waypoint.py), Waypoint constructor/properties, from_message, to_message, dist, calculate_heading | Position, frame, speed field, heading offset, acceptance radius; ROS message conversion | Reference metadata separation. ROS conversion belongs in our adapter, not Core. Equality checks position only; speed setters do not enforce finite positive bounds. |
| [path_generator.py](https://github.com/uuvsimulator/uuv_simulator/blob/bfb40cb153684a0703173117b6bbf4258e8e71c5/uuv_control/uuv_trajectory_control/src/uuv_trajectory_generator/path_generator/path_generator.py), PathGenerator init/reset, factories, get_segment_idx, init_waypoints, is_finished, has_started, _compute_rot_quat, abstract generate methods | Separate interpolation operations, normalized parameter, duration/start time, orientation construction | Reference contracts only. is_finished/has_started use strict time comparisons; non-time branches simply return True. They do not implement spatial mission completion. Quaternion pitch sign depends on its frame, not our scalar pitch. |
| [linear_interpolator.py](https://github.com/uuvsimulator/uuv_simulator/blob/bfb40cb153684a0703173117b6bbf4258e8e71c5/uuv_control/uuv_trajectory_control/src/uuv_trajectory_generator/path_generator/linear_interpolator.py), LinearInterpolator init_interpolator, get_samples, generate_pos, generate_pnt, generate_quat | Piecewise-line positions; normalized cumulative length; total duration from mean waypoint max_forward_speed; finite-difference direction; heading-offset spline | A speed metadata field does not ensure each waypoint's limit is enforced: this code uses a mean. get_samples sets t=0. generate_pnt sets position/time/orientation, not a complete audited velocity/acceleration profile. Heading spline uses k=3 although only >=2 waypoints are checked. |

trajectory_point.py and wp_trajectory_generator.py were intentionally outside the
eight-file scope. Their full derivative and orchestration semantics remain unaudited.
We cannot attribute a complete acceleration/velocity contract to these three files.

## Supplementary sources and decisions

Python Vehicle Simulator commit proposed for the next audit:
`695c673ea6aed3d3e3e09889a02241dc54043abd`. Its guidance.py/refModel3 and autopilot
have not been inspected in this eight-file audit. Treat it as supplementary control/
reference-model research, not the principal LOS source. The existing Physics audit
at a different pinned PVS commit remains valid for its original scope.

Nav2 Regulated Pure Pursuit and MOOS-IvP are later references; no source claims or
adoption decisions are added here. Stonefish remains outside this milestone.

## Design consequences

1. Follow current `physics/frames.py`: xdot=u cos(theta)+w sin(theta),
   zdot=u sin(theta)-w cos(theta), positive pitch nose-up. Target pitch is atan2(dz,dx).
2. Keep finite projected progress, committed progress and active segment distinct.
3. A finite forward window reduces distant jumps; segment gates must still prevent
   shortcuts across nearby U/zigzag branches.
4. Raw LOS can jump at switches; continuity requires an explicit later observer or
   rate policy. q_ref=0 in v1 is an inner-loop convention, not the derivative of a jump.
5. Arc-length lookahead across a corner is a project variant, not identical to
   LOSchi on a supporting line. A visible target is not a swept-motion safety proof.

See [progress LOS specification](guidance_progress_los_specification.md) for our
internal mathematical and integration contract; formulas are defined there once.
