# Simulator-only operational readiness review

Date: 2026-11-20. Scope: **simulator-only**; no sea trial, no HIL, no certification.

## Evidence

- Joint matrix: **18/18 passed** (F0 no-current control, F1 constant-current control, F2 v10 measured-current; each P0–P4 + P0 repeat).
- Repeat runs: **27/27 passed** (P0–P4 three runs per flow).
- Joint records total: **45**.
- Five real S4 pilots preserved: F0/F1/F2 P0/P2/P4.
- 18 S4 Python cases preserved: `all_18_original_passed`, `all_18_verified`.
- GUI demo: S4 scene + D path + 16 current reference balls; visualization-only.
- Offscreen camera recorder: service starts and writes MP4, but rendered frames are **black**; not accepted as video evidence.

## Joint matrix ranges

- Arrival: 60.45–61.35 s.
- Minimum clearance: 1.4415–1.5696 m.
- Safety margin: 0.5415–0.6696 m.
- RMS: 0.1325–0.1672 m.
- Maximum error: 0.3802–0.4020 m.

## Readiness assessment

| Dimension | Status | Basis |
|---|---|---|
| Configuration | ready for sim | frozen G12/v6, S4, D path |
| Planning | demonstrated | S4 A/B/C/D, D real pilots |
| Control | demonstrated | five real pilots + 33 joint records |
| Integration | demonstrated | xdyn + Gazebo + ROS + gate/trial |
| Repeatability | demonstrated in sim | P0–P4 three runs for all three flows |
| Boundary/gate | demonstrated | gate rejection and failure records |
| GUI video | not accepted | camera MP4 black due EGL/render |
| Sea trial | not attempted | no hardware/sea evidence |

## Conclusion

Ready for **simulator-level demonstration and review**. Not operationally certified; not a sea-trial result.

Limits: measured current six samples east-only 60–150 s onset; F0/F1 are controls; flat seabed; no vertical shear; camera video blocked by EGL.
