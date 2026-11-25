# Literature Review Source Map (L01–L22)

Date: 2026-11-25  
Purpose: support the literature review of the UUV S4 / G12 / measured-current project.  
Selection rule: prefer peer-reviewed or authoritative sources; keep claims narrower than the source; do not use a source for a claim it does not support.

Notes:

- Reading notes (Chinese, one entry per source below): [literature_notes_20261125/00_reading_guide.md](literature_notes_20261125/00_reading_guide.md); comparison matrix: [99_comparison_matrix.md](literature_notes_20261125/99_comparison_matrix.md).
- DOI metadata were checked against Crossref on 2026-11-25.
- Books and standards without DOI are marked `book` / `standard` / `web`.
- The target is 2–3 sources per theme; L11 has one core source because it is a 1–2 target.
- These are **sources to read and cite**, not claims that the project implemented every method.

---

## 1. Quick map

| ID | Theme | Recommended sources (short) | Use in the paper | Boundary |
|---|---|---|---|---|
| L01 | UUV/AUV background | Paull et al. 2014; Sahoo et al. 2019; Wynn et al. 2014 | Introduction: applications, navigation/obstacle avoidance importance | Do not turn a review into a claim that all AUVs are autonomous |
| L02 | Marine craft 6-DOF dynamics | Fossen 2011; Antonelli 2018 | Method: rigid-body + added mass + Coriolis + damping + restoring | Fossen is a general marine-craft reference, not an AUV-specific validation |
| L03 | Longitudinal dynamics | Healey & Lienard 1993; Lapierre & Soetanto 2007 | Method: surge–heave–pitch reduction and dive-plane model | A reduced model must be justified; do not present it as full 6-DOF |
| L04 | Underactuated UUV | Aguiar & Hespanha 2007; Do & Pan 2009; Gao et al. 2016 | Method: no direct heave; depth via surge/pitch | Underactuated control literature often assumes different actuator layouts |
| L05 | Thruster allocation | Johansen & Fossen 2013; Fossen & Johansen 2006 | Method: force/moment mapping and constrained allocation | Allocation survey is general; AUV-specific limits still need project evidence |
| L06 | Saturation / anti-windup | Tarbouriech & Turner 2009; Kim et al. 2013 | Method: thrust limits, integral windup, feedback correction | Anti-windup theory does not by itself prove AUV stability |
| L07 | Ocean current modeling | Fossen 2011; Al Makdah et al. 2019; Kim et al. 2018 | Method: relative velocity, constant/time-varying current | Current observer papers are not the same as using a measured schedule |
| L08 | Global planning in known static maps | LaValle 2006; Wang et al. 2004 | Scope: offline global planning in a known map | Do not claim online/unknown-environment planning |
| L09 | A* theory | Hart et al. 1968; Dechter & Pearl 1985 | Method: completeness, admissibility, optimality | A* optimality depends on heuristic/graph assumptions |
| L10 | Occupancy grid / inflation | Elfes 1989; Latombe 1991 | Method: grid, obstacle inflation, clearance | Clearance is a design constraint, not a physical collision guarantee |
| L11 | Path shortcutting | Geraerts & Overmars 2007 | Method: safe shortcut/pruning | Shortcutting must preserve collision checking |
| L12 | Path smoothing / fillet | Fraichard & Scheuer 2004; Yang & Sukkarieh 2010; Park 2022 | Method: curvature continuity, corner reduction | Smoothing can reduce clearance; re-check geometry |
| L13 | LOS guidance | Lekkas & Fossen 2014; Fossen & Lekkas 2015; Fossen et al. 2003 | Method: lookahead, cross-track error, integral LOS | LOS stability assumptions must be stated |
| L14 | PID / cascaded PID | Sahoo et al. 2019; Bingul & Gul 2023 | Method: speed/pitch/path-error loops | PID tuning is project-specific; papers do not validate our gains |
| L15 | Planning–control coupling | Donald et al. 1993; Khorrami & Krishnamurthy 2009 | Discussion: geometric path ≠ dynamically executable path | Do not claim joint optimization if planning and control are separate |
| L16 | MPC / SMC alternatives | Shen & Shi 2020; Wang & Du 2024 | Related work: why a simpler PID was chosen | Do not claim our PID outperforms MPC/SMC |
| L17 | RRT*/APF/ACO alternatives | Kot 2022; Banik et al. 2018 | Related work: alternative planners | We did not implement these methods |
| L18 | ROS/Gazebo simulation | Koenig & Howard 2004; Buche et al. 2026; Quigley et al. 2009 | System integration and reproducible simulation | Simulator use is not model validation by itself |
| L19 | Verification & validation | Sargent 2013; Oberkampf & Roy 2010 | Method: numerical/model consistency, coordinate/time checks | V&V of a simulator is not sea-trial validation |
| L20 | Reproducible experiments | Sandve et al. 2013; Peng 2011 | Method: frozen config, hashes, logs, failures | Reproducibility does not imply generalization |
| L21 | NOAA measured current | NOAA CO-OPS station pm0101; NOAA measurement specifications | Data section: source, variables, limitations | Single station, near-surface, 6-min sampling; not a full current field |
| L22 | Simulation-to-reality gap | Zhao et al. 2020; Tiboni et al. 2023 | Limitations: synthetic parameters, truth feedback, no sea trial | Do not extrapolate simulator success to operational deployment |

---

## 2. Detailed entries

### L01 — UUV/AUV applications and autonomous navigation

**Use for:** opening background; why navigation and obstacle avoidance matter.

1. **Paull, L., Saeedi, S., Seto, M., & Li, H. (2014).** AUV navigation and localization: A review. *IEEE Journal of Oceanic Engineering*, 39(1), 131–149. DOI: [10.1109/JOE.2013.2278891](https://doi.org/10.1109/JOE.2013.2278891)
   - Use for: navigation/localization taxonomy and sensing limits.
   - Do not use for: proof that a specific planner is optimal.

2. **Sahoo, A., Dwivedy, S. K., & Robi, P. S. (2019).** Advancements in the field of autonomous underwater vehicle. *Ocean Engineering*, 181, 145–160. DOI: [10.1016/j.oceaneng.2019.04.011](https://doi.org/10.1016/j.oceaneng.2019.04.011)
   - Use for: AUV applications, design trends, autonomy challenges.

3. **Wynn, R. B., et al. (2014).** Autonomous Underwater Vehicles (AUVs): Their past, present and future contributions to the advancement of marine geoscience. *Marine Geology*, 352, 451–468. DOI: [10.1016/j.margeo.2014.03.012](https://doi.org/10.1016/j.margeo.2014.03.012)
   - Use for: application breadth and operational context.

### L02 — General marine-craft dynamics

1. **Fossen, T. I. (2011).** *Handbook of Marine Craft Hydrodynamics and Motion Control*. Wiley. DOI: [10.1002/9781119994138](https://doi.org/10.1002/9781119994138)
   - Use for: 6-DOF rigid-body/added-mass/Coriolis/damping/restoring formulation; relative velocity.

2. **Antonelli, G. (2018).** *Underwater Robots* (4th ed.). Springer. DOI: [10.1007/978-3-319-77899-0](https://doi.org/10.1007/978-3-319-77899-0)
   - Use for: underwater-specific modeling and control context.

### L03 — Longitudinal-plane dynamics

1. **Healey, A. J., & Lienard, D. (1993).** Multivariable sliding mode control for autonomous diving and steering of unmanned underwater vehicles. *IEEE Journal of Oceanic Engineering*, 18(3), 327–339. DOI: [10.1109/JOE.1993.236372](https://doi.org/10.1109/JOE.1993.236372)
   - Use for: dive-plane/longitudinal model and steering/diving control.

2. **Lapierre, L., & Soetanto, D. (2007).** Nonlinear path-following control of an AUV. *Ocean Engineering*, 34(11–12), 1734–1744. DOI: [10.1016/j.oceaneng.2006.10.019](https://doi.org/10.1016/j.oceaneng.2006.10.019)
   - Use for: path-following formulation in the vertical plane.

### L04 — Underactuated UUV

1. **Aguiar, A. P., & Hespanha, J. P. (2007).** Trajectory-tracking and path-following of underactuated autonomous vehicles with parametric modeling uncertainty. *IEEE Transactions on Automatic Control*, 52(8), 1362–1379. DOI: [10.1109/TAC.2007.902731](https://doi.org/10.1109/TAC.2007.902731)
   - Use for: underactuated tracking/path-following framework.

2. **Do, K. D., & Pan, J. (2009).** *Control of Ships and Underwater Vehicles*. Springer. DOI: [10.1007/978-1-84882-730-1](https://doi.org/10.1007/978-1-84882-730-1)
   - Use for: underactuated marine vehicle control design.

3. **Gao, D., Cheng, J., & Yang, Q. (2016).** Depth control for underactuated AUV in vertical plane using optimal internal model controller. *CCDC 2016*. DOI: [10.1109/CCDC.2016.7531944](https://doi.org/10.1109/CCDC.2016.7531944)
   - Use for: explicit depth control with underactuation.

### L05 — Thruster allocation

1. **Johansen, T. A., & Fossen, T. I. (2013).** Control allocation—A survey. *Automatica*, 49(5), 1087–1103. DOI: [10.1016/j.automatica.2013.01.035](https://doi.org/10.1016/j.automatica.2013.01.035)
   - Use for: constrained allocation, actuator redundancy, attainable set.

2. **Fossen, T. I., & Johansen, T. A. (2006).** A survey of control allocation methods for ships and underwater vehicles. *MED 2006*. DOI: [10.1109/MED.2006.328749](https://doi.org/10.1109/MED.2006.328749)
   - Use for: marine-specific allocation context.

### L06 — Saturation and anti-windup

1. **Tarbouriech, S., & Turner, M. (2009).** Anti-windup design: an overview of some recent advances and open problems. *IET Control Theory & Applications*, 3(1), 1–19. DOI: [10.1049/iet-cta:20070435](https://doi.org/10.1049/iet-cta:20070435)
   - Use for: saturation, windup, anti-windup mechanisms.

2. **Kim, M., Joe, H., Pyo, J., Kim, J., Kim, H., & Yu, S.-C. (2013).** Variable-structure PID controller with anti-windup for autonomous underwater vehicle. *OCEANS 2013 San Diego*. DOI: [10.23919/OCEANS.2013.6741024](https://doi.org/10.23919/OCEANS.2013.6741024)
   - Use for: AUV-specific PID anti-windup.

### L07 — Ocean current modeling

1. **Fossen (2011)** — relative-velocity formulation and current disturbance.
2. **Al Makdah, A. A. R., Daher, N., Asmar, D., & Shammas, E. (2019).** Three-dimensional trajectory tracking of a hybrid autonomous underwater vehicle in the presence of underwater current. *Ocean Engineering*, 185, 14–27. DOI: [10.1016/j.oceaneng.2019.05.030](https://doi.org/10.1016/j.oceaneng.2019.05.030)
   - Use for: current-affected 3D tracking.
3. **Kim, E., Fan, S., & Bose, N. (2018).** Autonomous Underwater Vehicle Model-Based High-Gain Observer for Ocean Current Estimation. *AUV 2018*. DOI: [10.1109/AUV.2018.8729741](https://doi.org/10.1109/AUV.2018.8729741)
   - Use for: current estimation/observer context.
   - Boundary: observer/estimation ≠ using NOAA measured schedule.

### L08 — Global planning in known static environments

1. **LaValle, S. M. (2006).** *Planning Algorithms*. Cambridge University Press. Book. Online: [planning.cs.uiuc.edu](http://planning.cs.uiuc.edu/)
   - Use for: global planning, completeness, configuration space.

2. **Wang, H.-J., Bian, X.-Q., Zhang, X., Fu, M.-Y., & Li, J. (2004).** Two approaches for autonomous underwater vehicle global path planning in large range ocean environment. *ICIMA 2004*. DOI: [10.1109/ICIMA.2004.1384193](https://doi.org/10.1109/ICIMA.2004.1384193)
   - Use for: AUV global planning in a known map.

### L09 — A* theory

1. **Hart, P. E., Nilsson, N. J., & Raphael, B. (1968).** A formal basis for the heuristic determination of minimum cost paths. *IEEE Transactions on Systems Science and Cybernetics*, 4(2), 100–107. DOI: [10.1109/TSSC.1968.300136](https://doi.org/10.1109/TSSC.1968.300136)
   - Use for: A* original formulation.

2. **Dechter, R., & Pearl, J. (1985).** Generalized best-first search strategies and the optimality of A*. *Journal of the ACM*, 32(3), 505–536. DOI: [10.1145/3828.3830](https://doi.org/10.1145/3828.3830)
   - Use for: optimality conditions and heuristic assumptions.

### L10 — Occupancy grid and obstacle inflation

1. **Elfes, A. (1989).** Using occupancy grids for mobile robot perception and navigation. *Computer*, 22(6), 46–57. DOI: [10.1109/2.30720](https://doi.org/10.1109/2.30720)
   - Use for: occupancy grid representation.

2. **Latombe, J.-C. (1991).** *Robot Motion Planning*. Kluwer/Springer. Book.
   - Use for: configuration-space planning and clearance.

### L11 — Path shortcutting

1. **Geraerts, R., & Overmars, M. H. (2007).** Creating high-quality paths for motion planning. *The International Journal of Robotics Research*, 26(8), 845–863. DOI: [10.1177/0278364907079280](https://doi.org/10.1177/0278364907079280)
   - Use for: path shortcutting/pruning while preserving collision-free properties.
   - Boundary: every shortcut must be re-checked against clearance.

### L12 — Path smoothing / fillet

1. **Fraichard, T., & Scheuer, A. (2004).** From Reeds and Shepp's to continuous-curvature paths. *IEEE Transactions on Robotics*, 20(6), 1025–1035. DOI: [10.1109/TRO.2004.833789](https://doi.org/10.1109/TRO.2004.833789)
   - Use for: continuous-curvature path construction.

2. **Yang, K., & Sukkarieh, S. (2010).** An analytical continuous-curvature path-smoothing algorithm. *IEEE Transactions on Robotics*, 26(3), 561–568. DOI: [10.1109/TRO.2010.2042990](https://doi.org/10.1109/TRO.2010.2042990)
   - Use for: analytical smoothing and curvature continuity.

3. **Park, S. (2022).** Three-dimensional Dubins-path-guided continuous curvature path smoothing. *Applied Sciences*, 12(22), 11336. DOI: [10.3390/app122211336](https://doi.org/10.3390/app122211336)
   - Use for: 3D AUV-relevant smoothing context.
   - Boundary: smoothing may reduce obstacle clearance; re-check after smoothing.

### L13 — LOS path-following guidance

1. **Lekkas, A. M., & Fossen, T. I. (2014).** Integral LOS path following for curved paths based on a monotone cubic Hermite spline parametrization. *IEEE Transactions on Control Systems Technology*, 22(6), 2287–2301. DOI: [10.1109/TCST.2014.2306774](https://doi.org/10.1109/TCST.2014.2306774)
   - Use for: LOS with curved paths and integral action.

2. **Fossen, T. I., & Lekkas, A. M. (2015).** Direct and indirect adaptive integral line-of-sight path-following controllers for marine craft exposed to ocean currents. *International Journal of Adaptive Control and Signal Processing*, 29(4), 445–463. DOI: [10.1002/acs.2550](https://doi.org/10.1002/acs.2550)
   - Use for: current-affected LOS perspective.

3. **Fossen, T. I., Breivik, M., & Skjetne, R. (2003).** Line-of-sight path following of underactuated marine craft. *IFAC Proceedings Volumes*, 36(21), 211–216. DOI: [10.1016/S1474-6670(17)37809-6](https://doi.org/10.1016/S1474-6670(17)37809-6)
   - Use for: foundational LOS guidance.

### L14 — PID / cascaded PID

1. **Sahoo, A., Dwivedy, S. K., & Robi, P. S. (2019).** Development of a PID control strategy for a compact autonomous underwater vehicle. *OMAE 2019*. DOI: [10.1115/OMAE2019-95345](https://doi.org/10.1115/OMAE2019-95345)
   - Use for: AUV PID implementation context.

2. **Bingul, Z., & Gul, K. (2023).** Intelligent-PID with PD feedforward trajectory tracking control of an autonomous underwater vehicle. *Machines*, 11(2), 300. DOI: [10.3390/machines11020300](https://doi.org/10.3390/machines11020300)
   - Use for: PID/feedforward trajectory tracking.
   - Boundary: our gains and cascade structure are project-specific.

### L15 — Planning–control coupling

1. **Donald, B., Xavier, P., Canny, J., & Reif, J. (1993).** Kinodynamic motion planning. *Journal of the ACM*, 40(5), 1048–1066. DOI: [10.1145/174147.174150](https://doi.org/10.1145/174147.174150)
   - Use for: geometric path vs dynamic feasibility.

2. **Khorrami, F., & Krishnamurthy, P. (2009).** A hierarchical path planning and obstacle avoidance system for an autonomous underwater vehicle. *ACC 2009*. DOI: [10.1109/ACC.2009.5160300](https://doi.org/10.1109/ACC.2009.5160300)
   - Use for: AUV planning/avoidance integration.

### L16 — MPC and SMC alternatives

1. **Shen, C., & Shi, Y. (2020).** Distributed implementation of nonlinear model predictive control for AUV trajectory tracking. *Automatica*, 115, 108863. DOI: [10.1016/j.automatica.2020.108863](https://doi.org/10.1016/j.automatica.2020.108863)
   - Use for: MPC alternative and computational burden.

2. **Wang, Y., & Du, Z. (2024).** Trajectory tracking control for an underactuated AUV via nonsingular fast terminal sliding mode approach. *Journal of Marine Science and Engineering*, 12(8), 1442. DOI: [10.3390/jmse12081442](https://doi.org/10.3390/jmse12081442)
   - Use for: SMC alternative and robustness claims.
   - Boundary: do not claim our PID outperforms them without an experiment.

### L17 — Alternative planners (RRT*, APF, ACO)

1. **Kot, R. (2022).** Review of collision avoidance and path planning algorithms used in autonomous underwater vehicles. *Electronics*, 11(15), 2301. DOI: [10.3390/electronics11152301](https://doi.org/10.3390/electronics11152301)
   - Use for: survey of AUV planners.

2. **Banik, S., Rathinam, S., & Sujit, P. B. (2018).** Min-max path planning algorithms for heterogeneous, autonomous underwater vehicles. *AUV 2018*. DOI: [10.1109/AUV.2018.8729714](https://doi.org/10.1109/AUV.2018.8729714)
   - Use for: alternative planning formulations.
   - Boundary: we did not implement RRT*/APF/ACO.

### L18 — ROS/Gazebo and simulation integration

1. **Koenig, N., & Howard, A. (2004).** Design and use paradigms for Gazebo, an open-source multi-robot simulator. *IROS 2004*. DOI: [10.1109/IROS.2004.1389727](https://doi.org/10.1109/IROS.2004.1389727)
   - Use for: Gazebo architecture and simulation role.

2. **Buche, C., et al. (2026).** LOTUSim: Multi-domain simulator for marine robotics. *IROS 2026*. DOI: [10.1109/IROS61738.2026.11777234](https://doi.org/10.1109/IROS61738.2026.11777234)
   - Use for: LOTUSim/marine robotics simulation context.
   - Check camera-ready metadata before final submission.

3. **Quigley, M., et al. (2009).** ROS: an open-source Robot Operating System. *ICRA Workshop on Open Source Software*.
   - Use for: ROS middleware/integration.
   - No DOI; cite the workshop paper, not a generic website.

### L19 — Verification and validation

1. **Sargent, R. G. (2013).** Verification and validation of simulation models. *Journal of Simulation*, 7(1), 12–24. DOI: [10.1057/jos.2012.20](https://doi.org/10.1057/jos.2012.20)
   - Use for: simulation V&V concepts.

2. **Oberkampf, W. L., & Roy, C. J. (2010).** *Verification and Validation in Scientific Computing*. Cambridge University Press. DOI: [10.1017/CBO9780511760396](https://doi.org/10.1017/CBO9780511760396)
   - Use for: code verification, solution verification, validation hierarchy.
   - Boundary: model V&V ≠ sea-trial validation.

### L20 — Reproducible experiments / research software

1. **Sandve, G. K., Nekrutenko, A., Taylor, J., & Hovig, E. (2013).** Ten simple rules for reproducible computational research. *PLoS Computational Biology*, 9(10), e1003285. DOI: [10.1371/journal.pcbi.1003285](https://doi.org/10.1371/journal.pcbi.1003285)
   - Use for: frozen inputs, scripts, provenance, logging.

2. **Peng, R. D. (2011).** Reproducible research in computational science. *Science*, 334(6060), 1226–1227. DOI: [10.1126/science.1213847](https://doi.org/10.1126/science.1213847)
   - Use for: reproducibility as a research standard.

### L21 — NOAA measured current data

1. **NOAA CO-OPS.** Station pm0101 — Portsmouth Harbor, Gangway Rocks LB13. Web: [https://tidesandcurrents.noaa.gov/cdata/StationInfo?id=pm0101](https://tidesandcurrents.noaa.gov/cdata/StationInfo?id=pm0101)
   - Use for: station identity, location, near-surface data, sample interval, deployment metadata.
   - Observed on page: sample interval 6 min; near-surface current; station depth 11.00 m; UTC−5.

2. **NOAA CO-OPS.** Measurement specifications / currents information. Web: [https://tidesandcurrents.noaa.gov/measure.html](https://tidesandcurrents.noaa.gov/measure.html) and [https://tidesandcurrents.noaa.gov/currents_info.html](https://tidesandcurrents.noaa.gov/currents_info.html)
   - Use for: measurement variables, units, quality/limitations.
   - Boundary: one station, near-surface, six samples used in the project; not a full 3D current field.

### L22 — Simulation-to-reality gap

1. **Zhao, W., Queralta, J. P., & Westerlund, T. (2020).** Sim-to-real transfer in deep reinforcement learning for robotics: a survey. *SSCI 2020*. DOI: [10.1109/SSCI47803.2020.9308468](https://doi.org/10.1109/SSCI47803.2020.9308468)
   - Use for: general sim-to-real gap framing.

2. **Tiboni, G., Arndt, K., & Kyrki, V. (2023).** DROPO: Sim-to-real transfer with offline domain randomization. *Robotics and Autonomous Systems*, 166, 104432. DOI: [10.1016/j.robot.2023.104432](https://doi.org/10.1016/j.robot.2023.104432)
   - Use for: domain randomization and reality-gap mitigation.
   - Boundary: project has no sea trial; use for limitations, not proof of transfer.

---

## 3. Suggested paragraph placement

| Section | Sources |
|---|---|
| Introduction: AUV applications | L01 |
| Introduction: why planning/control integration | L15, L17 |
| Method: 6-DOF and longitudinal model | L02, L03 |
| Method: underactuation and depth | L04 |
| Method: allocation and saturation | L05, L06 |
| Method: current | L07 |
| Method: global planning | L08, L09, L10 |
| Method: shortcut/smoothing | L11, L12 |
| Method: LOS/PID | L13, L14 |
| Related work: MPC/SMC and other planners | L16, L17 |
| System integration | L18 |
| Verification/reproducibility | L19, L20 |
| Data | L21 |
| Limitations | L22 |

---

## 4. Citation cautions

1. **Do not over-cite A* optimality** without stating the heuristic/graph assumptions.
2. **Do not claim “collision-free”** from a smoothed path until clearance is re-checked.
3. **Do not claim the project implemented RRT*/APF/ACO**; cite them as alternatives.
4. **Do not claim PID outperforms MPC/SMC** without a controlled comparison.
5. **Do not call the NOAA six-sample profile a full current field.**
6. **Do not equate simulator V&V with sea-trial validation.**
7. **Do not cite LOTUSim 2026 as a fixed published version** until the camera-ready metadata is confirmed.

---

## 5. Minimal reading order

If time is limited:

1. L01 Paull 2014 + Sahoo 2019
2. L02 Fossen 2011
3. L03 Healey & Lienard 1993
4. L04 Aguiar & Hespanha 2007
5. L09 Hart 1968 + Dechter & Pearl 1985
6. L12 Fraichard & Scheuer 2004
7. L13 Lekkas & Fossen 2014
8. L14 Sahoo 2019 / Bingul 2023
9. L15 Donald 1993
10. L18 Koenig & Howard 2004 + LOTUSim 2026
11. L19 Sargent 2013
12. L21 NOAA pm0101
13. L22 Zhao 2020

---

## 6. BibTeX keys

Companion file: [literature_review_sources_20261125.bib](literature_review_sources_20261125.bib)

| ID | BibTeX keys |
|---|---|
| L01 | `paull2014auv`, `sahoo2019advancements`, `wynn2014auvs` |
| L02 | `fossen2011handbook`, `antonelli2018underwater` |
| L03 | `healey1993multivariable`, `lapierre2007nonlinear` |
| L04 | `aguiar2007trajectory`, `do2009control`, `gao2016depth` |
| L05 | `johansen2013control`, `fossen2006survey` |
| L06 | `tarbouriech2009antiwindup`, `kim2013variable` |
| L07 | `fossen2011handbook`, `almakdah2019three`, `kim2018observer` |
| L08 | `lavalle2006planning`, `wang2004two` |
| L09 | `hart1968formal`, `dechter1985generalized` |
| L10 | `elfes1989using`, `latombe1991robot` |
| L11 | `geraerts2007creating` |
| L12 | `fraichard2004reeds`, `yang2010analytical`, `park2022three` |
| L13 | `lekkas2014integral`, `fossen2015direct`, `fossen2003line` |
| L14 | `sahoo2019development`, `bingul2023intelligent` |
| L15 | `donald1993kinodynamic`, `khorrami2009hierarchical` |
| L16 | `shen2020distributed`, `wang2024trajectory` |
| L17 | `kot2022review`, `banik2018minmax` |
| L18 | `koenig2004design`, `buche2026lotusim`, `quigley2009ros` |
| L19 | `sargent2013verification`, `oberkampf2010verification` |
| L20 | `sandve2013ten`, `peng2011reproducible` |
| L21 | `noaa2024pm0101`, `noaa_coops_measure` |
| L22 | `zhao2020sim`, `tiboni2023dropo` |

Note: `fossen2011handbook` is reused in L02 and L07 intentionally; the same source supports the general model and the relative-velocity/current formulation.
