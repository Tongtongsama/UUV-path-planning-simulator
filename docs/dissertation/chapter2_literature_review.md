# 2 Literature Review

This chapter explains how existing research has shaped the design of the navigation system studied here and states the questions the experiments must answer. The review follows the execution chain rather than grouping work by algorithm family: scope (§2.1), underactuated dynamics (§2.2), environmental and actuator limits (§2.3), planning (§2.4), path processing (§2.5), guidance and tracking (§2.6) and verification (§2.7). Section 2.8 states the research questions; parameters, code and results are deferred to Chapters 3–7.

## 2.1 Task Context and Scope of Underwater Navigation

Autonomous underwater vehicles are used for survey, inspection and geoscience work where ship-based observation is expensive, sparse or impossible (Wynn et al., 2014; Sahoo et al., 2019a). Navigation underlies all of it: a vehicle that does not know where it is cannot execute a mission, and one that cannot hold a track cannot deliver consistent data (Paull et al., 2014). Much of this work reduces, operationally, to a point-to-point transit: the vehicle is launched with a route fixed in advance, moves from one task point to another without collision or large deviation from the intended track, and is recovered (Wynn et al., 2014). That transit is the unit of work simulated here.

A complete navigation system contains several stages: estimating the vehicle state, representing the environment, planning a route, turning it into guidance commands, and tracking them with the actuators. Localisation and mapping are problems in their own right, with their own error structure: accuracy is set less by the filter than by how often an external correction is available (Paull et al., 2014). Here both are inputs. A state and a map are assumed, and the question is what can be established about the motion produced from them. Keeping that boundary makes the execution question identifiable: a failure to follow a route can be sought in planning, guidance, control or actuation rather than in estimation.

The setting is narrow: a single vehicle, a static obstacle map known in advance, a given start and goal, and state feedback from simulation ground truth. Formally this is the planning problem of LaValle (2006): a state space, an obstacle region, the free space that remains, a start and a goal, a path as a continuous map into the free space, and a cost functional. Within it the thesis asks whether a route computed on the map can be executed safely when the vehicle is underactuated, subject to a current, and bounded in thrust. Dynamic obstacles, online mapping and localisation error are outside the scope and are treated as limitations in Chapter 8.

## 2.2 UUV Dynamics and Underactuated Motion

The standard description of a marine vehicle is Fossen's (2011) six-degree-of-freedom vector model, in which rigid-body and hydrodynamic forces are collected into inertia, Coriolis and centripetal terms, damping, restoring forces and the generalized actuator force. Inertia contains rigid-body and added mass, the latter taken as constant, ignoring the frequency dependence of radiation forces. Coriolis terms arise because the equations are written in a rotating body frame. Damping is linear plus quadratic, the latter dominating at speed. Restoring forces depend on attitude and depth and appear in the longitudinal plane mainly as pitch stiffness. The formulation is a framework, not a parameter set: coefficients must be identified for a particular hull. The parameters used here are synthetic and interpretable, so the model supports statements about this system, not predictions for a particular vehicle.

The mission is defined in the longitudinal x–z plane. The justification is not that other degrees of freedom are absent but that the reduction has a documented basis: Healey and Lienard (1993) separate diving and steering under a weak-coupling assumption valid for small attitude angles and low speed, and state it explicitly; Lapierre and Soetanto (2007) formulate vertical-plane path following. The reduction is a recognised modelling choice with conditions, and the conditions matter: decoupling is an assumption to be tested, not a fact to be asserted. It is equally important to separate the reduction used for design from the model integrated. The solver integrates a synthetic six-degree-of-freedom model; the mission and acceptance criteria are formulated in the longitudinal plane, but the solver is not replaced by a three-degree-of-freedom surrogate. The reduced equations design and interpret, the full model supplies the measured motion, and the analysis stays exposed to the coupling it neglects.

In thruster-configuration language, the actuators define a map from thruster forces to generalized force through a configuration matrix, whose rank determines which generalized-force directions can be produced at all (Antonelli, 2018). The vehicle here has two axial thrusters above and below the centre line. They produce a surge force and a pitch moment; neither produces a direct heave force. The consequence is structural rather than a matter of tuning: depth changes must be generated through pitch and forward speed, and the set of executable motions is smaller than the set of smooth references that can be written down. Aguiar and Hespanha (2007) make this precise for a general class of underactuated vehicles: trackability is a property of the reference relative to the vehicle, not of the controller alone. A geometric route must therefore be compatible with the vehicle's motion capability before it can be followed. The matrix, its rank and the envelope are derived in Chapter 3; the point carried forward is that planning outputs must be judged against actuation.

## 2.3 Ocean Currents, Control Allocation and Actuator Saturation

A current enters the dynamics through the relative velocity between vehicle and water (Fossen, 2011). This is the only consistent entry point: hydrodynamic forces depend on the motion of the water relative to the hull, not of the hull relative to the ground. The substitution is exact under three assumptions — the current is constant in time, irrotational and spatially uniform. Under them a constant current does not by itself destabilise the relative motion; for an asymmetric vehicle it leaves a constant bias, visible as a steady drift angle. Two consequences follow. A constant-current experiment tests the rejection of a constant bias, not the response to a realistic current. Second, when the current varies in time or space one assumption fails, so such cases are reported separately. The formal matrix here uses a constant current; the later demonstration with a measured, time-varying series is descriptive and outside the formal acceptance.

Control allocation sits between controller and thrusters. The controller requests a surge force and a pitch moment; the allocator finds thruster forces that produce them. In general this is constrained: the set of generalized forces the thrusters can produce is the attainable set, and allocation maps a request to the best point of it, using pseudo-inverse, weighted least squares or quadratic programming (Johansen and Fossen, 2013). Here the map is square and invertible — two requests, two thrusters — so nominal allocation is a direct inversion, and low-redundancy configurations of this kind leave no room to redistribute a deficit (Fossen and Johansen, 2006). The rule used here, uniform scaling of both commands when the request exceeds the envelope, is an implementation choice of this thesis rather than a method from the literature; it is specified in Chapter 4.

The envelope is set by the thrust limit of each thruster. With two equal limits the realisable force–moment pairs form a diamond, and requests outside it cannot be realised exactly. The controller keeps asking for more while the vehicle delivers less, and integral action accumulates the difference. Anti-windup aims to make the response under saturation as close as possible to the response without it; the theory is mainly for linear plants, for which conditions on performance recovery can be given (Tarbouriech and Turner, 2009). It does not certify the nonlinear, underactuated loop considered here, and no proof is borrowed from it. The closer precedent is the dual-loop variable-structure PID with anti-windup reported by Kim et al. (2013), which reduces overshoot and settling time on a real vehicle. The motivation is adopted; the guarantee is not. The distinction between a physical limit and a guidance law is maintained in §2.6.

## 2.4 A* Planning in a Known Static Environment

Global planning in a known static environment is the problem of finding a path from a start to a goal through free space at minimum cost. LaValle (2006) supplies the vocabulary: a state space, an obstacle region, the free space that remains, a path as a continuous map into it, and a cost functional. The same source separates completeness, resolution completeness and probabilistic completeness, a distinction that matters once free space is discretised. Here the problem is solved offline, once, on a map known before the run begins; no claim of online planning is made.

A* is the algorithm used. Hart et al. (1968) evaluated a node by the cost accumulated plus a heuristic estimate of the remaining cost, and showed that the search returns a minimum-cost path when the estimate never exceeds the true remaining cost — when it is admissible; consistency additionally guarantees that no node must be reopened. Dechter and Pearl (1985) place A* in a family of best-first strategies and show how relaxing admissibility to ε-admissibility trades optimality for speed with an explicit bound on the loss. Optimality is thus a property of the triple of graph, edge costs and heuristic, not of the algorithm's name: change the discretisation or the cost and the guarantee must be re-established.

The planner here searches a finite grid in the x–z plane with eight-connected moves and Euclidean edge costs, from a start cell to a goal region, with obstacles inflated by the vehicle radius plus a margin so that a point search approximates a vehicle with extent (Latombe, 1991). The search is bounded in node expansions and time. Three consequences follow. The path is optimal only with respect to the constructed graph and the specified cost; it is not optimal in the continuous plane, and nothing makes it optimal in time, energy or closed-loop tracking error. The inflation is a design constraint: the clearance it produces belongs to the representation, because the equivalence between point-robot planning and the original problem holds exactly only for the exact configuration-space obstacle and degrades under discretisation. And the grid is deterministic, not probabilistic; the occupancy-grid literature (Elfes, 1989) is cited for the representation only. The goal is a region rather than a point, and the arrival test is applied to the executed trajectory rather than to the reference path. Large-range planning on reduced-resolution maps (Wang et al., 2004) addresses a different regime and is left to future work.

## 2.5 Path Shortcutting, Smoothing and Executability

The output of a grid search is a polyline. It is safe in the sense of the graph — no cell on it is in the inflated obstacle set — but it is not yet a reference a vehicle can follow. Consecutive segments meet at vertices where the tangent changes abruptly, and the staircase detours are artefacts of the eight-connected grid rather than properties of the task. For a vehicle with inertia and bounded thrust, an abrupt change of tangent is a step demand on the inner loop: guidance asks for a new heading at once, the controller saturates, and the vehicle overshoots the vertex it was meant to round. Post-processing is where a geometric solution becomes a candidate reference.

Shortcutting addresses the first defect. Geraerts and Overmars (2007) replace subpaths by straight segments whenever the segment is collision-free, iterating within a budget; each accepted shortcut can only reduce length, so cost decreases monotonically. The operation is safe only if every candidate is checked: a straight line between distant points may pass closer to an obstacle than the polyline it replaces, so the result can be shorter and yet have less clearance. The shortcutting used here follows that pattern: candidates are validated against the same inflated map, accepted only if planning clearance is preserved, and rejected candidates are logged. A fixed seed and budget keep it deterministic, and the output is a shorter polyline with fewer vertices.

Smoothing addresses the second defect. Fraichard and Scheuer (2004) formulate continuous-curvature paths with bounded curvature and curvature derivative, built from line, arc and clothoid segments; a discontinuity in curvature is a discontinuity in steering demand, which a real vehicle cannot produce. The construction has a cost: transition segments consume space and add length, and the result is not length-optimal. Yang and Sukkarieh (2010) give a closed form from parametric cubic Bézier segments requiring only a maximum curvature, and note that the tangent-continuous construction is not curvature-continuous at the joins; only the curvature-continuous variant is. Park (2022) extends smoothing to three dimensions. The stage used here is corner rounding: each vertex becomes a quadratic segment whose extent is limited relative to the adjacent edges, and each rounded segment is checked against the inflated map. It reproduces none of these algorithms and does not make the reference curvature-continuous; it reduces the discrete turning angle. Whether that improves the executed motion is an empirical question.

The organising distinction is kinodynamic: a path feasible geometrically need not be feasible dynamically, and planning in position space cannot establish the latter (Donald et al., 1993). In the hierarchical planner of Khorrami and Krishnamurthy (2009) the interface between layers is a waypoint list and a speed command. Such an interface carries no turning or braking capability, so the lower layer can receive a reference it cannot execute. Two families of indicators follow. Geometric ones — length, clearance, number and magnitude of turns — describe the reference; closed-loop ones — tracking error, saturation, swept clearance, arrival — describe the execution. Neither substitutes for the other. This motivates the controlled comparison reported in Chapter 7, which applies one vehicle, guidance law, controller and acceptance set to an unprocessed A* path, a shortened path, a rounded path, and a shortened path with a terminal approach. The literature motivates the comparison, not its outcome.

## 2.6 Path Following Guidance and Tracking Control

Planning produces a geometric reference; guidance turns it into a motion reference. The line-of-sight law is standard for underactuated marine craft: the desired heading is the path tangent plus a correction depending on the cross-track error, with a lookahead distance setting the ratio (Fossen et al., 2003). The error dynamics then converge at a rate of order speed over lookahead, so the lookahead is the single continuous parameter trading convergence speed against overshoot: small values correct quickly but demand large heading changes near the path, large ones are smooth but cut corners. Two elements of the original construction are often dropped in application and are retained here: the projection defining cross-track error on the active segment, and a reference model that smooths the switch between segments and supplies the derivative of the heading reference. Without the latter, a waypoint switch is a step the inner loop must absorb.

Guidance is not control. Here guidance computes a pitch reference and a speed reference from the vehicle position, its projection and the lookahead; the controller converts the errors into a surge force and a pitch moment; the allocator converts those into thruster commands. The cascade is the engineering standard for small AUVs, where channel-wise PID loops with separated bandwidths are preferred for transparency and low tuning cost (Sahoo et al., 2019b); model-free variants replace the model with an online estimate of the lumped disturbance (Bingul and Gul, 2023), at the price of numerical differentiation and non-physical parameters. What matters is the closure: the force and moment requested are not those applied, because they pass through allocation and the thrust limit of §2.3. The loop is saturating, and the guidance law's assumption that its commands can be realised is untested.

Two issues arise at the chain's ends. Under a persistent current a proportional law leaves a steady cross-track error: the current produces a drift the law cannot distinguish from a path error. Integral line-of-sight removes it with an integral state in the lookahead (Lekkas and Fossen, 2014); there the integral law is derived for straight-line path following, the curved extension relies on a spline parameterisation, and the stability result covers the guidance-and-heading cascade without actuator saturation. Adaptive variants estimate the current instead and are developed for horizontal-plane motion (Fossen and Lekkas, 2015); estimating it from the vehicle's own motion is a third route, likewise not taken here (Kim et al., 2018). This thesis does not implement integral line-of-sight — its integral action sits in the controller — and no claim of equivalence in current rejection is made. At the other end, a mission ends when the vehicle arrives and stays within a tolerance; a reference passing through the goal at speed is not one that terminates there, hence the terminal approach variant reported in Chapter 7. Outside this structure, model predictive control handles constraints explicitly (Shen and Shi, 2020) and sliding-mode control buys robustness with large control authority (Wang and Du, 2024); neither is compared here, and no ranking is asserted.

## 2.7 Simulation Verification and Reproducibility

The results reported here are simulation results, and their interpretation depends on which component produced which part of them. The platform is LOTUSim, a multi-domain simulator for marine robotics supplying scenario and entity abstractions, an underwater environment and interfaces to external dynamics (Buche et al., 2026). The six-degree-of-freedom motion is integrated by xdyn; Gazebo provides the scene and receives the pose for display (Koenig and Howard, 2004); ROS and a shared-memory IPC path carry commands and state between processes (Quigley et al., 2009). These are platform capabilities. What the platform did here is narrower: the hydrodynamic model, the thruster mapping and the current injection are implemented in this project, the layered current model is not enabled, and the Gazebo pose renders the solver state rather than sourcing motion. A platform's capability is not evidence that a model was used; using a simulator is not evidence that a model was verified.

Verification and validation are separate: verification asks whether the model is implemented correctly, validation whether it is the right model for the intended use, and credibility is relative to that use (Sargent, 2013). Within verification, code verification establishes that the equations are solved as intended, and solution verification estimates numerical error, for instance by halving the step and observing the order of convergence (Oberkampf and Roy, 2010). The checks reported here belong to this layer: zero-input and static-response tests, step-size refinement, coordinate round-trips, time alignment, and a comparison between an independent Python implementation and the solver. They support the claim that, under the stated settings, the recorded motion is the motion of the specified model. They do not support a claim about a real vehicle: the parameters are synthetic and the feedback is ground truth.

Reproducibility is the third condition. The rules are established: keep raw data, script every step, version everything, record software versions, avoid manual operations in graphical interfaces, and treat the ability to reproduce a failure as part of the result (Sandve et al., 2013; Peng, 2011). This thesis freezes configuration, source and dependencies, archives inputs with their hashes, keeps failed runs, and re-runs the reference matrix in a separate environment. The environmental input is deliberately limited: constant-current cases use prescribed values, and the time-varying demonstration uses a short record from a single near-surface station sampled every six minutes, which anchors the magnitude of a disturbance rather than representing a field (NOAA CO-OPS, 2024). The remaining gap decomposes into dynamics, sensing, actuation and environment differences, and closing it needs data this study does not have (Zhao et al., 2020; Tiboni et al., 2023).

## 2.8 Synthesis and Research Positioning

Taken together, the literature supplies a method for each link: a six-degree-of-freedom formulation with a documented longitudinal reduction, a relative-velocity treatment of the current, a constrained view of allocation, a graph search with explicit optimality conditions, post-processing operations, a guidance law with a tracking cascade, and a vocabulary for verification and reproducibility. It does not supply evidence about the chain as a whole under the conditions studied here. Each link is validated under assumptions the next link does not inherit: planning assumes a point or a kinematically constrained vehicle, guidance assumes its commands can be realised, allocation assumes a request that can be met.

This thesis connects these methods in one closed loop and asks three questions. RQ1: how can a UUV model with nonlinear and underactuated characteristics be built and verified for this class of simulation? RQ2: how can environment, planning, guidance, control and allocation be connected so that a point-to-point mission in a static obstacle field is completed by the dynamics rather than by replay? RQ3: how do the current and the actuator limits change safety, tracking and arrival, and where is the boundary of the region in which the system works? The contribution is the connection and the evidence about it, not the invention of any component, and the comparisons reported later are limited to the variants and conditions actually run.

---

## References cited in this chapter

Metadata follow the keys in `docs/literature_review_sources_20261125.bib`; the citation format will be converted to the school template at submission.

Aguiar, A. P., & Hespanha, J. P. (2007). Trajectory-tracking and path-following of underactuated autonomous vehicles with parametric modeling uncertainty. *IEEE Transactions on Automatic Control*, 52(8), 1362–1379. https://doi.org/10.1109/TAC.2007.902731

Antonelli, G. (2018). *Underwater Robots* (4th ed.). Springer. https://doi.org/10.1007/978-3-319-77899-0

Bingul, Z., & Gul, K. (2023). Intelligent-PID with PD feedforward trajectory tracking control of an autonomous underwater vehicle. *Machines*, 11(2), 300. https://doi.org/10.3390/machines11020300

Buche, C., et al. (2026). LOTUSim: A multi-domain simulator for marine robotics. In *Proceedings of the IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS 2026)*. https://doi.org/10.1109/IROS61738.2026.11777234

Dechter, R., & Pearl, J. (1985). Generalized best-first search strategies and the optimality of A*. *Journal of the ACM*, 32(3), 505–536. https://doi.org/10.1145/3828.3830

Donald, B., Xavier, P., Canny, J., & Reif, J. (1993). Kinodynamic motion planning. *Journal of the ACM*, 40(5), 1048–1066. https://doi.org/10.1145/174147.174150

Elfes, A. (1989). Using occupancy grids for mobile robot perception and navigation. *Computer*, 22(6), 46–57. https://doi.org/10.1109/2.30720

Fossen, T. I. (2011). *Handbook of Marine Craft Hydrodynamics and Motion Control*. Wiley. https://doi.org/10.1002/9781119994138

Fossen, T. I., Breivik, M., & Skjetne, R. (2003). Line-of-sight path following of underactuated marine craft. *IFAC Proceedings Volumes*, 36(21), 211–216. https://doi.org/10.1016/S1474-6670(17)37809-6

Fossen, T. I., & Johansen, T. A. (2006). A survey of control allocation methods for ships and underwater vehicles. In *Proceedings of the 14th Mediterranean Conference on Control and Automation (MED 2006)*. https://doi.org/10.1109/MED.2006.328749

Fossen, T. I., & Lekkas, A. M. (2015). Direct and indirect adaptive integral line-of-sight path-following controllers for marine craft exposed to ocean currents. *International Journal of Adaptive Control and Signal Processing*, 29(4), 445–463. https://doi.org/10.1002/acs.2550

Fraichard, T., & Scheuer, A. (2004). From Reeds and Shepp's to continuous-curvature paths. *IEEE Transactions on Robotics*, 20(6), 1025–1035. https://doi.org/10.1109/TRO.2004.833789

Geraerts, R., & Overmars, M. H. (2007). Creating high-quality paths for motion planning. *The International Journal of Robotics Research*, 26(8), 845–863. https://doi.org/10.1177/0278364907079280

Hart, P. E., Nilsson, N. J., & Raphael, B. (1968). A formal basis for the heuristic determination of minimum cost paths. *IEEE Transactions on Systems Science and Cybernetics*, 4(2), 100–107. https://doi.org/10.1109/TSSC.1968.300136

Healey, A. J., & Lienard, D. (1993). Multivariable sliding mode control for autonomous diving and steering of unmanned underwater vehicles. *IEEE Journal of Oceanic Engineering*, 18(3), 327–339. https://doi.org/10.1109/JOE.1993.236372

Johansen, T. A., & Fossen, T. I. (2013). Control allocation — A survey. *Automatica*, 49(5), 1087–1103. https://doi.org/10.1016/j.automatica.2013.01.035

Khorrami, F., & Krishnamurthy, P. (2009). A hierarchical path planning and obstacle avoidance system for an autonomous underwater vehicle. In *Proceedings of the American Control Conference (ACC 2009)*. https://doi.org/10.1109/ACC.2009.5160300

Kim, E., Fan, S., & Bose, N. (2018). Autonomous Underwater Vehicle model-based high-gain observer for ocean current estimation. In *Proceedings of the IEEE/OES Autonomous Underwater Vehicle Workshop (AUV 2018)*. https://doi.org/10.1109/AUV.2018.8729741

Kim, M., Joe, H., Pyo, J., Kim, J., Kim, H., & Yu, S.-C. (2013). Variable-structure PID controller with anti-windup for autonomous underwater vehicle. In *Proceedings of OCEANS 2013 MTS/IEEE San Diego*. https://doi.org/10.23919/OCEANS.2013.6741024

Koenig, N., & Howard, A. (2004). Design and use paradigms for Gazebo, an open-source multi-robot simulator. In *Proceedings of the IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS 2004)*. https://doi.org/10.1109/IROS.2004.1389727

Lapierre, L., & Soetanto, D. (2007). Nonlinear path-following control of an AUV. *Ocean Engineering*, 34(11–12), 1734–1744. https://doi.org/10.1016/j.oceaneng.2006.10.019

Latombe, J.-C. (1991). *Robot Motion Planning*. Kluwer Academic Publishers.

LaValle, S. M. (2006). *Planning Algorithms*. Cambridge University Press.

Lekkas, A. M., & Fossen, T. I. (2014). Integral LOS path following for curved paths based on a monotone cubic Hermite spline parametrization. *IEEE Transactions on Control Systems Technology*, 22(6), 2287–2301. https://doi.org/10.1109/TCST.2014.2306774

NOAA CO-OPS. (2024). *Station pm0101 — Portsmouth Harbor, Gangway Rocks LB13: current observations*. National Oceanic and Atmospheric Administration, Center for Operational Oceanographic Products and Services. https://tidesandcurrents.noaa.gov/cdata/StationInfo?id=pm0101

Oberkampf, W. L., & Roy, C. J. (2010). *Verification and Validation in Scientific Computing*. Cambridge University Press. https://doi.org/10.1017/CBO9780511760396

Park, S. (2022). Three-dimensional Dubins-path-guided continuous curvature path smoothing. *Applied Sciences*, 12(22), 11336. https://doi.org/10.3390/app122211336

Paull, L., Saeedi, S., Seto, M., & Li, H. (2014). AUV navigation and localization: A review. *IEEE Journal of Oceanic Engineering*, 39(1), 131–149. https://doi.org/10.1109/JOE.2013.2278891

Peng, R. D. (2011). Reproducible research in computational science. *Science*, 334(6060), 1226–1227. https://doi.org/10.1126/science.1213847

Quigley, M., Gerkey, B., Conley, K., Faust, J., Foote, T., Leibs, J., Wheeler, R., & Ng, A. Y. (2009). ROS: An open-source Robot Operating System. In *ICRA Workshop on Open Source Software*.

Sahoo, A., Dwivedy, S. K., & Robi, P. S. (2019a). Advancements in the field of autonomous underwater vehicle. *Ocean Engineering*, 181, 145–160. https://doi.org/10.1016/j.oceaneng.2019.04.011

Sahoo, A., Dwivedy, S. K., & Robi, P. S. (2019b). Development of a PID control strategy for a compact autonomous underwater vehicle. In *Proceedings of the ASME 2019 38th International Conference on Ocean, Offshore and Arctic Engineering (OMAE 2019)*. https://doi.org/10.1115/OMAE2019-95345

Sandve, G. K., Nekrutenko, A., Taylor, J., & Hovig, E. (2013). Ten simple rules for reproducible computational research. *PLoS Computational Biology*, 9(10), e1003285. https://doi.org/10.1371/journal.pcbi.1003285

Sargent, R. G. (2013). Verification and validation of simulation models. *Journal of Simulation*, 7(1), 12–24. https://doi.org/10.1057/jos.2012.20

Shen, C., & Shi, Y. (2020). Distributed implementation of nonlinear model predictive control for AUV trajectory tracking. *Automatica*, 115, 108863. https://doi.org/10.1016/j.automatica.2020.108863

Tarbouriech, S., & Turner, M. (2009). Anti-windup design: An overview of some recent advances and open problems. *IET Control Theory & Applications*, 3(1), 1–19. https://doi.org/10.1049/iet-cta:20070435

Tiboni, G., Arndt, K., & Kyrki, V. (2023). DROPO: Sim-to-real transfer with offline domain randomization. *Robotics and Autonomous Systems*, 166, 104432. https://doi.org/10.1016/j.robot.2023.104432

Wang, H.-J., Bian, X.-Q., Zhang, X., Fu, M.-Y., & Li, J. (2004). Two approaches for autonomous underwater vehicle global path planning in large range ocean environment. In *Proceedings of the International Conference on Intelligent Mechatronics and Automation (ICIMA 2004)*. https://doi.org/10.1109/ICIMA.2004.1384193

Wang, Y., & Du, Z. (2024). Trajectory tracking control for an underactuated AUV via nonsingular fast terminal sliding mode approach. *Journal of Marine Science and Engineering*, 12(8), 1442. https://doi.org/10.3390/jmse12081442

Wynn, R. B., Huvenne, V. A. I., Le Bas, T. P., Murton, B. J., Connelly, D. P., Bett, B. J., Ruhl, H. A., Morris, K. J., Peakall, J., Parsons, D. R., Sumner, E. J., Darby, S. E., Dorrell, R. M., & Hunt, J. E. (2014). Autonomous Underwater Vehicles (AUVs): Their past, present and future contributions to the advancement of marine geoscience. *Marine Geology*, 352, 451–468. https://doi.org/10.1016/j.margeo.2014.03.012

Yang, K., & Sukkarieh, S. (2010). An analytical continuous-curvature path-smoothing algorithm. *IEEE Transactions on Robotics*, 26(3), 561–568. https://doi.org/10.1109/TRO.2010.2042990

Zhao, W., Queralta, J. P., & Westerlund, T. (2020). Sim-to-real transfer in deep reinforcement learning for robotics: A survey. In *Proceedings of the IEEE Symposium Series on Computational Intelligence (SSCI 2020)*. https://doi.org/10.1109/SSCI47803.2020.9308468
