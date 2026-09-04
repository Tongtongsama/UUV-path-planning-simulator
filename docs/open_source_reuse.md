# Open-Source Reuse and Research Provenance Register

**Status:** Active decision and provenance register  
**Initial audit date:** 2026-09-03  
**Scope:** UUV Simulator, with emphasis on Physics v0.4

## 1. Purpose

This register records which external software, equations, implementations,
parameters, and data sources are adopted, consulted, deferred, or rejected by
the project. It exists to make technical decisions reproducible and to keep
license and attribution obligations auditable.

The project does not integrate an entire third-party UUV simulator. External
work is classified and recorded according to how it is actually used:

| Reuse category | Meaning |
|---|---|
| Direct dependency | Installed and called by project code |
| Validation dependency | Installed and called only by tests or offline validation |
| Adapted implementation | External code is copied or modified; attribution and license review are mandatory |
| Reference implementation | Code is read or executed for comparison, but no code is copied |
| Theoretical reference | Equations or modelling guidance are taken from a publication |
| Parameter/data source | Numerical values or datasets are adopted with units and conversion records |
| Future backend | Candidate for a later replaceable backend, not part of the current runtime |
| Rejected/deferred | Evaluated but deliberately not integrated in the current milestone |

The mathematical and software contracts implemented by this project are
defined in [`physics_v0.4_specification.md`](physics_v0.4_specification.md).
This register MUST NOT redefine those interfaces or equations. Numerical
verification evidence will be reported separately in
`physics_v0.4_validation.md`.

## 2. Register rules

1. A branch name such as `master` or `main` is not a reproducible version.
   Before external code, parameters, or generated reference outputs are used,
   the exact release and/or commit hash MUST be recorded here.
2. Reading a source does not imply copying its code. `Code copied or adapted`
   MUST remain `None` until a concrete project file contains derived code.
3. Equations, code, parameters, and datasets have different provenance and
   licensing implications and MUST be recorded separately.
4. Any copied or adapted implementation MUST record the upstream file,
   function or line-level region, copyright notice, license, local destination,
   and modifications.
5. External parameter sets MUST record original symbols, units, sign
   conventions, conversions, and whether the complete model or only a subset
   was adopted.
6. A vehicle configuration MUST NOT use a real vehicle's name unless its
   geometry, inertia, hydrodynamic parameters, and actuation assumptions form a
   defensible representation of that vehicle.
7. Remote references used to generate validation results SHOULD be archived as
   a release, commit-pinned checkout, checksum, or versioned local result so the
   experiment can be repeated offline.
8. Dependency versions used for a dissertation release MUST be frozen in the
   release environment, even when development requirements use compatible
   version ranges.
9. Every detailed source record MUST include an `Evidence location` and a
   `Last verified` date. A live URL is acceptable during initial assessment,
   but adopted code, parameters, and generated reference outputs require a
   commit-pinned checkout, archived artifact, or checksum before release.
10. Academic sources MUST point to the dissertation citation key and BibTeX
    location once the project bibliography exists. `TBD` is permitted while a
    source remains unpinned or the bibliography has not yet been created.

## 3. Current decision summary

| Source | Category | Physics v0.4 role | Current decision |
|---|---|---|---|
| NumPy | Direct dependency | Arrays, matrices, linear solves | Adopted |
| SciPy | Validation dependency | `solve_ivp` reference solution | Adopted for validation; not yet installed in audited environment |
| Fossen marine-craft framework | Theoretical reference | Equation structure and notation | Adopted as theoretical framework |
| Python Vehicle Simulator | Reference implementation | Equation review and numerical comparison | Adopt as reference; no runtime dependency |
| Marine Systems Simulator (MSS) | Reference implementation | Optional MATLAB/Octave comparison | Optional reference |
| Gazebo Harmonic Hydrodynamics | Future backend | Later high-fidelity validation | Deferred beyond v0.4 |
| SDFormat fluid added mass | Future parameter adapter | Canonical-to-SDF parameter mapping | Deferred beyond v0.4 |
| UUV Simulator | Historical reference | Legacy plugin and vehicle-configuration review | Do not integrate |
| Plankton | Historical ROS 2 reference | Historical ROS 2 port review | Do not integrate |
| Capytaine | Future parameter estimation | Mesh-based hydrodynamic coefficients | Outside v0.4 |

As of the initial audit date:

- no source code from Python Vehicle Simulator, MSS, Gazebo, UUV Simulator,
  Plankton, or Capytaine has been copied or adapted into this repository;
- no vehicle parameter set from those projects has been adopted;
- no third-party reference trajectory has yet been generated;
- Physics production code has not yet been implemented;
- the first planned vehicle configuration is synthetic and test-only.

## 4. Detailed source records

### 4.1 NumPy

**Name:** NumPy  
**Project:** [numpy/numpy](https://github.com/numpy/numpy)  
**Website:** [numpy.org](https://numpy.org/)  
**Evidence location:** Upstream project and website links above; no local
source archive required for current public-API use  
**Last verified:** 2026-09-03  
**Version or commit:** Development environment currently reports `2.2.6`;
release version not yet frozen  
**License:** BSD 3-Clause  
**Project role:** Vector/matrix representation, trigonometry, finite-value
checks, matrix validation, and `numpy.linalg.solve`  
**Reuse category:** Direct dependency  
**Files/functions inspected:** Normal public API documentation; no upstream
source file copied  
**Code copied or adapted:** None  
**Parameters adopted:** None  
**Modifications:** None  
**Reason for adoption:** Mature numerical-array API already used by Core and
Environment; sufficient for the fixed-step reference backend  
**Known limitations:** Floating-point results depend on dtype, BLAS/LAPACK, and
platform; release experiments require a frozen environment  
**Required attribution:** Preserve NumPy's license in any distribution process
that requires bundled third-party notices  
**Validation performed:** NumPy `2.2.6` import and existing project tests pass
in the audited Windows Python environment  
**Decision status:** Adopted

### 4.2 SciPy

**Name:** SciPy  
**Project:** [scipy/scipy](https://github.com/scipy/scipy)  
**Website:** [scipy.org](https://scipy.org/)  
**Evidence location:** Upstream project and website links above; installed
distribution and validation artifact still pending  
**Last verified:** 2026-09-03  
**Version or commit:** SciPy `1.11.1` used in the offline Anaconda validation
environment; listed in `requirements.txt`; dissertation release version not
yet frozen  
**License:** BSD 3-Clause  
**Project role:** Offline high-accuracy integration reference through
`scipy.integrate.solve_ivp`  
**Reuse category:** Validation dependency  
**Files/functions inspected:** `solve_ivp` public API documentation only;
upstream implementation not yet inspected  
**Code copied or adapted:** None  
**Parameters adopted:** None  
**Modifications:** None  
**Reason for adoption:** Independent adaptive integration reference for Euler
and RK4 convergence checks  
**Known limitations:** It is not the real-time/default integrator and must not
enter the Physics runtime path; tolerances and solver method must be recorded
with every validation result  
**Required attribution:** Preserve applicable SciPy notices when redistributed  
**Validation performed:** `solve_ivp`/DOP853 comparison completed for damped
free response, constant thrust, and constant world current with pitch motion;
see `physics_v0.4_validation.md`  
**Decision status:** Adopted and exercised as an offline validation dependency

### 4.3 Fossen marine-craft modelling framework

**Name:** Fossen marine-craft equations and SNAME notation  
**Primary publication:** T. I. Fossen, *Handbook of Marine Craft Hydrodynamics
and Motion Control*, 2nd edition, Wiley, 2021  
**Companion material:** [Fossen's Marine Craft Model](https://www.fossen.biz/html/marineCraftModel.html)  
**Evidence location:** Publication and companion-material link above; approved
reduction stored in `docs/physics_3dof_derivation.md`  
**Last verified:** 2026-09-03  
**Dissertation citation key:** `fossen2021handbook` (planned)  
**BibTeX location:** `docs/references.bib` (planned; file not yet created)  
**Version or edition:** Second edition, 2021  
**License:** Publication copyright applies; mathematical concepts are cited,
not treated as software code  
**Project role:** Theoretical structure for inertia, Coriolis/centripetal,
damping, restoring, relative velocity, and marine notation  
**Reuse category:** Theoretical reference  
**Sections/equations inspected:** General 6-DOF marine-craft equation,
relative-current equation, rigid-body Coriolis parametrization, and added-mass
Coriolis construction  
**Code copied or adapted:** None  
**Parameters adopted:** None  
**Modifications:** Project-specific ENU world frame, vertical-plane reduction,
diagonal-matrix assumptions, and nose-up pitch convention must be derived and
documented rather than attributed directly to the general equation  
**Reason for adoption:** Established marine-control framework aligned with the
project's SNAME body variables  
**Known limitations:** The general framework does not by itself validate this
project's reduced signs, origin assumptions, or Coriolis matrices  
**Required attribution:** Cite the book and any specific companion material in
the dissertation and derivation note  
**Validation performed:** Kinematic/current convention and independently
reduced Coriolis matrices reviewed; skew-symmetry, zero-power, scalar dynamics,
and world-current correction covered by project tests  
**Decision status:** Adopted theoretical framework

### 4.4 Python Vehicle Simulator

**Name:** Python Vehicle Simulator  
**Repository:** [cybergalactic/PythonVehicleSimulator](https://github.com/cybergalactic/PythonVehicleSimulator)  
**Evidence location:** [Pinned GitHub tree](https://github.com/cybergalactic/PythonVehicleSimulator/tree/6073346c86180087d3bca608b1aa2bc08be59708);
temporary read-only checkout used for the initial audit  
**Last verified:** 2026-09-03  
**Dissertation citation key:** TBD when the inspected commit and appropriate
academic source are selected  
**BibTeX location:** `docs/references.bib` (planned; file not yet created)  
**Version or commit:** `6073346c86180087d3bca608b1aa2bc08be59708`;
repository metadata reports package version `1.0.1`  
**License:** MIT  
**Project role:** Equation review, GNC/frame utility comparison, REMUS model
review, and independent numerical reference cases  
**Reuse category:** Reference implementation  
**Files/functions inspected:**
`src/python_vehicle_simulator/lib/gnc.py::m2c` and
`src/python_vehicle_simulator/vehicles/remus100.py` mass-matrix construction
and Coriolis call sites  
**Code copied or adapted:** None  
**Parameters adopted:** None  
**Modifications:** None  
**Reason for adoption:** Pure-Python implementation associated with the Fossen
framework and compatible marine notation; useful for transparent comparison  
**Known limitations:** Its vehicle, actuator, guidance, control, and simulation
responsibilities do not match this project's module boundaries; an equivalent
comparison case may require explicit frame, sign, parameter, and input mapping  
**Required attribution:** If code is later adapted, preserve the MIT copyright
and permission notice and record the exact source/local mapping. Reference-only
use still requires normal academic citation  
**Validation performed:** Independently reduced the 6-DOF diagonal-mass
construction to surge-heave-pitch; checked matrix entries, skew symmetry, and
zero quadratic power in project tests. See `physics_3dof_derivation.md`  
**Decision status:** Use as reference only; do not add as runtime dependency or
inherit its simulator architecture

Before any code inspection is used as evidence, record:

```text
Pinned commit: 6073346c86180087d3bca608b1aa2bc08be59708
Checkout/archive checksum: TBD
Files inspected: src/python_vehicle_simulator/lib/gnc.py;
  src/python_vehicle_simulator/vehicles/remus100.py
Functions inspected: gnc.py::m2c; REMUS mass construction and Coriolis call sites
Reference cases generated: TBD
```

### 4.5 Marine Systems Simulator (MSS)

**Name:** Marine Systems Simulator (MSS)  
**Repository:** [cybergalactic/MSS](https://github.com/cybergalactic/MSS)  
**Evidence location:** Upstream repository link above; commit-pinned checkout
or archive not yet created  
**Last verified:** 2026-09-03  
**Dissertation citation key:** TBD; expected to cover the MSS publication and
any specific model source actually used  
**BibTeX location:** `docs/references.bib` (planned; file not yet created)  
**Version or commit:** **Not yet pinned**  
**License:** MIT for the repository; individual external tools or data consumed
by some MSS workflows may have separate terms  
**Project role:** Optional MATLAB/GNU Octave theoretical and numerical
comparison  
**Reuse category:** Reference implementation  
**Files/functions inspected:** None recorded at specification stage  
**Code copied or adapted:** None  
**Parameters adopted:** None  
**Modifications:** None  
**Reason for adoption:** Longstanding companion implementation for the Fossen
framework with marine craft models and GNC utilities  
**Known limitations:** MATLAB/Simulink is not a project runtime dependency;
some hydrodynamic workflows rely on separately licensed tools; its full 6-DOF
models do not automatically validate this project's 3-DOF reduction  
**Required attribution:** Repository requests citation of T. I. Fossen and
T. Perez (2004), Marine Systems Simulator, in addition to applicable license
notices if code is reused  
**Validation performed:** None yet  
**Decision status:** Optional reference; no integration planned

### 4.6 Gazebo Harmonic Hydrodynamics

**Name:** Gazebo Harmonic / Gazebo Sim Hydrodynamics system  
**Documentation baseline:** [Gazebo Sim 8 Hydrodynamics](https://gazebosim.org/api/sim/8/classgz_1_1sim_1_1systems_1_1Hydrodynamics.html)  
**Harmonic version map:** [Gazebo Harmonic libraries](https://gazebosim.org/docs/harmonic/install/)  
**ROS pairing:** [Installing Gazebo with ROS](https://gazebosim.org/docs/jetty/ros_installation/)  
**Evidence location:** Official Gazebo documentation links above; no local
installation, checkout, or archive yet  
**Last verified:** 2026-09-03  
**Version family:** Gazebo Harmonic / `gz-sim8`  
**Exact package version:** Not yet frozen  
**License:** Gazebo component licenses and bundled notices must be audited at
integration time; Gazebo Sim is generally distributed under Apache License 2.0  
**Project role:** Future high-fidelity backend and cross-validation target  
**Reuse category:** Future backend  
**Files/functions inspected:** Public hydrodynamics and installation
documentation only  
**Code copied or adapted:** None  
**Parameters adopted:** None  
**Modifications:** None  
**Reason for adoption:** ROS 2 Jazzy's recommended Gazebo pairing is Harmonic;
Gazebo provides rigid-body simulation and hydrodynamic systems without making
the Python reference backend depend on its runtime  
**Known limitations:** Backend behaviour depends on physics engine, SDF
configuration, plugin/native added-mass selection, current handling, time step,
and bridge configuration. DART native added mass and legacy plugin added mass
must not be enabled simultaneously  
**Required attribution:** Record exact Gazebo, gz-sim, physics-engine, and
plugin package licenses/versions when the backend is implemented  
**Validation performed:** Documentation review only  
**Decision status:** Deferred until the Python 3-DOF backend is stable

### 4.7 SDFormat fluid added mass

**Name:** SDFormat `<fluid_added_mass>`  
**Specification:** [SDFormat 1.12 link/inertial specification](https://sdformat.org/spec/1.12/link/)  
**Evidence location:** Official SDFormat specification link above; no local
schema archive or generated SDF artifact yet  
**Last verified:** 2026-09-03  
**Version or commit:** Target schema currently identified as SDFormat 1.12;
final Gazebo integration schema/version not frozen  
**License:** SDFormat implementation/specification terms must be checked at
adapter implementation time  
**Project role:** Future adapter from canonical project parameters to Gazebo
link inertial configuration  
**Reuse category:** Future parameter adapter  
**Files/functions inspected:** Public link/inertial specification only  
**Code copied or adapted:** None  
**Parameters adopted:** None  
**Modifications:** None  
**Reason for adoption:** Native representation separates project canonical
parameters from legacy plugin derivative names  
**Known limitations:** Added-mass sign and matrix conventions must be converted
explicitly. Native and legacy added mass must not be configured together  
**Required attribution:** Record schema, library, and generated-file notices as
required when the adapter is introduced  
**Validation performed:** Documentation review only  
**Decision status:** Deferred beyond Physics v0.4

### 4.8 UUV Simulator

**Name:** UUV Simulator  
**Repository:** [uuvsimulator/uuv_simulator](https://github.com/uuvsimulator/uuv_simulator)  
**Evidence location:** Archived upstream repository link above; no
commit-pinned checkout or local archive yet  
**Last verified:** 2026-09-03  
**Dissertation citation key:** TBD when a specific publication and inspected
repository commit are selected  
**BibTeX location:** `docs/references.bib` (planned; file not yet created)  
**Version or commit:** Repository archived on 2023-02-06; no commit pinned  
**License:** Apache License 2.0, with additional third-party notices in the
upstream repository  
**Project role:** Historical review of Gazebo underwater plugins, vehicle
configurations, and prior architecture  
**Reuse category:** Rejected/deferred historical reference  
**Files/functions inspected:** None recorded for Physics v0.4  
**Code copied or adapted:** None  
**Parameters adopted:** None  
**Modifications:** None  
**Reason considered:** Important prior underwater Gazebo project with
hydrodynamics, thruster, sensor, and vehicle examples  
**Reason not adopted:** Archived ROS 1 / Gazebo-classic architecture conflicts
with the project's ROS 2 Jazzy / Gazebo Harmonic target and clean module
boundaries  
**Known limitations:** Upstream itself describes the project as a research
prototype; old plugin conventions and dependencies require careful conversion  
**Required attribution:** Apache-2.0 `LICENSE`/`NOTICE` obligations apply if any
material is later adapted; academic use should cite the upstream publication  
**Validation performed:** Repository-level review only  
**Decision status:** Do not integrate; historical reference only

### 4.9 Plankton

**Name:** Plankton  
**Repository:** [Liquid-ai/Plankton](https://github.com/Liquid-ai/Plankton)  
**Evidence location:** Upstream repository link above; no commit-pinned
checkout or local archive yet  
**Last verified:** 2026-09-03  
**Version or commit:** **Not yet pinned**  
**License:** Apache License 2.0  
**Project role:** Historical ROS 2 port and architecture reference  
**Reuse category:** Rejected/deferred historical reference  
**Files/functions inspected:** Repository overview only  
**Code copied or adapted:** None  
**Parameters adopted:** None  
**Modifications:** None  
**Reason considered:** Demonstrates a ROS 2 maritime simulator derived from UUV
Simulator  
**Reason not adopted:** Targets older ROS 2/Gazebo combinations and inherits
substantial UUV Simulator architecture; it is not the chosen Jazzy/Harmonic
backend  
**Known limitations:** Historical compatibility does not establish current
Harmonic behaviour or validate this project's 3-DOF equations  
**Required attribution:** Apache-2.0 obligations apply if material is later
adapted  
**Validation performed:** Repository-level review only  
**Decision status:** Do not integrate; historical reference only

### 4.10 Capytaine

**Name:** Capytaine  
**Repository:** [capytaine/capytaine](https://github.com/capytaine/capytaine)  
**Documentation:** [Capytaine documentation](https://capytaine.github.io/)  
**Evidence location:** Upstream repository and documentation links above; no
release archive, geometry, or generated coefficient artifact yet  
**Last verified:** 2026-09-03  
**Version or commit:** No version pinned; upstream states version 3 and later
use Apache License 2.0  
**License:** Apache License 2.0 for version 3 and later; older-version and
bundled-component terms require separate verification  
**Project role:** Possible future mesh-based potential-flow/BEM coefficient
estimation  
**Reuse category:** Rejected/deferred beyond v0.4  
**Files/functions inspected:** Repository overview only  
**Code copied or adapted:** None  
**Parameters adopted:** None  
**Modifications:** None  
**Reason considered:** Can calculate frequency-domain hydrodynamic quantities
for floating bodies from mesh geometry  
**Reason not adopted:** Requires validated vehicle geometry and a higher-
fidelity hydrodynamic scope; unnecessary for the synthetic 3-DOF reference
model  
**Known limitations:** Potential-flow/BEM assumptions and frequency-domain
outputs do not directly supply a validated complete nonlinear UUV model  
**Required attribution:** Audit the exact Capytaine release and its notices if
future generated coefficients are used  
**Validation performed:** None  
**Decision status:** Outside Physics v0.4

## 5. Parameter provenance register

The first Physics configuration is synthetic and MUST be named
`uuv_3dof_synthetic_v1`. Values are chosen for software and mathematical
validation, not to represent REMUS 100 or another real vehicle.

Physics v0.4 accepts only diagonal rigid-body mass, added-mass, and damping
matrices. Provenance for off-diagonal parameters is outside the current
milestone; its absence from this table is intentional.

| Project parameter | Value | Unit | Source | Original symbol | Conversion | Confidence/use |
|---|---:|---|---|---|---|---|
| `mass_rb[0,0]` | 20.0 | kg | Synthetic v0.4 | N/A | None | Test-only |
| `mass_rb[1,1]` | 20.0 | kg | Synthetic v0.4 | N/A | None | Test-only |
| `mass_rb[2,2]` | 8.0 | kg m^2 | Synthetic v0.4 | N/A | None | Test-only |
| `mass_added[0,0]` | 5.0 | kg | Synthetic v0.4 | N/A | Stored positive | Test-only |
| `mass_added[1,1]` | 8.0 | kg | Synthetic v0.4 | N/A | Stored positive | Test-only |
| `mass_added[2,2]` | 2.0 | kg m^2 | Synthetic v0.4 | N/A | Stored positive | Test-only |
| surge linear damping | 4.0 | N s/m | Synthetic v0.4 | N/A | Stored positive | Test-only |
| heave linear damping | 6.0 | N s/m | Synthetic v0.4 | N/A | Stored positive | Test-only |
| pitch linear damping | 3.0 | N m s/rad | Synthetic v0.4 | N/A | Stored positive | Test-only |
| surge quadratic damping | 1.0 | N s^2/m^2 | Synthetic v0.4 | N/A | Stored positive | Test-only |
| heave quadratic damping | 1.5 | N s^2/m^2 | Synthetic v0.4 | N/A | Stored positive | Test-only |
| pitch quadratic damping | 0.5 | N m s^2/rad^2 | Synthetic v0.4 | N/A | Stored positive | Test-only |
| `restoring_pitch_coefficient` | 6.0 | N m | Synthetic v0.4 | N/A | Reduced coefficient under the specification's neutral-buoyancy, CG-origin, and vertical CG/CB-separation assumptions; not an identified vehicle parameter | Test-only |
| `input_matrix` | `diag(1,0,1)` | dimensionless | Physics v0.4 specification | N/A | Admissible generalized-force subspace; non-zero `tau_z` is rejected by the baseline configuration | Dissertation baseline assumption |

When a literature/open-source parameter is proposed, add a new versioned
configuration rather than overwriting this table. Record each adopted value,
including sign conversion from hydrodynamic derivatives to the project's
positive canonical mass/damping convention.

## 6. Adapted-code register

No adapted third-party code is currently present.

Before an adapted implementation is committed, add a row:

| Local file/function | Upstream source | Commit | License | Copyright notice | Nature of adaptation | Tests |
|---|---|---|---|---|---|---|
| _None_ | | | | | | |

The commit containing adapted code MUST also contain or reference the required
license/notice material. Superficial renaming or translation does not remove
the obligation to record derivation from upstream code.

## 7. Reference-output register

The following third-party-assisted numerical reference output has been
generated. SciPy provides integration only; the evaluated dynamics equations
remain the project's implementation.

Future comparison artifacts MUST record:

| Case ID | Source | Commit/version | Source model/config | Project config | Initial state/input/current | Output artifact/checksum | Comparison metric |
|---|---|---|---|---|---|---|---|
| `physics_v0.4_convergence_v1` | SciPy `solve_ivp` / DOP853 | SciPy 1.11.1 | `uuv_3dof_synthetic_v1` dynamics | `uuv_3dof_synthetic_v1` | Three cases recorded in artifact; `rtol=1e-11`, `atol=1e-13` | `validation/reference_cases/physics_v0.4_results.json`; SHA-256 `c17b22b5a3d7c156a759d6cc4158cc7ecb5acba03eb8f60ead019e91fc703662` | Final-state L2 error and observed convergence order |

Expected initial cases are free decay, constant surge step, pitch-moment step,
and constant-current response. Equivalent conditions and all frame/sign
conversions must be documented; visual similarity alone is not validation.

## 8. Adoption workflow

Before changing any source from reference/deferred to adapted/dependent status:

- [ ] Identify the exact technical need.
- [ ] Pin a release and commit hash.
- [ ] Archive or checksum the inspected source where appropriate.
- [ ] Verify the repository and relevant-file licenses.
- [ ] Record exact files/functions/equations/parameters inspected.
- [ ] Decide whether the use is reference, copied code, adapted code, or data.
- [ ] Record all unit, axis, frame, ordering, and sign conversions.
- [ ] Add required license and copyright notices.
- [ ] Add independent project tests rather than relying only on upstream tests.
- [ ] Record numerical comparison results in `physics_v0.4_validation.md`.
- [ ] Update the decision summary and detailed source record.

## 9. Current attribution and distribution state

At the initial audit stage, the runtime project directly uses ordinary Python
dependencies declared in `requirements.txt`; no third-party simulator source
or vehicle parameter file has been vendored into the repository.

Before a distributable dissertation release, produce a dependency lock and a
third-party notice inventory for the exact shipped environment. This register
supports that inventory but does not replace the complete license texts or
legal terms supplied by upstream projects.

## 10. Change log

| Date | Source/section | Change | Author/reviewer |
|---|---|---|---|
| 2026-09-03 | Initial register | Recorded source categories, initial decisions, empty adapted-code/reference-output registers, and synthetic parameter provenance plan | Project team |
| 2026-09-03 | Sections 2, 4, and 5 | Corrected the Harmonic baseline to `gz-sim8`; added evidence, verification, and citation-location fields; synchronized diagonal-matrix, restoring, and actuation semantics with the Physics specification | Project team |
| 2026-09-03 | Python Vehicle Simulator | Pinned commit `6073346c...`, recorded inspected paths, and linked the independently derived 3-DOF Coriolis evidence | Project team |
| 2026-09-03 | Parameter provenance | Recorded the concrete `uuv_3dof_synthetic_v1` smoke/validation parameters | Project team |
| 2026-09-03 | SciPy/reference output | Recorded DOP853 convergence validation and checksummed result artifact | Project team |
