# M1 A* acceptance and static figures

Run from the repository root with the same Python used for project tests:

```powershell
python -m pip install -r requirements-dev.txt -r requirements-visualization.txt
python -m validation.planner_acceptance --output artifacts/planner/m1_run_001
```

The output directory must be new and outside the source packages. Existing
evidence is never overwritten. Use another directory for a subsequent run.
`--skip-tests` creates a preview only and explicitly leaves automated_acceptance
false. `--repeats` defaults to two and must be at least two.

## Frozen inputs and checks

The source is config/scenarios/planner_acceptance_v1.json. The three cases are
detour, narrow_passage and unreachable. Expected results are SUCCESS, SUCCESS
and NO_PATH respectively. Their common resolution is 0.5 m, connectivity 8,
vehicle radius 0.2 m, safety margin 0.1 m and goal tolerance 0.25 m. The narrow
passage is 0.8 m wide, leaving 0.2 m of centre corridor after total clearance.
These deterministic A* scenarios do not use a random seed.

Each case runs repeatedly, comparing status, Path, length and expanded nodes.
Every successful result is revalidated against the original request using
continuous segment collision checks. Timing samples are recorded but excluded
from determinism checks. They are not a warmed-up performance benchmark.
Status mismatch, invalid success, nondeterminism or failed pytest causes a
nonzero CLI exit. A successful NO_PATH outcome is accepted only where expected.

## Figures and evidence

Each case produces PNG and SVG figures showing ENU axes, boundary and its
clearance inset, obstacles and inflation, free/blocked lattice nodes, start,
goal centre and tolerance disk. Successful paths are the unchanged A* output:
raw and accepted geometry are identical at this stage. Failure has no path.
Inflation drawings are visual approximations; Environment queries determine
collision validity. This renderer intentionally supports the rectangular
acceptance schema, not arbitrary imported WorldModel geometry.

The run directory includes config.json, results.json, summary.txt, pytest.log,
manifest.json and a source snapshot. Results record actual interpreter,
package versions, Git HEAD and dirty status, per-case path/validation and
repeat timing. Source and artifact hashes bind evidence to the uncommitted
code as well as committed files. The snapshot contains project Python source,
tests, configuration, pytest.ini and requirements. It is not a Git checkout.
Run reproduction commands from its source directory after installing declared
dependencies. Results record actual versions separately; dependency ranges
are not a fully locked environment.

Automated acceptance does not certify visual layout or a fresh-environment
release. Inspect all three PNGs before closing M1. A skipped optional SciPy
test must remain visible in the saved log. Static path acceptance provides no
claim of trajectory feasibility or executed navigation safety.

## First inspected run

Output: `artifacts/planner/m1_20260912`. Full pytest: 548 passed, 1 skipped;
the skipped test requires optional SciPy. Repeated results were identical
apart from timing:

| Case | Status | Length (m) | Expanded nodes |
| --- | --- | ---: | ---: |
| detour | SUCCESS | 10.071067811865477 | 145 |
| narrow_passage | SUCCESS | 8.0 | 26 |
| unreachable | NO_PATH | N/A | 171 |

All three PNGs were inspected: axes, legend, inflation, goal region and path
are readable without clipping. Unreachable contains no fabricated route.
The plotted clearance value is the configured required clearance (0.3 m),
not a measured minimum path clearance. Raw and accepted paths coincide because
there is no post-processing stage yet.

The run used Python 3.10.9 and Matplotlib 3.10.9. Its actual pytest 9.1.1 is
outside requirements-dev.txt's <9 bound. This is recorded evidence of the
current environment, not certification of declared-dependency installation.
The checkout is dirty; the source snapshot and hashes preserve the tested
implementation. Commit cleanup and a fresh declared-dependency reproduction
remain separate release tasks.
