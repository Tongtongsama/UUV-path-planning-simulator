# S4 planning improvement demonstration

This is a dissertation figure and table package generated **only from existing records**. Run `..\.\.venv\Scripts\python.exe -B build.py` from the repository root to regenerate. `input_sha256.json` lists all 27 source files used. The source experiments and frozen code were read only; no simulation was started.

## Recommended placement

Place the S4 scene and planning workflow after the baseline A* and G12 navigation methods. Place the comparison in a distinct planning demonstration subsection of Results, after the formal S1–S3 real matrix. The S4 material supplements, rather than changes, the formal 53/54 real-matrix result.

| Figure | Proposed caption and claim | Scope |
|---|---|---|
| [01 geometry](01_s4_geometry_paths.svg) | S4 five-obstacle wide-channel map and four F0/P0 references. A is raw A*; B applies the existing safe shortcut; C adds safe smoothing; D retains B and appends a 4 m level goal approach. All have at least 1.5 m planned clearance. | Geometry only; same P0 map. The five obstacles do not imply five separate avoidance turns. |
| [02 F0 Python loop](02_s4_f0_python_closed_loop.svg) | Actual Python 3-DoF trajectories and original path-error samples under frozen G12, F0/P0. B improves arrival and tracking relative to A; C is geometrically smoother but does not improve tracking over B. | Four schemes, one flow and initial state, different reference paths; errors are relative to each scheme's own path. |
| [03 F2 terminal comparison](03_s4_f2_terminal_failure_comparison.svg) | The original B path fails F2/P0 at the goal approach; D's 4 m level approach passes under the same frozen controller and scene. Cross marks B's final recorded position; green circle marks D's final recorded position. | Python reference dynamics. Failure is at terminal holding/right boundary, not an intermediate obstacle collision. The old failed run is retained. |
| [04 Python versus real](04_s4_terminal_python_real_overlay.svg) | D's three P0 Python and GUI xdyn actual trajectories, drawn separately by flow. | Visual comparison, not a numerical equivalence claim. The xdyn runs have their own original acceptance records. |

The SVG files are vector masters; PNGs are previews. All map panels use equal x/z scale and the original obstacle dimensions. The CSVs [geometry and F0 Python](geometry_and_f0_python.csv), [F2 terminal comparison](terminal_f2_comparison.csv), and [five GUI pilots](real_five_pilots.csv) supply editable tables.

## Results that can be stated in the manuscript

For S4/F0/P0 Python, A/B/C/D path lengths were 18.485/17.207/17.073/17.627 m. B reduced arrival time from A's 132.90 to 52.45 s and RMS path error from 0.1950 to 0.0837 m. C reduced geometric maximum discrete turn to 1.402°, but increased RMS to 0.1716 m and arrival time to 59.10 s versus B. D's level terminal segment increases length relative to B and changes the end geometry; it is a separate terminal candidate, not a strict improvement on every F0 metric.

B's expanded Python validation passed F0/F1 but failed all six F2 entries, including P0 and its repeat. D's independent Python validation passed 18/18 with three strict P0 repetitions. Five selected GUI xdyn D pilots passed all original checks: F0/P0, F1/P0, F2/P0, F2/P2 and F2/P4. The narrowest real maximum-error margin was 0.068573 m at F2/P2. These five pilots are **not** a full 18-case S4 real matrix. Different initial states were replanned by the same rule, rather than perturbed against a single fixed P0 path.

## Claim boundaries and citations

The planning contribution is the **integration and evaluation** of existing grid A*, safe shortcutting, optional smoothing and a preregistered level terminal segment with frozen G12 feedback. Do not describe A*, shortcutting or Bézier rounding as newly invented algorithms. Single-run wall-clock planning timings do not support a general speed claim. The plots do not establish optimality across maps, currents or unknown obstacles. Cite algorithm origins and LOTUSim/xdyn platform sources separately when writing the literature and methods sections; no bibliographic citation has been fabricated here.

The formal dissertation matrix remains 53/54 real trials, with nine group acceptance/repeatability results. The five GUI reproduction trials and strong-current real gate rejection retain their separate denominators. The S4 figures and tables do not replace any historical failed trial or gate refusal.
