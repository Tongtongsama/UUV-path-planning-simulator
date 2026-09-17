# Declared-dependency environment verification

Date: 2026-09-13. Status: passed.

## Resolution

The global Python 3.10 environment contained pytest 9.1.1, outside the project's
`pytest>=7.0,<9.0` declaration. The declaration is retained unchanged.
An isolated project `.venv` was created without global site packages, then
installed directly from `requirements-validation.txt` and
`requirements-visualization.txt`. These include the development and runtime
requirements. No global package was downgraded or removed.

The project environment uses Python 3.10.9, pytest 8.4.2, NumPy 2.2.6,
PyYAML 6.0.3, Shapely 2.1.2, SciPy 1.15.3 and Matplotlib 3.10.9.

## Verification

```powershell
.\.venv\Scripts\python.exe -m pip check
.\.venv\Scripts\python.exe -m pytest tests -q -rs -p no:cacheprovider --junitxml=artifacts/dependency_audit/pytest8_20260913/junit.xml
```

- pip check: `No broken requirements found.`
- Full test suite: **589 passed in 18.90s; zero failures, errors or skips**.
- The previously skipped SciPy reference/convergence test now executes and passes.
- Pytest-produced machine-readable results: `artifacts/dependency_audit/pytest8_20260913/junit.xml`.
- Exact installed package snapshot: `artifacts/dependency_audit/pytest8_20260913/installed.txt`.

This verifies the resolved versions, not every version permitted by the broad
requirements ranges. The snapshot is evidence for this environment, not a new
cross-platform lockfile. The 16-case Navigation acceptance runner was not rerun
for this dependency-only task; its previous artifacts remain unchanged.

## Use the correct interpreter

Use `.venv\Scripts\python.exe` in the IDE and explicit commands above, or activate
the environment before running generic `python -m pytest` commands. In an
unactivated terminal, bare Python/pytest may still use global pytest 9.1.1.
Explicit paths require no PowerShell execution-policy changes.

`.venv/` and `artifacts/` are already excluded from Git. Recreate the environment
from requirements on another computer; do not copy or commit the virtual environment.
