# Local setup (Windows)

The original project is checked out at the repository root. Use the project
virtual environment from PowerShell in this directory:

```powershell
.\.venv\Scripts\python.exe -m pip check
.\.venv\Scripts\python.exe -m pytest tests -q
.\.venv\Scripts\python.exe -m simulation.planner_smoke_demo
.\.venv\Scripts\python.exe -m simulation.physics_smoke_demo
.\.venv\Scripts\python.exe -m simulation.control_tracking_demo
```

The virtual environment was created with Python 3.12.14. If it is missing or
must be rebuilt, install Python 3.10 or newer and run:

```powershell
python -m venv .venv
.\.venv\Scripts\python.exe -m pip install -r requirements-validation.txt -r requirements-visualization.txt
```

`main.py` in the original repository is empty. The runnable entry points are
the `simulation` and `validation` modules listed in `README.md`.

## LOTUSim reference

The separate upstream LOTUSim source checkout is at `external/LOTUSim`.
This Python simulator currently has no LOTUSim runtime imports or integration;
the checkout is available as a local reference and is excluded from this
repository's Git tracking. To refresh it, run `git -C external/LOTUSim pull`.

LOTUSim is installed separately in the `NixOS` WSL distribution, including its
Web UI. Open a PowerShell terminal and enter it with:

```powershell
wsl -d NixOS
```

Inside NixOS, start the simulation and UI in separate terminals:

```sh
lotusim run --gui
lotusim-ui
```

The UI is served at `http://localhost:8080` in a Windows browser. For a
headless simulation, use `lotusim run`. The NixOS configuration for the LOTUSim
binary caches, graphics, Git, and the WSL mount guard is in
`/etc/nixos/lotusim.nix`. The downloaded NixOS image and a copy of that
configuration are in `C:\Users\Songyuan\Downloads\NixOS-WSL`.

The separate LOTUSim source is available at `external/LOTUSim`; its EPL-2.0
license is in `external/LOTUSim/LICENSE`.
