# Package Information — PICO × AgileX Nero Teleop Host (Base Mounting Edition)

- Package: `analytic_control_base_mounting_20261110`
- Date: 2026-11-10
- Baseline: public upstream `main` (`c23287b`, Name the joint that owns the tightest limit margin)
- Added feature: power-port / base-mounting facing presets (Front / Down / Custom), cross-page settings transaction handling, timeout cancellation, persistence-failure recovery, and the Start gate.
- Deliberately excluded: local settings, logs, virtual environments, and build artifacts.
- Structure: `analytic_control/` is the project root. `README.md` is English; `docs/screenshots/` contains the UI screenshots referenced by the README. Chinese reference copies are kept outside the delivery package and are not part of the repository.
- Build first: install `requirements.txt`, install `third_party/pyAgxArm`, then run `python setup.py build_ext --inplace` (the package intentionally does not ship generated `.so`).
- Offline verification: the source in this package was tested in a pure-simulation environment on 2026-11-10 with a locally built IK extension: `147 passed` (`python -m pytest -q -p no:cacheprovider`). This is not a real-arm, load, safety, or precision acceptance.
- The physical mounting axis is still an assumption to be checked against the real hardware.
