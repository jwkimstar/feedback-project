# Development Log

This file tracks repository changes by session date so package refactors and experiment tooling changes stay easy to review.

## 2026-03-21

### Summary

Worked toward two immediate project goals:

1. Create a functional interface between Python and X-Plane that can reliably discover the simulator, ingest telemetry, display it, record it, and plot it live before we dig into desgining and analyzing PID controller.
2. Create a modularized development template so future controller development and performance analysis can be added without collapsing back into a single-script prototype.

To support those goals, the project architecture was recreated around a single `python_client` package under `src/`, the previous standalone client script was replaced with package modules, and callable entry points were added for telemetry streaming, session recording, real-time plotting, and running all three together.

### Added

- `src/python_client/__init__.py`
- `src/python_client/__main__.py`
- `src/python_client/console.py`
- `src/python_client/config.py`
- `src/python_client/models.py`
- `src/python_client/main.py`
- `src/python_client/cli/__init__.py`
- `src/python_client/cli/run_client.py`
- `src/python_client/cli/plot_live.py`
- `src/python_client/cli/record_session.py`
- `src/python_client/cli/run_all.py`
- `src/python_client/xplane/__init__.py`
- `src/python_client/xplane/exceptions.py`
- `src/python_client/xplane/beacon.py`
- `src/python_client/xplane/packets.py`
- `src/python_client/xplane/client.py`
- `src/python_client/control/__init__.py`
- `src/python_client/control/heading_hold.py`
- `src/python_client/control/fly_by_wire.py`
- `src/python_client/plotting/__init__.py`
- `src/python_client/plotting/realtime.py`
- `src/python_client/plotting/analysis.py`
- `src/python_client/logging/__init__.py`
- `src/python_client/logging/schema.py`
- `src/python_client/logging/recorder.py`
- `src/python_client/runtime.py`
- `tests/test_console.py`
- `tests/test_packets.py`

### Modified

- `.gitignore`
  - Expanded Python ignores and added generated output directories such as `artifacts/` and `*.egg-info/`.
- `README.md`
  - Replaced the loose prototype notes with the new package architecture, fuller setup instructions, and the combined run command.
- `pyproject.toml`
  - Added package metadata, setuptools configuration, optional dependencies, CLI scripts, and basic tool configuration, including the combined run entry point.
- `src/python_client/cli/run_client.py`
  - Refactored terminal telemetry output to use the shared streaming runtime.
- `src/python_client/cli/record_session.py`
  - Refactored CSV recording to use the shared streaming runtime.
- `src/python_client/plotting/realtime.py`
  - Refactored the live plot into a reusable `LivePlotter` handler so it can run alongside other outputs.
- `src/python_client/logging/recorder.py`
  - Added handler-style and direct-close methods for use in the combined runtime.

### Removed

- `src/python_client/python_client.py`
  - Replaced by the new package structure and CLI entry points.

### Verification

- `python3 -m compileall src tests`
- `PYTHONPATH=src python3 -m python_client --help`
- `PYTHONPATH=src python3 -m python_client.cli.record_session --help`
- `PYTHONPATH=src python3 -m python_client.cli.run_all --help`
- `pytest -q`

### Next Step Ideas

- Add actuator command transmission to the packaged runtime so Python can both read telemetry from and send control inputs to X-Plane.
- Build a closed-loop execution path with the sequence: read state -> compute control -> send command -> log results.
- Expand the recording schema to include timestamps, reference commands, control outputs, and experiment metadata.
- Measure and standardize loop timing so simple PID control can be tuned against a known sample period.
- Add scripted step-input experiment support for repeatable controller evaluation runs.
- Add offline analysis plots and metrics for step response, including overshoot, settling time, rise time, and steady-state error.
- Add replay tooling so recorded sessions can be re-plotted and compared across controller versions.
