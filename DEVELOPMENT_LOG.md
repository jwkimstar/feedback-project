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

## 2026-04-02

### Summary

Refreshed the repository context to match the current tree and current local verification environment.

The project is still centered on a `src/python_client/` package for X-Plane telemetry experiments, with transport, CLI entry points, logging, plotting, and controller prototypes split into separate modules. The older monolithic prototype also still exists as `actual_code.py`, which matters because it contains control-command transmission logic that has not yet been folded back into the packaged runtime.

### Current Repository State

- `python_client.xplane`
  - Handles multicast beacon discovery, UDP client setup, RPOS request/stop packets, and RPOS packet parsing into typed models.
- `python_client.cli`
  - Exposes the main runnable surfaces:
    - `run_client`: print live pitch, heading, and roll
    - `record_session`: stream telemetry into CSV
    - `plot_live`: open the live Matplotlib plot
    - `run_all`: fan out one telemetry stream to terminal, CSV, and plot handlers
- `python_client.runtime`
  - Provides the shared loop that discovers X-Plane, receives telemetry, and dispatches each sample to handler objects.
- `python_client.logging`
  - Writes telemetry-only CSV rows using a fixed schema with no timestamps, reference commands, or actuator outputs yet.
- `python_client.plotting`
  - Includes a live plotter and a minimal offline CSV loader, but not step-response metrics or richer analysis outputs yet.
- `python_client.control`
  - Contains prototype control abstractions:
    - `HeadingHoldController` computes an aileron command from heading and roll feedback
    - `ManualFlyByWire` passes pilot inputs through a stable interface for future augmentation
- `actual_code.py`
  - Remains in the repository as the older all-in-one script.
  - It still contains direct `DATA` packet transmission for rudder commands and a yaw-damper experiment path that the packaged runtime does not yet implement.

### Verification Notes

- `pyproject.toml` declares `requires-python = ">=3.11"`.
- The current local default interpreter is `Python 3.9.6`, so package imports fail immediately on `float | None` type syntax in `src/python_client/models.py`.
- `PYTHONPATH=src python3 -m python_client --help`
  - Failed under Python 3.9 due to the Python 3.11+ type-annotation syntax used by the package.
- `PYTHONPATH=src python3 -m python_client.cli.record_session --help`
  - Failed for the same interpreter-version reason.
- `PYTHONPATH=src python3 -m python_client.cli.run_all --help`
  - Failed for the same interpreter-version reason.
- `python3 -m pytest -q`
  - Failed because `pytest` is not installed in the current Python 3.9 environment.
- `python3 -m compileall src tests`
  - Failed in the sandbox because Python attempted to write bytecode cache files under `/Users/jaykim/Library/Caches/com.apple.python/`, which is outside the writable sandbox.

### Context To Carry Forward

- Meaningful local verification now depends on setting up a Python 3.11+ environment first.
- The packaged runtime can ingest and fan out telemetry, but it still does not send actuator commands back to X-Plane.
- Controller prototypes exist, but they are not yet wired into a closed-loop execution path.
- `actual_code.py` is still the only in-repo implementation that demonstrates direct control-command transmission, so it remains relevant reference material until that path is migrated into `src/python_client/`.

## 2026-04-02

### Summary

Migrated the prototype control path from `actual_code.py` into the packaged `python_client` control system, expanded the control stack into composable blocks, and updated the telemetry, plotting, recording, and CLI surfaces around that new control path.

The packaged control path now supports three modes that mirror the prototype chain depth: yaw damper only, yaw damper plus roll damper, and heading hold plus roll damper plus yaw damper. Both the dedicated controller runner and the combined telemetry runner can activate those modes without opening multiple competing X-Plane telemetry subscriptions.

During the same session, the logging and visualization paths were expanded so command outputs can be recorded and plotted, live plots were reworked into a multi-row layout, an offline CSV plotting tool was added, and the user-facing unit system was shifted to degrees / degrees-per-second while keeping controller internals in radians.

### Added

- `src/python_client/control/legacy_heading_hold.py`
  - Added the simple proportional heading-hold block preserved from `actual_code.py`.
- `src/python_client/control/roll_damper.py`
  - Added the simple proportional roll-damper block preserved from `actual_code.py`.
- `src/python_client/control/master_controller.py`
  - Added the cascaded controller, control modes, and a handler that emits the final aileron command.
- `src/python_client/cli/control_options.py`
  - Added shared CLI argument wiring for the three mutually exclusive control-mode flags and their gains/targets.
- `src/python_client/cli/plot_recording.py`
  - Added an offline plotting entry point for post-simulation CSV review.
- `tests/test_master_controller.py`
  - Added regression tests for the three-block controller chain and its individual block formulas.
- `tests/test_logging_schema.py`
  - Added coverage for the expanded CSV schema and optional command columns.
- `tests/test_plotting_analysis.py`
  - Added coverage for offline CSV loading and elapsed-time reconstruction.

### Modified

- `src/python_client/control/yaw_damper.py`
  - Refactored the yaw-damper block to accept a generic upstream signal so it can sit at the end of the cascade.
- `src/python_client/control/__init__.py`
  - Exported the new block controllers, master controller, and mode types.
- `src/python_client/cli/run_yaw_damper.py`
  - Reworked the dedicated controller runner to use the packaged master controller and support `--yaw-damper`, `--yaw-roll-damper`, and `--yaw-roll-heading-hold`.
- `src/python_client/cli/run_all.py`
  - Reworked the combined runner to accept the same control-mode flags and drive the master controller in the same process as plotting, logging, and console output.
- `src/python_client/plotting/realtime.py`
  - Reworked the live plot layout to use a three-row figure:
    - roll, pitch, yaw / heading
    - commanded aileron, elevator, rudder
    - `p`, `q`, `r`
  - Converted displayed body rates from radians per second to degrees per second.
- `src/python_client/plotting/analysis.py`
  - Expanded offline CSV loading to include control-command traces and degree-based rate columns.
  - Added post-simulation plotting for roll, pitch, yaw / heading, control commands, and `p/q/r` over elapsed time.
- `src/python_client/logging/schema.py`
  - Expanded the CSV schema to record `aileron_cmd`, `elevator_cmd`, and `rudder_cmd`.
  - Converted recorded `p`, `q`, and `r` values from rad/s to deg/s in the stored CSV output.
- `src/python_client/logging/recorder.py`
  - Updated the recorder to pull the latest control command from the active controller so command traces are stored alongside telemetry.
- `src/python_client/console.py`
  - Updated terminal output to show commanded aileron plus `q` and `r`.
  - Converted displayed rates from rad/s to deg/s.
- `pyproject.toml`
  - Added the installed script entry point for offline CSV plotting.
- `README.md`
  - Documented the new control modes, the degree-based CLI flags, and the offline plotting workflow.
- `DEVELOPMENT_LOG.md`
  - Updated this session summary to reflect the full scope of the control, logging, plotting, and unit-conversion work.
- `tests/test_console.py`
  - Updated console expectations around the current telemetry / command display format.
- `tests/test_yaw_damper.py`
  - Updated the yaw-damper regression tests to match the refactored controller interface.

### Verification

- `PYTHONPYCACHEPREFIX=/tmp/codex-pycache python3 -m compileall src tests`

### Next Step Ideas

- Check radians-to-degrees conversion across the CLI input path, CSV output, console output, and both plotting paths.
- Tune the yaw-roll damping path against live simulation data.
- Verify the full yaw-roll-heading controller mode in simulation and confirm the chained block behavior matches intent.
