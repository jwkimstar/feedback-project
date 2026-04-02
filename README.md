# feedback-project

Prototype tooling for aircraft fault-tolerant steering experiments in X-Plane.

## Current package architecture

The source tree now uses a single package namespace under `src/python_client/`:

- `python_client.xplane`: X-Plane beacon discovery, UDP transport, and packet parsing
- `python_client.control`: controller modules such as heading hold and fly-by-wire stubs
- `python_client.plotting`: real-time and offline plotting entry points
- `python_client.logging`: telemetry recording and CSV schema helpers
- `python_client.cli`: runnable command modules

## Development goals

1. Compare closed-loop control against human pilot steering during rudder-failure scenarios.
2. Build fly-by-wire control paths that can recover or compensate for lost rudder authority.
3. Capture telemetry and produce repeatable plots for analysis.
4. Keep transport, control, plotting, and recording code decoupled enough to test independently.

## Requirements

- Python 3.11 or newer
- X-Plane running on the same network
- X-Plane network access enabled so the client can receive the UDP beacon and RPOS telemetry

If you use a virtual environment, activate it first.

## Installation

Install the package in editable mode from the repository root:

```bash
python3 -m pip install -e .
```

If you want live plotting support, install the plotting extra.

If you are using `zsh`, quote the extras specifier:

```bash
python3 -m pip install -e '.[plots]'
```

Equivalent `zsh`-safe form:

```bash
python3 -m pip install -e .\[plots\]
```

## How To Run

### 1. Stream live orientation telemetry

This discovers X-Plane via its UDP beacon, subscribes to `RPOS` data, and prints pitch, heading, and roll.

```bash
python3 -m python_client --hz 10
```

Installed script form:

```bash
python-client --hz 10
```

Options:
- `--hz`: requested telemetry update rate
- `--wait`: seconds to wait for X-Plane discovery before timing out

Example:

```bash
python3 -m python_client --hz 5 --wait 10
```

### 2. Record a telemetry session to CSV

This writes telemetry samples to a CSV file for later analysis.

```bash
python3 -m python_client.cli.record_session --hz 10
```

Installed script form:

```bash
python-client-record --hz 10
```

By default, output is written to:

```text
artifacts/session-YYYYMMDD-HHMMSS.csv
```

To choose an explicit output path:

```bash
python3 -m python_client.cli.record_session --hz 10 --output artifacts/my-run.csv
```

### 3. Open a real-time plot

This opens a live Matplotlib window and plots pitch, heading, and roll over time.

```bash
python3 -m python_client.cli.plot_live --hz 10 --history-seconds 60
```

Installed script form:

```bash
python-client-plot-live --hz 10 --history-seconds 60
```

Options:
- `--hz`: requested telemetry update rate
- `--wait`: seconds to wait for X-Plane discovery
- `--history-seconds`: amount of recent telemetry to keep on screen

### 4. Run terminal output, CSV recording, and live plotting together

This uses a single telemetry stream and fans each sample out to all three outputs in one process.

```bash
python3 -m python_client.cli.run_all --hz 10 --history-seconds 60
```

Installed script form:

```bash
python-client-run-all --hz 10 --history-seconds 60
```

To choose an explicit CSV output path:

```bash
python3 -m python_client.cli.run_all --hz 10 --output artifacts/all-in-one.csv
```

### 5. Run the legacy yaw damper inside the package

This preserves the control law that previously lived in `actual_code.py`, but runs it through the packaged client instead of opening a competing telemetry subscription.

```bash
python3 -m python_client.cli.run_yaw_damper --hz 10 --yaw-damper-gain 1.0
```

Installed script form:

```bash
python-client-run-yaw-damper --hz 10 --yaw-damper-gain 1.0
```

To target a nonzero yaw rate, pass `--desired-yaw-rate-rad-s`:

```bash
python3 -m python_client.cli.run_yaw_damper --hz 10 --yaw-damper-gain 1.0 --desired-yaw-rate-rad-s 0.05
```

Options:
- `--yaw-damper-gain`: proportional gain for the yaw-damper controller
- `--desired-yaw-rate-rad-s`: desired yaw rate in radians per second. `0.0` means drive yaw rate toward zero.

### 6. Run plotting, recording, terminal output, and yaw damper together

This is the recommended way to plot telemetry and apply the yaw damper concurrently, because everything shares one X-Plane client and one `RPOS` stream.

```bash
python3 -m python_client.cli.run_all --hz 10 --history-seconds 60 --yaw-damper
```

To use the same desired-yaw-rate setting in the combined runner:

```bash
python3 -m python_client.cli.run_all --hz 10 --history-seconds 60 --yaw-damper --desired-yaw-rate-rad-s 0.05
```

## Running Without Installing

If you want to run directly from source without `pip install -e`, use:

```bash
PYTHONPATH=src python3 -m python_client --hz 10
```

Other commands:

```bash
PYTHONPATH=src python3 -m python_client.cli.record_session --hz 10
```

```bash
PYTHONPATH=src python3 -m python_client.cli.plot_live --hz 10 --history-seconds 60
```

```bash
PYTHONPATH=src python3 -m python_client.cli.run_all --hz 10 --history-seconds 60
```

## Package Layout

The current package lives under `src/python_client/`:

- `python_client.__main__`: default module entry point
- `python_client.main`: top-level entry shim
- `python_client.cli.run_client`: stream live telemetry in the terminal
- `python_client.cli.record_session`: record telemetry to CSV
- `python_client.cli.plot_live`: open a live telemetry plot
- `python_client.cli.run_all`: run terminal output, CSV recording, and live plotting together
- `python_client.cli.run_yaw_damper`: run the migrated yaw damper controller
- `python_client.xplane`: X-Plane discovery, UDP socket handling, and packet parsing
- `python_client.control`: controller logic modules including heading hold and yaw damper
- `python_client.logging`: telemetry recording helpers
- `python_client.plotting`: live and offline plotting helpers
- `python_client.models`: shared typed data structures
- `python_client.config`: package configuration defaults
- `python_client.runtime`: shared telemetry streaming loop used by multiple entry points
- `python_client.console`: terminal formatting and telemetry printing helpers

## Development Commands

Run the test suite:

```bash
pytest -q
```

Run from source during development:

```bash
PYTHONPATH=src python3 -m python_client --help
```

## Common Issues

### `zsh: no matches found: .[plots]`

Your shell expanded the brackets before passing them to `pip`.

Use:

```bash
python3 -m pip install -e '.[plots]'
```

### X-Plane is not discovered

If the client times out waiting for X-Plane:

- confirm X-Plane is running
- confirm X-Plane accepts incoming network connections
- confirm both systems are on the same network if X-Plane is running remotely
- try a larger wait time, for example `--wait 10`

### Live plot command fails with `ImportError`

Install the plotting dependency:

```bash
python3 -m pip install -e '.[plots]'
```

### Why the original commands do not run together in one process

`python_client`, `python_client.cli.record_session`, and `python_client.cli.plot_live` were originally written as separate blocking applications. Each one:

- discovers X-Plane independently
- opens its own socket
- enters its own infinite loop
- owns the process until interrupted

That means one command cannot also run the others unless there is an orchestrator that reads one telemetry stream and dispatches each sample to multiple outputs. `python_client.cli.run_all` is that combined entry point, and the new `--yaw-damper` option extends that same pattern to closed-loop control.
