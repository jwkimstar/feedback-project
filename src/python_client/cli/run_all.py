import argparse
from datetime import datetime
from pathlib import Path

from python_client.cli.control_options import add_master_controller_arguments, build_master_controller
from python_client.console import TelemetryPrinter
from python_client.config import DEFAULT_NETWORK_CONFIG, DEFAULT_PATHS_CONFIG
from python_client.control import MasterController, MasterControllerHandler
from python_client.logging.recorder import SessionRecorder
from python_client.plotting.realtime import LivePlotter
from python_client.runtime import StopRequested, close_handlers, run_client_to_handlers, trap_sigint
from python_client.xplane.client import XPlaneClient
from python_client.xplane.exceptions import XPlaneIpNotFound


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Stream telemetry, record CSV, and plot live data in one process."
    )
    parser.add_argument(
        "--wait",
        type=float,
        default=DEFAULT_NETWORK_CONFIG.beacon_wait_seconds,
        help="Seconds to wait for the X-Plane beacon.",
    )
    parser.add_argument(
        "--hz",
        type=int,
        default=DEFAULT_NETWORK_CONFIG.rpos_hz,
        help="Requested RPOS update rate in Hz.",
    )
    parser.add_argument(
        "--history-seconds",
        type=int,
        default=60,
        help="How much recent telemetry to keep visible in the live plot.",
    )
    parser.add_argument(
        "--output",
        type=Path,
        default=None,
        help="Path to the CSV file. Defaults to artifacts/session-YYYYMMDD-HHMMSS.csv",
    )
    add_master_controller_arguments(parser, require_mode=False)
    return parser


def default_output_path() -> Path:
    timestamp = datetime.now().strftime("%Y%m%d-%H%M%S")
    return DEFAULT_PATHS_CONFIG.artifacts_dir / f"session-{timestamp}.csv"


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    output_path = args.output or default_output_path()
    handlers = []
    mode, gains, targets, controller_types = build_master_controller(args)

    try:
        with trap_sigint() as should_stop:
            with XPlaneClient.discover(wait=args.wait, hz=args.hz) as client:
                command_provider = None
                master_handler = None
                if mode is not None:
                    master_handler = MasterControllerHandler(
                        sender=client,
                        controller=MasterController(
                            mode=mode,
                            gains=gains,
                            targets=targets,
                            controller_types=controller_types,
                        ),
                    )
                    command_provider = master_handler

                printer = TelemetryPrinter(command_provider=command_provider)
                recorder = SessionRecorder(output_path, command_provider=command_provider)
                plotter = LivePlotter(
                    hz=args.hz,
                    history_seconds=args.history_seconds,
                    command_provider=command_provider,
                )
                if master_handler is not None:
                    handlers = [master_handler, recorder, printer, plotter]
                else:
                    handlers = [recorder, printer, plotter]

                recorder.__enter__()
                print("Streaming telemetry, recording CSV, and updating the live plot.")
                if mode is not None:
                    print(f"Control mode '{mode.value}' is enabled in this same process.")
                print(f"Recording telemetry to {output_path}. Press Ctrl+C to stop.\n")
                run_client_to_handlers(client, handlers, should_stop=should_stop)
    except (KeyboardInterrupt, StopRequested):
        print("\nStopping...")
    except (ImportError, TimeoutError, XPlaneIpNotFound) as exc:
        print(exc)
        return 1
    finally:
        close_handlers(handlers)

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
