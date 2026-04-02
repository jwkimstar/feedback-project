import argparse
from datetime import datetime
from pathlib import Path

from python_client.console import TelemetryPrinter
from python_client.config import DEFAULT_NETWORK_CONFIG, DEFAULT_PATHS_CONFIG
from python_client.control import YawDamperController, YawDamperGains, YawDamperHandler
from python_client.logging.recorder import SessionRecorder
from python_client.plotting.realtime import LivePlotter
from python_client.runtime import close_handlers, run_client_to_handlers
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
    parser.add_argument(
        "--yaw-damper",
        action="store_true",
        help="Enable the legacy yaw damper in the same process as the plotter/loggers.",
    )
    parser.add_argument(
        "--yaw-damper-gain",
        type=float,
        default=1.0,
        help="Proportional gain used by the yaw damper when enabled.",
    )
    parser.add_argument(
        "--desired-yaw-rate-rad-s",
        type=float,
        default=0.0,
        help="Desired yaw rate in radians per second when the yaw damper is enabled.",
    )
    return parser


def default_output_path() -> Path:
    timestamp = datetime.now().strftime("%Y%m%d-%H%M%S")
    return DEFAULT_PATHS_CONFIG.artifacts_dir / f"session-{timestamp}.csv"


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    output_path = args.output or default_output_path()
    handlers = []

    try:
        with XPlaneClient.discover(wait=args.wait, hz=args.hz) as client:
            command_provider = None
            yaw_damper = None
            if args.yaw_damper:
                yaw_damper = YawDamperHandler(
                    sender=client,
                    controller=YawDamperController(
                        YawDamperGains(proportional_gain=args.yaw_damper_gain)
                    ),
                    desired_yaw_rate_rad_s=args.desired_yaw_rate_rad_s,
                )
                command_provider = yaw_damper

            printer = TelemetryPrinter(command_provider=command_provider)
            recorder = SessionRecorder(output_path)
            plotter = LivePlotter(
                hz=args.hz,
                history_seconds=args.history_seconds,
                command_provider=command_provider,
            )
            handlers = [recorder]
            if yaw_damper is not None:
                handlers.append(yaw_damper)
            handlers.extend([printer, plotter])

            recorder.__enter__()
            print("Streaming telemetry, recording CSV, and updating the live plot.")
            if args.yaw_damper:
                print("Yaw damper control is enabled in this same process.")
            print(f"Recording telemetry to {output_path}. Press Ctrl+C to stop.\n")
            run_client_to_handlers(client, handlers)
    except KeyboardInterrupt:
        print("\nStopping...")
    except (ImportError, TimeoutError, XPlaneIpNotFound) as exc:
        print(exc)
        return 1
    finally:
        close_handlers(handlers)

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
