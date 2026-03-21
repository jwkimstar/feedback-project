import argparse
from datetime import datetime
from pathlib import Path

from python_client.console import TelemetryPrinter
from python_client.config import DEFAULT_NETWORK_CONFIG, DEFAULT_PATHS_CONFIG
from python_client.logging.recorder import SessionRecorder
from python_client.plotting.realtime import LivePlotter
from python_client.runtime import close_handlers, stream_to_handlers


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
    return parser


def default_output_path() -> Path:
    timestamp = datetime.now().strftime("%Y%m%d-%H%M%S")
    return DEFAULT_PATHS_CONFIG.artifacts_dir / f"session-{timestamp}.csv"


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    output_path = args.output or default_output_path()
    handlers = []

    try:
        printer = TelemetryPrinter()
        recorder = SessionRecorder(output_path)
        plotter = LivePlotter(hz=args.hz, history_seconds=args.history_seconds)
        handlers = [printer, recorder, plotter]

        recorder.__enter__()
        print("Streaming telemetry, recording CSV, and updating the live plot.")
        print(f"Recording telemetry to {output_path}. Press Ctrl+C to stop.\n")
        stream_to_handlers(wait=args.wait, hz=args.hz, handlers=handlers)
    except KeyboardInterrupt:
        print("\nStopping...")
    except (ImportError, TimeoutError) as exc:
        print(exc)
        return 1
    finally:
        close_handlers(handlers)

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
