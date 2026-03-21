import argparse
from datetime import datetime
from pathlib import Path

from python_client.config import DEFAULT_NETWORK_CONFIG, DEFAULT_PATHS_CONFIG
from python_client.logging.recorder import SessionRecorder
from python_client.runtime import close_handlers, stream_to_handlers


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Record an X-Plane telemetry session to CSV."
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
    recorder = SessionRecorder(output_path)

    try:
        recorder.__enter__()
        print(f"Recording telemetry to {output_path}. Press Ctrl+C to stop.\n")
        stream_to_handlers(wait=args.wait, hz=args.hz, handlers=[recorder])
    except KeyboardInterrupt:
        print("\nStopping...")
    except TimeoutError as exc:
        print(exc)
        return 1
    finally:
        close_handlers([recorder])

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
