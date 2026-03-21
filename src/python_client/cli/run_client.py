import argparse

from python_client.console import TelemetryPrinter
from python_client.config import DEFAULT_NETWORK_CONFIG
from python_client.runtime import close_handlers, stream_to_handlers


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Stream aircraft orientation data from X-Plane."
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
    return parser


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    printer = TelemetryPrinter()

    try:
        print("Printing aircraft orientation. Press Ctrl+C to stop.\n")
        stream_to_handlers(wait=args.wait, hz=args.hz, handlers=[printer])
    except KeyboardInterrupt:
        print("\nStopping...")
    except TimeoutError as exc:
        print(exc)
        return 1
    finally:
        close_handlers([printer])

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
