import argparse

from python_client.config import DEFAULT_NETWORK_CONFIG
from python_client.plotting.realtime import plot_live_stream


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Open a real-time plot of pitch, heading, and roll."
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
    return parser


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)

    try:
        plot_live_stream(
            wait=args.wait,
            hz=args.hz,
            history_seconds=args.history_seconds,
        )
    except KeyboardInterrupt:
        print("\nStopping...")
    except (ImportError, RuntimeError, TimeoutError) as exc:
        print(exc)
        return 1

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
