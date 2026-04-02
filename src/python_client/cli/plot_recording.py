import argparse
from pathlib import Path

from python_client.config import DEFAULT_NETWORK_CONFIG
from python_client.plotting.analysis import plot_recording


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Plot a recorded CSV session against elapsed time."
    )
    parser.add_argument(
        "path",
        type=Path,
        help="Path to a recorded CSV file.",
    )
    parser.add_argument(
        "--hz",
        type=int,
        default=DEFAULT_NETWORK_CONFIG.rpos_hz,
        help="Sample rate used when the recording was captured. It is used to reconstruct the elapsed-time axis because the CSV does not store timestamps.",
    )
    return parser


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)

    try:
        plot_recording(args.path, hz=args.hz)
    except (ImportError, OSError, ValueError) as exc:
        print(exc)
        return 1

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
