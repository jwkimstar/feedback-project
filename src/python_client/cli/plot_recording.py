import argparse
from pathlib import Path

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
        default=None,
        help="Optional sample-rate override used to reconstruct elapsed time. If omitted, the tool reads hz from newer CSV recording metadata.",
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
