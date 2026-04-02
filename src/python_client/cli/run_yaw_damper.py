import argparse

from python_client.cli.control_options import add_master_controller_arguments, build_master_controller
from python_client.config import DEFAULT_NETWORK_CONFIG
from python_client.control import MasterController, MasterControllerHandler, MasterControllerMode
from python_client.runtime import close_handlers, run_client_to_handlers
from python_client.xplane.client import XPlaneClient
from python_client.xplane.exceptions import XPlaneIpNotFound


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Run the prototype master controller against a live X-Plane telemetry stream."
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
    add_master_controller_arguments(parser, require_mode=False)
    return parser


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    handlers = []
    mode, gains, targets = build_master_controller(args)
    mode = mode or MasterControllerMode.YAW_DAMPER

    try:
        with XPlaneClient.discover(wait=args.wait, hz=args.hz) as client:
            print("Found X-Plane:")
            print(f"  Hostname: {client.beacon.hostname}")
            print(f"  IP:       {client.beacon.ip}")
            print(f"  UDP port: {client.beacon.port}")
            print(f"  Version:  {client.beacon.xplane_version}")
            print()

            controller = MasterController(mode=mode, gains=gains, targets=targets)
            master_handler = MasterControllerHandler(
                sender=client,
                controller=controller,
                print_status=True,
            )
            handlers = [master_handler]
            print(f"Running control mode '{mode.value}'. Press Ctrl+C to stop.\n")
            run_client_to_handlers(client, handlers)
    except KeyboardInterrupt:
        print("\nStopping...")
    except (TimeoutError, XPlaneIpNotFound) as exc:
        print(exc)
        print(
            "If you are using X-Plane 12, check Settings > Network > Accept incoming connections."
        )
        return 1
    finally:
        close_handlers(handlers)

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
