import argparse

from python_client.config import DEFAULT_NETWORK_CONFIG
from python_client.control import YawDamperController, YawDamperGains, YawDamperHandler
from python_client.runtime import close_handlers, run_client_to_handlers
from python_client.xplane.client import XPlaneClient
from python_client.xplane.exceptions import XPlaneIpNotFound


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Run the legacy yaw damper against a live X-Plane telemetry stream."
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
        "--yaw-damper-gain",
        type=float,
        default=1.0,
        help="Proportional gain used by the yaw damper.",
    )
    parser.add_argument(
        "--desired-yaw-rate-rad-s",
        type=float,
        default=0.0,
        help="Desired yaw rate in radians per second.",
    )
    return parser


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    handlers = []

    try:
        with XPlaneClient.discover(wait=args.wait, hz=args.hz) as client:
            print("Found X-Plane:")
            print(f"  Hostname: {client.beacon.hostname}")
            print(f"  IP:       {client.beacon.ip}")
            print(f"  UDP port: {client.beacon.port}")
            print(f"  Version:  {client.beacon.xplane_version}")
            print()

            yaw_damper = YawDamperHandler(
                sender=client,
                controller=YawDamperController(
                    YawDamperGains(proportional_gain=args.yaw_damper_gain)
                ),
                desired_yaw_rate_rad_s=args.desired_yaw_rate_rad_s,
                print_status=True,
            )
            handlers = [yaw_damper]
            print("Running yaw damper. Press Ctrl+C to stop.\n")
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
