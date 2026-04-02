import argparse
from math import radians

from python_client.control import (
    MasterControllerGains,
    MasterControllerMode,
    MasterControllerTargets,
)


def add_master_controller_arguments(
    parser: argparse.ArgumentParser,
    *,
    require_mode: bool,
) -> None:
    group = parser.add_mutually_exclusive_group(required=require_mode)
    group.add_argument(
        "--yaw-damper",
        dest="control_mode",
        action="store_const",
        const=MasterControllerMode.YAW_DAMPER.value,
        help="Enable only the yaw-damper block.",
    )
    group.add_argument(
        "--yaw-roll-damper",
        dest="control_mode",
        action="store_const",
        const=MasterControllerMode.YAW_ROLL_DAMPER.value,
        help="Enable the yaw-damper and roll-damper blocks.",
    )
    group.add_argument(
        "--yaw-roll-heading-hold",
        dest="control_mode",
        action="store_const",
        const=MasterControllerMode.YAW_ROLL_HEADING_HOLD.value,
        help="Enable heading hold, roll damper, and yaw damper in cascade.",
    )
    parser.add_argument(
        "--yaw-damper-gain",
        type=float,
        default=MasterControllerGains.yaw_damper_gain,
        help="Proportional gain used by the yaw-damper block.",
    )
    parser.add_argument(
        "--roll-damper-gain",
        type=float,
        default=MasterControllerGains.roll_damper_gain,
        help="Proportional gain used by the roll-damper block.",
    )
    parser.add_argument(
        "--heading-hold-gain",
        type=float,
        default=MasterControllerGains.heading_hold_gain,
        help="Proportional gain used by the heading-hold block.",
    )
    parser.add_argument(
        "--desired-yaw-rate-deg-s",
        type=float,
        default=0.0,
        help="Desired yaw-rate signal in degrees per second for yaw-damper-only mode.",
    )
    parser.add_argument(
        "--desired-roll-rate-deg-s",
        type=float,
        default=0.0,
        help="Desired roll-rate signal in degrees per second for yaw-plus-roll mode.",
    )
    parser.add_argument(
        "--target-heading-deg",
        type=float,
        default=MasterControllerTargets.target_heading_deg,
        help="Target heading in degrees for yaw-plus-roll-plus-heading mode.",
    )


def build_master_controller(
    args: argparse.Namespace,
) -> tuple[MasterControllerMode | None, MasterControllerGains, MasterControllerTargets]:
    mode = None
    if args.control_mode is not None:
        mode = MasterControllerMode(args.control_mode)

    gains = MasterControllerGains(
        yaw_damper_gain=args.yaw_damper_gain,
        roll_damper_gain=args.roll_damper_gain,
        heading_hold_gain=args.heading_hold_gain,
    )
    targets = MasterControllerTargets(
        desired_yaw_rate_rad_s=radians(args.desired_yaw_rate_deg_s),
        desired_roll_rate_rad_s=radians(args.desired_roll_rate_deg_s),
        target_heading_deg=args.target_heading_deg,
    )
    return mode, gains, targets
