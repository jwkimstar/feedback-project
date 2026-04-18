import argparse
from math import radians

from python_client.control import (
    MasterControllerControllerTypes,
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
        "--heading-hold-only",
        dest="control_mode",
        action="store_const",
        const=MasterControllerMode.HEADING_HOLD_ONLY.value,
        help="Enable a direct heading-hold baseline without roll or yaw dampers.",
    )
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
        "--yaw-damper-integral-gain",
        type=float,
        default=MasterControllerGains.yaw_damper_integral_gain,
        help="Integral gain used by the yaw-damper block when PI mode is selected.",
    )
    parser.add_argument(
        "--roll-damper-gain",
        type=float,
        default=MasterControllerGains.roll_damper_gain,
        help="Proportional gain used by the roll-damper block.",
    )
    parser.add_argument(
        "--roll-damper-integral-gain",
        type=float,
        default=MasterControllerGains.roll_damper_integral_gain,
        help="Integral gain used by the roll-damper block when PI mode is selected.",
    )
    parser.add_argument(
        "--roll-damper-max-yaw-rate-deg-s",
        type=float,
        default=None,
        help="Clamp the roll-damper output to a symmetric max commanded yaw rate in deg/s.",
    )
    parser.add_argument(
        "--yaw-controller-type",
        choices=("p", "pi"),
        default=MasterControllerControllerTypes.yaw_damper,
        help="Choose the yaw-damper controller type.",
    )
    parser.add_argument(
        "--roll-controller-type",
        choices=("p", "pi"),
        default=MasterControllerControllerTypes.roll_damper,
        help="Choose the roll-damper controller type.",
    )
    parser.add_argument(
        "--heading-controller-type",
        choices=("p", "pi", "pd", "pid"),
        default=MasterControllerControllerTypes.heading_hold,
        help="Choose the heading-hold controller type.",
    )
    parser.add_argument(
        "--heading-hold-gain",
        type=float,
        default=MasterControllerGains.heading_hold_gain,
        help="Proportional gain used by the heading-hold block.",
    )
    parser.add_argument(
        "--heading-hold-integral-gain",
        type=float,
        default=MasterControllerGains.heading_hold_integral_gain,
        help="Integral gain used by the heading-hold block when PI mode is selected.",
    )
    parser.add_argument(
        "--heading-hold-derivative-gain",
        type=float,
        default=MasterControllerGains.heading_hold_derivative_gain,
        help="Derivative gain used by the heading-hold block when PD or PID mode is selected.",
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
        help="Target heading in degrees for heading-hold modes.",
    )


def build_master_controller(
    args: argparse.Namespace,
) -> tuple[
    MasterControllerMode | None,
    MasterControllerGains,
    MasterControllerTargets,
    MasterControllerControllerTypes,
]:
    mode = None
    if args.control_mode is not None:
        mode = MasterControllerMode(args.control_mode)

    gains = MasterControllerGains(
        yaw_damper_gain=args.yaw_damper_gain,
        yaw_damper_integral_gain=args.yaw_damper_integral_gain,
        roll_damper_gain=args.roll_damper_gain,
        roll_damper_integral_gain=args.roll_damper_integral_gain,
        roll_damper_max_yaw_rate_rad_s=(
            None
            if args.roll_damper_max_yaw_rate_deg_s is None
            else radians(args.roll_damper_max_yaw_rate_deg_s)
        ),
        heading_hold_gain=args.heading_hold_gain,
        heading_hold_integral_gain=args.heading_hold_integral_gain,
        heading_hold_derivative_gain=args.heading_hold_derivative_gain,
    )
    targets = MasterControllerTargets(
        desired_yaw_rate_rad_s=radians(args.desired_yaw_rate_deg_s),
        desired_roll_rate_rad_s=radians(args.desired_roll_rate_deg_s),
        target_heading_deg=args.target_heading_deg,
    )
    controller_types = MasterControllerControllerTypes(
        yaw_damper=args.yaw_controller_type,
        roll_damper=args.roll_controller_type,
        heading_hold=args.heading_controller_type,
    )
    return mode, gains, targets, controller_types
