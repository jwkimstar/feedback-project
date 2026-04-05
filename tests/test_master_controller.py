import pytest

from python_client.control import (
    LegacyHeadingHoldController,
    LegacyHeadingHoldGains,
    MasterController,
    MasterControllerGains,
    MasterControllerMode,
    MasterControllerTargets,
    RollDamperController,
    RollDamperGains,
)
from python_client.models import AircraftState, ControlCommand


def make_state(
    *,
    heading_deg: float = 0.0,
    p_rad_s: float = 0.0,
    r_rad_s: float = 0.0,
) -> AircraftState:
    return AircraftState(
        lon_deg=0.0,
        lat_deg=0.0,
        ele_m=0.0,
        agl_m=0.0,
        pitch_deg=0.0,
        heading_deg=heading_deg,
        roll_deg=0.0,
        vx_east=0.0,
        vy_up=0.0,
        vz_south=0.0,
        p_rad_s=p_rad_s,
        q_rad_s=0.0,
        r_rad_s=r_rad_s,
    )


def test_legacy_heading_hold_matches_actual_code_formula() -> None:
    controller = LegacyHeadingHoldController(
        LegacyHeadingHoldGains(proportional_gain=0.35)
    )

    output = controller.compute_output(12.0, 4.0)

    assert output == -2.8


def test_roll_damper_matches_actual_code_formula() -> None:
    controller = RollDamperController(RollDamperGains(proportional_gain=0.05))

    output = controller.compute_output(0.3, 0.1)

    assert output == -0.01


def test_master_controller_yaw_only_mode_returns_aileron_command() -> None:
    controller = MasterController(
        mode=MasterControllerMode.YAW_DAMPER,
        gains=MasterControllerGains(yaw_damper_gain=2.0),
        targets=MasterControllerTargets(desired_yaw_rate_rad_s=0.1),
    )

    command = controller.compute(make_state(r_rad_s=0.25))

    assert command == ControlCommand(aileron=-0.3)


def test_master_controller_yaw_roll_mode_cascades_roll_into_yaw() -> None:
    controller = MasterController(
        mode=MasterControllerMode.YAW_ROLL_DAMPER,
        gains=MasterControllerGains(yaw_damper_gain=2.0, roll_damper_gain=0.5),
        targets=MasterControllerTargets(desired_roll_rate_rad_s=0.1),
    )

    command = controller.compute(make_state(p_rad_s=0.3, r_rad_s=0.4))

    assert command.aileron == pytest.approx(-1.0)


def test_master_controller_heading_mode_matches_three_block_cascade() -> None:
    controller = MasterController(
        mode=MasterControllerMode.YAW_ROLL_HEADING_HOLD,
        gains=MasterControllerGains(
            yaw_damper_gain=0.5,
            roll_damper_gain=0.5,
            heading_hold_gain=0.25,
        ),
        targets=MasterControllerTargets(target_heading_deg=2.0),
    )

    command = controller.compute(
        make_state(heading_deg=10.0, p_rad_s=0.4, r_rad_s=0.7)
    )

    assert command == ControlCommand(aileron=-0.95)
