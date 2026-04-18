import pytest
from math import radians

from python_client.control import (
    HeadingHoldController,
    HeadingHoldGains,
    MasterController,
    MasterControllerControllerTypes,
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
    controller = HeadingHoldController(HeadingHoldGains(proportional_gain=0.35))

    output = controller.compute_output(12.0, 4.0)

    assert output == pytest.approx(0.35 * radians(4.0 - 12.0))


def test_legacy_heading_hold_wraps_to_shortest_heading_error() -> None:
    controller = HeadingHoldController(HeadingHoldGains(proportional_gain=1.0))

    right_turn = controller.compute_output(273.0, 0.0)
    left_turn = controller.compute_output(10.0, 350.0)

    assert right_turn == pytest.approx(radians(87.0))
    assert left_turn == pytest.approx(radians(-20.0))


def test_legacy_heading_hold_pi_accumulates_integral_action() -> None:
    controller = HeadingHoldController(
        HeadingHoldGains(
            proportional_gain=0.25,
            integral_gain=0.5,
            integral_limit=10.0,
        )
    )

    first = controller.compute_output_pi(12.0, 4.0, dt_s=0.5)
    second = controller.compute_output_pi(12.0, 4.0, dt_s=0.5)

    error = radians(4.0 - 12.0)
    assert first == pytest.approx(0.25 * error + 0.5 * error * 0.5)
    assert second == pytest.approx(0.25 * error + 0.5 * error * 1.0)


def test_heading_hold_pd_uses_measurement_derivative_without_target_kick() -> None:
    controller = HeadingHoldController(
        HeadingHoldGains(
            proportional_gain=0.0,
            derivative_gain=1.0,
        )
    )

    first = controller.compute_output_pd(10.0, 0.0, dt_s=1.0)
    second = controller.compute_output_pd(10.0, 20.0, dt_s=1.0)

    assert first == pytest.approx(0.0)
    assert second == pytest.approx(0.0)


def test_heading_hold_pd_wraps_measurement_derivative_across_heading_boundary() -> None:
    controller = HeadingHoldController(
        HeadingHoldGains(
            proportional_gain=0.0,
            derivative_gain=1.0,
        )
    )

    controller.compute_output_pd(359.0, 0.0, dt_s=1.0)
    wrapped = controller.compute_output_pd(1.0, 0.0, dt_s=1.0)

    assert wrapped == pytest.approx(-radians(2.0))


def test_heading_hold_pid_combines_integral_and_derivative_terms() -> None:
    controller = HeadingHoldController(
        HeadingHoldGains(
            proportional_gain=0.25,
            integral_gain=0.5,
            integral_limit=10.0,
            derivative_gain=1.0,
        )
    )

    first = controller.compute_output_pid(10.0, 0.0, dt_s=0.5)
    second = controller.compute_output_pid(8.0, 0.0, dt_s=0.5)

    first_error = radians(-10.0)
    second_error = radians(-8.0)
    second_integral = first_error * 0.5 + second_error * 0.5
    heading_rate = radians(-2.0) / 0.5

    assert first == pytest.approx(0.25 * first_error + 0.5 * first_error * 0.5)
    assert second == pytest.approx(
        0.25 * second_error + 0.5 * second_integral - 1.0 * heading_rate
    )


def test_roll_damper_matches_actual_code_formula() -> None:
    controller = RollDamperController(RollDamperGains(proportional_gain=0.05))

    output = controller.compute_output(0.3, 0.1)

    assert output == -0.01


def test_roll_damper_p_clamps_large_signal_to_max_yaw_rate() -> None:
    controller = RollDamperController(
        RollDamperGains(
            proportional_gain=10.0,
            min_output=-radians(5.0),
            max_output=radians(5.0),
        )
    )

    output = controller.compute_output_p(0.0, -1.0)

    assert output == pytest.approx(-radians(5.0))


def test_roll_damper_pi_clamps_large_signal_to_max_yaw_rate() -> None:
    controller = RollDamperController(
        RollDamperGains(
            proportional_gain=10.0,
            integral_gain=1.0,
            integral_limit=10.0,
            min_output=-radians(5.0),
            max_output=radians(5.0),
        )
    )

    output = controller.compute_output_pi(0.0, -1.0, dt_s=1.0)

    assert output == pytest.approx(-radians(5.0))


def test_roll_damper_pd_uses_measurement_derivative() -> None:
    controller = RollDamperController(
        RollDamperGains(
            proportional_gain=0.0,
            derivative_gain=1.0,
        )
    )

    first = controller.compute_output_pd(0.3, 0.0, dt_s=0.5)
    second = controller.compute_output_pd(0.4, 0.0, dt_s=0.5)

    assert first == pytest.approx(0.0)
    assert second == pytest.approx(-0.2)


def test_roll_damper_pd_avoids_derivative_kick_on_signal_step() -> None:
    controller = RollDamperController(
        RollDamperGains(
            proportional_gain=0.0,
            derivative_gain=1.0,
        )
    )

    first = controller.compute_output_pd(0.3, 0.0, dt_s=0.5)
    second = controller.compute_output_pd(0.3, 1.0, dt_s=0.5)

    assert first == pytest.approx(0.0)
    assert second == pytest.approx(0.0)


def test_roll_damper_pid_combines_integral_and_derivative_terms() -> None:
    controller = RollDamperController(
        RollDamperGains(
            proportional_gain=2.0,
            integral_gain=1.0,
            integral_limit=10.0,
            derivative_gain=0.5,
        )
    )

    first = controller.compute_output_pid(0.3, 0.1, dt_s=0.5)
    second = controller.compute_output_pid(0.4, 0.1, dt_s=0.5)

    assert first == pytest.approx(-0.5)
    assert second == pytest.approx(-0.95)


def test_roll_damper_reset_clears_derivative_state() -> None:
    controller = RollDamperController(
        RollDamperGains(
            proportional_gain=0.0,
            derivative_gain=1.0,
        )
    )

    controller.compute_output_pd(0.3, 0.0, dt_s=0.5)
    charged = controller.compute_output_pd(0.4, 0.0, dt_s=0.5)
    controller.reset()
    reset = controller.compute_output_pd(0.4, 0.0, dt_s=0.5)

    assert charged == pytest.approx(-0.2)
    assert reset == pytest.approx(0.0)


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


def test_master_controller_yaw_roll_pi_mode_uses_integral_action() -> None:
    controller = MasterController(
        mode=MasterControllerMode.YAW_ROLL_DAMPER,
        gains=MasterControllerGains(
            yaw_damper_gain=0.5,
            yaw_damper_integral_gain=0.5,
            roll_damper_gain=0.5,
            roll_damper_integral_gain=0.5,
        ),
        targets=MasterControllerTargets(desired_roll_rate_rad_s=0.1),
        controller_types=MasterControllerControllerTypes(
            yaw_damper="pi",
            roll_damper="pi",
        ),
    )

    first = controller.compute(make_state(p_rad_s=0.3, r_rad_s=0.4), dt_s=0.5)
    second = controller.compute(make_state(p_rad_s=0.3, r_rad_s=0.4), dt_s=0.5)

    assert first.aileron == pytest.approx(-0.4125)
    assert second.aileron == pytest.approx(-0.5875)


def test_master_controller_yaw_roll_pid_mode_uses_derivative_action() -> None:
    controller = MasterController(
        mode=MasterControllerMode.YAW_ROLL_DAMPER,
        gains=MasterControllerGains(
            yaw_damper_gain=0.5,
            yaw_damper_integral_gain=0.5,
            yaw_damper_derivative_gain=0.25,
            roll_damper_gain=0.5,
            roll_damper_integral_gain=0.5,
            roll_damper_derivative_gain=0.25,
        ),
        targets=MasterControllerTargets(desired_roll_rate_rad_s=0.1),
        controller_types=MasterControllerControllerTypes(
            yaw_damper="pid",
            roll_damper="pid",
        ),
    )

    first = controller.compute(make_state(p_rad_s=0.3, r_rad_s=0.4), dt_s=0.5)
    second = controller.compute(make_state(p_rad_s=0.35, r_rad_s=0.45), dt_s=0.5)

    assert first.aileron == pytest.approx(-0.4125)
    assert second.aileron == pytest.approx(-0.696875)


def test_master_controller_reset_clears_integral_state() -> None:
    controller = MasterController(
        mode=MasterControllerMode.YAW_DAMPER,
        gains=MasterControllerGains(
            yaw_damper_gain=0.0,
            yaw_damper_integral_gain=1.0,
        ),
        targets=MasterControllerTargets(desired_yaw_rate_rad_s=0.0),
        controller_types=MasterControllerControllerTypes(yaw_damper="pi"),
    )

    charged = controller.compute(make_state(r_rad_s=0.5), dt_s=1.0)
    controller.reset()
    reset = controller.compute(make_state(r_rad_s=0.5), dt_s=0.0)

    assert charged.aileron == pytest.approx(-0.5)
    assert reset.aileron == pytest.approx(0.0)


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

    assert command.aileron == pytest.approx(-0.45872664625997165)


def test_master_controller_heading_mode_supports_pi_heading_hold() -> None:
    controller = MasterController(
        mode=MasterControllerMode.YAW_ROLL_HEADING_HOLD,
        gains=MasterControllerGains(
            yaw_damper_gain=0.5,
            roll_damper_gain=0.2,
            heading_hold_gain=0.25,
            heading_hold_integral_gain=0.1,
        ),
        targets=MasterControllerTargets(target_heading_deg=0.0),
        controller_types=MasterControllerControllerTypes(
            yaw_damper="p",
            roll_damper="p",
            heading_hold="pi",
        ),
    )

    first = controller.compute(make_state(heading_deg=10.0, p_rad_s=0.4, r_rad_s=0.7), dt_s=0.5)
    second = controller.compute(make_state(heading_deg=10.0, p_rad_s=0.4, r_rad_s=0.7), dt_s=0.5)

    assert second.aileron < first.aileron


def test_master_controller_heading_mode_supports_pid_heading_hold() -> None:
    controller = MasterController(
        mode=MasterControllerMode.YAW_ROLL_HEADING_HOLD,
        gains=MasterControllerGains(
            yaw_damper_gain=0.5,
            roll_damper_gain=0.2,
            heading_hold_gain=0.25,
            heading_hold_integral_gain=0.1,
            heading_hold_derivative_gain=0.5,
        ),
        targets=MasterControllerTargets(target_heading_deg=0.0),
        controller_types=MasterControllerControllerTypes(
            yaw_damper="p",
            roll_damper="p",
            heading_hold="pid",
        ),
    )

    first = controller.compute(make_state(heading_deg=10.0, p_rad_s=0.4, r_rad_s=0.7), dt_s=0.5)
    second = controller.compute(make_state(heading_deg=8.0, p_rad_s=0.4, r_rad_s=0.7), dt_s=0.5)

    assert second.aileron != pytest.approx(first.aileron)


def test_master_controller_heading_mode_clamps_roll_damper_output() -> None:
    controller = MasterController(
        mode=MasterControllerMode.YAW_ROLL_HEADING_HOLD,
        gains=MasterControllerGains(
            yaw_damper_gain=0.0,
            roll_damper_gain=10.0,
            roll_damper_max_yaw_rate_rad_s=radians(5.0),
            heading_hold_gain=10.0,
        ),
        targets=MasterControllerTargets(target_heading_deg=180.0),
    )

    controller.compute(make_state(heading_deg=0.0), dt_s=1.0)
    trace = controller.get_latest_trace()

    assert trace is not None
    assert trace.roll_damper_output == pytest.approx(-radians(5.0))


def test_master_controller_reset_clears_heading_hold_integral_state() -> None:
    controller = MasterController(
        mode=MasterControllerMode.YAW_ROLL_HEADING_HOLD,
        gains=MasterControllerGains(
            yaw_damper_gain=0.0,
            roll_damper_gain=0.0,
            heading_hold_gain=0.0,
            heading_hold_integral_gain=1.0,
        ),
        targets=MasterControllerTargets(target_heading_deg=0.0),
        controller_types=MasterControllerControllerTypes(heading_hold="pi"),
    )

    charged = controller.compute(make_state(heading_deg=10.0), dt_s=1.0)
    controller.reset()
    reset = controller.compute(make_state(heading_deg=10.0), dt_s=0.0)

    assert charged.aileron == pytest.approx(0.0)
    assert reset.aileron == pytest.approx(0.0)


def test_heading_hold_reset_clears_derivative_state() -> None:
    controller = HeadingHoldController(
        HeadingHoldGains(
            proportional_gain=0.0,
            derivative_gain=1.0,
        )
    )

    controller.compute_output_pd(10.0, 0.0, dt_s=1.0)
    charged = controller.compute_output_pd(8.0, 0.0, dt_s=1.0)
    controller.reset()
    reset = controller.compute_output_pd(8.0, 0.0, dt_s=1.0)

    assert charged != pytest.approx(0.0)
    assert reset == pytest.approx(0.0)
