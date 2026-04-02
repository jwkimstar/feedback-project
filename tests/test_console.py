from python_client.console import format_state_line
from python_client.models import AircraftState, ControlCommand


def test_format_state_line() -> None:
    state = AircraftState(
        lon_deg=0.0,
        lat_deg=0.0,
        ele_m=0.0,
        agl_m=0.0,
        pitch_deg=1.25,
        heading_deg=180.0,
        roll_deg=-2.5,
        vx_east=0.0,
        vy_up=0.0,
        vz_south=0.0,
        p_rad_s=0.0,
        q_rad_s=0.0,
        r_rad_s=0.0,
    )

    line = format_state_line(state)

    assert "Pitch:" in line
    assert "Heading:" in line
    assert "Roll:" in line
    assert "Aileron Cmd: n/a" in line
    assert "Elevator Cmd: n/a" in line
    assert "Rudder Cmd: n/a" in line


def test_format_state_line_includes_control_commands() -> None:
    state = AircraftState(
        lon_deg=0.0,
        lat_deg=0.0,
        ele_m=0.0,
        agl_m=0.0,
        pitch_deg=1.25,
        heading_deg=180.0,
        roll_deg=-2.5,
        vx_east=0.0,
        vy_up=0.0,
        vz_south=0.0,
        p_rad_s=0.0,
        q_rad_s=0.0,
        r_rad_s=0.0,
    )

    line = format_state_line(
        state,
        ControlCommand(aileron=0.1, elevator=-0.2, rudder=0.3),
    )

    assert "Aileron Cmd: +0.100" in line
    assert "Elevator Cmd: -0.200" in line
    assert "Rudder Cmd: +0.300" in line
