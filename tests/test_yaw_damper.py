import struct

from python_client.control import YawDamperController, YawDamperGains
from python_client.models import AircraftState, ControlCommand
from python_client.xplane.packets import build_control_command_packet


def make_state(yaw_rate: float) -> AircraftState:
    return AircraftState(
        lon_deg=0.0,
        lat_deg=0.0,
        ele_m=0.0,
        agl_m=0.0,
        pitch_deg=0.0,
        heading_deg=0.0,
        roll_deg=0.0,
        vx_east=0.0,
        vy_up=0.0,
        vz_south=0.0,
        p_rad_s=0.0,
        q_rad_s=0.0,
        r_rad_s=yaw_rate,
    )


def test_yaw_damper_preserves_legacy_proportional_law() -> None:
    controller = YawDamperController(YawDamperGains(proportional_gain=2.0))

    command = controller.compute(make_state(0.25), signal=0.1)

    assert command == ControlCommand(aileron=0.3)


def test_yaw_damper_clips_to_valid_command_range() -> None:
    controller = YawDamperController(YawDamperGains(proportional_gain=10.0))

    command = controller.compute(make_state(0.5), signal=0.0)

    assert command == ControlCommand(aileron=1.0)


def test_build_control_command_packet_populates_control_surface_row() -> None:
    packet = build_control_command_packet(
        ControlCommand(elevator=0.1, aileron=-0.2, rudder=0.3, throttle=0.4)
    )

    header = packet[:5]
    row_index, *values = struct.unpack("<i8f", packet[5:])

    assert header == b"DATA\x00"
    assert row_index == 11
    assert [round(value, 6) for value in values[:4]] == [0.1, -0.2, 0.3, 0.4]
    assert values[4:] == [-999.0, -999.0, -999.0, -999.0]
