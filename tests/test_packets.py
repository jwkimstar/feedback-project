import struct

from python_client.xplane.packets import parse_rpos_packet


def test_parse_rpos_packet() -> None:
    packet = struct.pack(
        "<4sxdddffffffffff",
        b"RPOS",
        1.0,
        2.0,
        3.0,
        4.0,
        5.0,
        6.0,
        7.0,
        8.0,
        9.0,
        10.0,
        11.0,
        12.0,
        13.0,
    )

    state = parse_rpos_packet(packet)

    assert state.heading_deg == 6.0
    assert state.roll_deg == 7.0
    assert state.r_rad_s == 13.0
