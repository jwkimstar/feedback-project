import struct

from python_client.models import AircraftState, ControlCommand


CONTROL_SURFACE_DATA_INDEX = 11
UNSET_CONTROL_VALUE = -999.0


def build_rpos_request(hz: int) -> bytes:
    return struct.pack("<4sx10s", b"RPOS", str(hz).encode("ascii"))


def build_rpos_stop() -> bytes:
    return struct.pack("<4sx10s", b"RPOS", b"0")


def build_control_command_packet(command: ControlCommand) -> bytes:
    values = [UNSET_CONTROL_VALUE] * 8

    if command.elevator is not None:
        values[0] = float(command.elevator)
    if command.aileron is not None:
        values[1] = float(command.aileron)
    if command.rudder is not None:
        values[2] = float(command.rudder)
    if command.throttle is not None:
        values[3] = float(command.throttle)

    return b"DATA\x00" + struct.pack("<i8f", CONTROL_SURFACE_DATA_INDEX, *values)


def parse_rpos_packet(packet: bytes) -> AircraftState:
    if packet[:4] != b"RPOS":
        raise ValueError("Received non-RPOS packet")

    (
        _header,
        lon_deg,
        lat_deg,
        ele_m,
        agl_m,
        pitch_deg,
        heading_deg,
        roll_deg,
        vx_east,
        vy_up,
        vz_south,
        p_rad_s,
        q_rad_s,
        r_rad_s,
    ) = struct.unpack("<4sxdddffffffffff", packet)

    return AircraftState(
        lon_deg=lon_deg,
        lat_deg=lat_deg,
        ele_m=ele_m,
        agl_m=agl_m,
        pitch_deg=pitch_deg,
        heading_deg=heading_deg,
        roll_deg=roll_deg,
        vx_east=vx_east,
        vy_up=vy_up,
        vz_south=vz_south,
        p_rad_s=p_rad_s,
        q_rad_s=q_rad_s,
        r_rad_s=r_rad_s,
    )
