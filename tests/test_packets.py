import struct
import socket

from python_client.config import NetworkConfig
from python_client.models import ControlCommand, XPlaneBeacon
from python_client.xplane.client import XPlaneClient
from python_client.xplane.packets import build_control_command_packet
from python_client.xplane.packets import parse_rpos_packet
from python_client.runtime import StopRequested, run_client_to_handlers


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


class DummySocket:
    def __init__(self) -> None:
        self.calls: list[tuple[bytes, tuple[str, int]]] = []
        self.timeout: float | None = None

    def sendto(self, packet: bytes, destination: tuple[str, int]) -> None:
        self.calls.append((packet, destination))

    def gettimeout(self) -> float | None:
        return self.timeout

    def setblocking(self, flag: bool) -> None:
        self.timeout = None if flag else 0.0

    def settimeout(self, value: float | None) -> None:
        self.timeout = value


def test_send_control_command_uses_discovered_beacon_port_by_default() -> None:
    recv_sock = DummySocket()
    send_sock = DummySocket()
    client = XPlaneClient(
        beacon=XPlaneBeacon(
            ip="192.168.1.10",
            port=49005,
            hostname="xplane",
            xplane_version=120000,
            role=1,
            raknet_port=0,
        ),
        sock=recv_sock,
        send_sock=send_sock,
        hz=10,
        config=NetworkConfig(command_port=None),
    )

    client.send_control_command(ControlCommand(aileron=0.25))

    assert len(send_sock.calls) == 1
    _packet, destination = send_sock.calls[0]
    assert destination == ("192.168.1.10", 49005)


def test_send_control_command_honors_explicit_override_port() -> None:
    recv_sock = DummySocket()
    send_sock = DummySocket()
    client = XPlaneClient(
        beacon=XPlaneBeacon(
            ip="192.168.1.10",
            port=49005,
            hostname="xplane",
            xplane_version=120000,
            role=1,
            raknet_port=0,
        ),
        sock=recv_sock,
        send_sock=send_sock,
        hz=10,
        config=NetworkConfig(command_port=49000),
    )

    client.send_control_command(ControlCommand(aileron=0.25))

    assert len(send_sock.calls) == 1
    _packet, destination = send_sock.calls[0]
    assert destination == ("192.168.1.10", 49000)


def _make_rpos_packet(heading_deg: float) -> bytes:
    return struct.pack(
        "<4sxdddffffffffff",
        b"RPOS",
        1.0,
        2.0,
        3.0,
        4.0,
        5.0,
        heading_deg,
        7.0,
        8.0,
        9.0,
        10.0,
        11.0,
        12.0,
        13.0,
    )


class DummyRecvSocket(DummySocket):
    def __init__(self, packets: list[tuple[bytes, tuple[str, int]]]) -> None:
        super().__init__()
        self.packets = packets

    def recvfrom(self, _size: int) -> tuple[bytes, tuple[str, int]]:
        if self.packets:
            return self.packets.pop(0)
        if self.timeout == 0.0:
            raise BlockingIOError
        raise socket.timeout


def test_recv_state_uses_newest_pending_rpos_packet() -> None:
    recv_sock = DummyRecvSocket(
        [
            (_make_rpos_packet(100.0), ("192.168.1.10", 49005)),
            (_make_rpos_packet(110.0), ("192.168.1.10", 49005)),
            (b"NOPE", ("192.168.1.10", 49005)),
            (_make_rpos_packet(120.0), ("192.168.1.10", 49005)),
        ]
    )
    recv_sock.settimeout(2.0)
    send_sock = DummySocket()
    client = XPlaneClient(
        beacon=XPlaneBeacon(
            ip="192.168.1.10",
            port=49005,
            hostname="xplane",
            xplane_version=120000,
            role=1,
            raknet_port=0,
        ),
        sock=recv_sock,
        send_sock=send_sock,
        hz=10,
        config=NetworkConfig(command_port=None),
    )

    state = client.recv_state()

    assert state.heading_deg == 120.0


class DummyHandler:
    def __init__(self) -> None:
        self.states = 0

    def handle_state(self, _state) -> None:
        self.states += 1

    def close(self) -> None:
        return None


class DummyClient:
    def recv_state(self):
        return object()


def test_run_client_to_handlers_stops_when_requested() -> None:
    client = DummyClient()
    handler = DummyHandler()

    def should_stop() -> bool:
        return True

    try:
        run_client_to_handlers(client, [handler], should_stop=should_stop)
    except StopRequested:
        pass
    else:
        raise AssertionError("Expected StopRequested to be raised.")

    assert handler.states == 0
