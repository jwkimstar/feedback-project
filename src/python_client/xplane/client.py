import socket
from dataclasses import dataclass
from typing import Iterator

from python_client.config import DEFAULT_NETWORK_CONFIG, NetworkConfig
from python_client.models import AircraftState, ControlCommand, XPlaneBeacon
from python_client.xplane.beacon import find_xplane
from python_client.xplane.packets import (
    build_control_command_packet,
    build_rpos_request,
    build_rpos_stop,
    parse_rpos_packet,
)


@dataclass
class XPlaneClient:
    beacon: XPlaneBeacon
    sock: socket.socket
    send_sock: socket.socket
    hz: int
    config: NetworkConfig = DEFAULT_NETWORK_CONFIG

    @classmethod
    def discover(
        cls,
        wait: float | None = None,
        hz: int | None = None,
        config: NetworkConfig = DEFAULT_NETWORK_CONFIG,
    ) -> "XPlaneClient":
        beacon = find_xplane(wait=wait, config=config)
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.bind(("", 0))
        sock.settimeout(config.socket_timeout_seconds)
        send_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        return cls(
            beacon=beacon,
            sock=sock,
            send_sock=send_sock,
            hz=hz or config.rpos_hz,
            config=config,
        )

    def __enter__(self) -> "XPlaneClient":
        self.start_stream()
        return self

    def __exit__(self, exc_type, exc, tb) -> None:
        self.close()

    def start_stream(self) -> None:
        self.sock.sendto(build_rpos_request(self.hz), (self.beacon.ip, self.beacon.port))

    def stop_stream(self) -> None:
        self.sock.sendto(build_rpos_stop(), (self.beacon.ip, self.beacon.port))

    def send_control_command(self, command: ControlCommand) -> None:
        packet = build_control_command_packet(command)
        command_port = self.config.command_port or self.beacon.port
        self.send_sock.sendto(packet, (self.beacon.ip, command_port))

    def recv_state(self) -> AircraftState:
        packet = self._recv_next_matching_rpos_packet()
        packet = self._drain_pending_rpos_packets(packet)
        return parse_rpos_packet(packet)

    def _recv_next_matching_rpos_packet(self) -> bytes:
        while True:
            try:
                packet, addr = self.sock.recvfrom(2048)
            except socket.timeout as exc:
                raise TimeoutError("Timed out waiting for RPOS data from X-Plane.") from exc

            if addr[0] != self.beacon.ip:
                continue

            if packet[:4] != b"RPOS":
                continue

            return packet

    def _drain_pending_rpos_packets(self, latest_packet: bytes) -> bytes:
        original_timeout = self.sock.gettimeout()
        self.sock.setblocking(False)
        try:
            while True:
                packet, addr = self.sock.recvfrom(2048)
                if addr[0] != self.beacon.ip:
                    continue
                if packet[:4] != b"RPOS":
                    continue
                latest_packet = packet
        except (BlockingIOError, socket.timeout):
            return latest_packet
        finally:
            self.sock.settimeout(original_timeout)

    def stream_states(self) -> Iterator[AircraftState]:
        while True:
            yield self.recv_state()

    def close(self) -> None:
        try:
            self.stop_stream()
        except OSError:
            pass
        self.sock.close()
        self.send_sock.close()
