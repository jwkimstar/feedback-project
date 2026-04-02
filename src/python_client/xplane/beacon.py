import platform
import socket
import struct

from python_client.config import DEFAULT_NETWORK_CONFIG, NetworkConfig
from python_client.models import XPlaneBeacon
from python_client.xplane.exceptions import XPlaneIpNotFound


def find_xplane(
    wait: float | None = None,
    config: NetworkConfig = DEFAULT_NETWORK_CONFIG,
) -> XPlaneBeacon:
    """Listen for the X-Plane multicast beacon and return the first match."""
    timeout_seconds = wait if wait is not None else config.beacon_wait_seconds

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)

    if platform.system() == "Windows":
        sock.bind(("", config.multicast_port))
    else:
        sock.bind((config.multicast_group, config.multicast_port))

    membership = struct.pack(
        "=4sl",
        socket.inet_aton(config.multicast_group),
        socket.INADDR_ANY,
    )
    sock.setsockopt(socket.IPPROTO_IP, socket.IP_ADD_MEMBERSHIP, membership)
    sock.settimeout(timeout_seconds)

    try:
        while True:
            packet, sender = sock.recvfrom(15000)
            if packet[:5] != b"BECN\x00":
                continue

            data = packet[5:21]
            (
                beacon_major_version,
                beacon_minor_version,
                application_host_id,
                xplane_version_number,
                role,
                port,
            ) = struct.unpack("<BBiiIH", data)

            if not (
                beacon_major_version == 1
                and beacon_minor_version == 2
                and application_host_id == 1
            ):
                continue

            computer_name = packet[21:].split(b"\x00")[0]
            (raknet_port,) = struct.unpack("<H", packet[-2:])
            return XPlaneBeacon(
                ip=sender[0],
                port=port,
                hostname=computer_name.decode("utf-8", errors="replace"),
                xplane_version=xplane_version_number,
                role=role,
                raknet_port=raknet_port,
            )
    except socket.timeout as exc:
        raise XPlaneIpNotFound(
            "Could not find any running X-Plane instance on the network."
        ) from exc
    finally:
        sock.close()
