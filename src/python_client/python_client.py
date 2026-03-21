import socket
import struct
import platform
import time


class XPlaneIpNotFound(Exception):
    pass


def find_xp(wait=3.0):
    """
    Listen for the X-Plane multicast beacon and return info about
    the first X-Plane instance found.
    """
    MCAST_GRP = "239.255.1.1"
    MCAST_PORT = 49707

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)

    if platform.system() == "Windows":
        sock.bind(("", MCAST_PORT))
    else:
        sock.bind((MCAST_GRP, MCAST_PORT))

    mreq = struct.pack("=4sl", socket.inet_aton(MCAST_GRP), socket.INADDR_ANY)
    sock.setsockopt(socket.IPPROTO_IP, socket.IP_ADD_MEMBERSHIP, mreq)

    sock.settimeout(wait)

    try:
        while True:
            packet, sender = sock.recvfrom(15000)

            # Beacon packets begin with BECN\0
            if packet[:5] != b"BECN\x00":
                continue

            # Fixed part of beacon payload
            data = packet[5:21]
            (
                beacon_major_version,
                beacon_minor_version,
                application_host_id,
                xplane_version_number,
                role,
                port,
            ) = struct.unpack("<BBiiIH", data)

            # Only accept X-Plane beacons
            if not (
                beacon_major_version == 1
                and beacon_minor_version == 2
                and application_host_id == 1
            ):
                continue

            computer_name = packet[21:]
            computer_name = computer_name.split(b"\x00")[0]
            (raknet_port,) = struct.unpack("<H", packet[-2:])

            return {
                "ip": sender[0],
                "port": port,
                "hostname": computer_name.decode("utf-8", errors="replace"),
                "xplane_version": xplane_version_number,
                "role": role,
                "raknet_port": raknet_port,
            }

    except socket.timeout:
        raise XPlaneIpNotFound("Could not find any running X-Plane instance on the network.")
    finally:
        sock.close()


def request_rpos(sock, xp_ip, xp_port, hz=1):
    """
    Ask X-Plane to send RPOS packets at 'hz' times per second.
    """
    msg = struct.pack("<4sx10s", b"RPOS", str(hz).encode("ascii"))
    sock.sendto(msg, (xp_ip, xp_port))


def stop_rpos(sock, xp_ip, xp_port):
    """
    Stop RPOS streaming.
    """
    msg = struct.pack("<4sx10s", b"RPOS", b"0")
    sock.sendto(msg, (xp_ip, xp_port))


def parse_rpos(packet):
    """
    Parse one RPOS packet.

    Returns:
        dict with lat/lon/elevation and orientation
    """
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

    return {
        "lon_deg": lon_deg,
        "lat_deg": lat_deg,
        "ele_m": ele_m,
        "agl_m": agl_m,
        "pitch_deg": pitch_deg,
        "heading_deg": heading_deg,
        "roll_deg": roll_deg,
        "vx_east": vx_east,
        "vy_up": vy_up,
        "vz_south": vz_south,
        "p_rad_s": p_rad_s,
        "q_rad_s": q_rad_s,
        "r_rad_s": r_rad_s,
    }


def main():
    try:
        beacon = find_xp(wait=5.0)
    except XPlaneIpNotFound as e:
        print(e)
        print("If you are using X-Plane 12, check Settings > Network > Accept incoming connections.")
        return

    xp_ip = beacon["ip"]
    xp_port = beacon["port"]

    print("Found X-Plane:")
    print(f"  Hostname: {beacon['hostname']}")
    print(f"  IP:       {xp_ip}")
    print(f"  UDP port: {xp_port}")
    print(f"  Version:  {beacon['xplane_version']}")
    print()

    # Separate UDP socket for normal communication with X-Plane
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(("", 0))          # let OS choose a local port
    sock.settimeout(2.0)

    try:
        # Subscribe to RPOS once per second
        request_rpos(sock, xp_ip, xp_port, hz=1)

        print("Printing aircraft orientation at 1 Hz. Press Ctrl+C to stop.\n")

        while True:
            packet, addr = sock.recvfrom(2048)

            # Ignore packets from other sources
            if addr[0] != xp_ip:
                continue

            if packet[:4] != b"RPOS":
                continue

            state = parse_rpos(packet)

            print(
                f"Pitch: {state['pitch_deg']:8.3f} deg | "
                f"Heading: {state['heading_deg']:8.3f} deg | "
                f"Roll: {state['roll_deg']:8.3f} deg"
            )

    except KeyboardInterrupt:
        print("\nStopping...")
    except socket.timeout:
        print("Timed out waiting for RPOS data from X-Plane.")
    finally:
        try:
            stop_rpos(sock, xp_ip, xp_port)
        except Exception:
            pass
        sock.close()


if __name__ == "__main__":
    main()