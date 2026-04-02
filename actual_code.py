import socket
import struct
import platform
import time
import numpy as np


class XPlaneIpNotFound(Exception):
    pass


def find_xp(wait=3.0):
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
    msg = struct.pack("<4sx10s", b"RPOS", str(hz).encode("ascii"))
    sock.sendto(msg, (xp_ip, xp_port))


def stop_rpos(sock, xp_ip, xp_port):
    msg = struct.pack("<4sx10s", b"RPOS", b"0")
    sock.sendto(msg, (xp_ip, xp_port))


def parse_rpos(packet):
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


def yaw_damper(current_yaw_rate, signal, k_p_yaw_damper):
    output = k_p_yaw_damper * (current_yaw_rate - signal)
    output = np.clip(output, -1.0, 1.0)
    return output

def roll_damper(current_roll_rate, signal, k_p_roll_damper):
    output = k_p_roll_damper * (current_roll_rate - signal)
    return output

def heading_hold(current_heading, target_heading, k_p_heading_hold):
    output = k_p_heading_hold * (current_heading - target_heading)
    return output


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

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(("", 0))
    sock.settimeout(2.0)

    # --- Controller socket ---
    SEND_PORT = 49000
    send_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    # --- Controller params ---
    damper_active = True
    k_p_yaw_damper = 9.0
    k_p_roll_damper = 0.05
    k_p_heading_hold = 0.35


    try:
        request_rpos(sock, xp_ip, xp_port, hz=1)

        print("Running... Press Ctrl+C to stop.\n")

        while True:
            packet, addr = sock.recvfrom(2048)

            if addr[0] != xp_ip:
                continue

            if packet[:4] != b"RPOS":
                continue

            state = parse_rpos(packet)

            q = state["q_rad_s"] # pitch rate in radians per second
            p = state["p_rad_s"] # roll rate in radians per second
            r = state["r_rad_s"] # yaw rate in radians per second
            heading_deg = state["heading_deg"]

            # --- Controller ---
            if damper_active:
                roll_damper_input = heading_hold(heading_deg, 0, k_p_heading_hold)
                yaw_damper_input = roll_damper(p, roll_damper_input, k_p_roll_damper)
                aileron_cmd = yaw_damper(r, yaw_damper_input, k_p_yaw_damper)

                pkt = b'DATA\x00' + struct.pack(
                    '<i8f',
                    11,
                    -999.0, float(aileron_cmd), -999.0,   # aileron is field [1]
                    -999.0, -999.0, -999.0, -999.0, -999.0
                )
                send_sock.sendto(pkt, (xp_ip, SEND_PORT))

                print(
                    f"Yaw Rate: {np.rad2deg(r):+6.2f}°/s | "
                    f"Aileron Cmd: {aileron_cmd:+.3f}   ",
                    end='\r'
                )
            else:
                print(
                    f"Yaw Rate: {np.rad2deg(r):+6.2f}°/s | Damper OFF   ",
                    end='\r'
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
        send_sock.close()


if __name__ == "__main__":
    main()