import socket
import struct
import numpy as np
import time
import sys
import select
import termios
import tty

# ==============================
# MACOS NON-BLOCKING KEY INPUT
# ==============================
def setup_terminal():
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    tty.setcbreak(fd)  # character-by-character input
    return fd, old_settings

def restore_terminal(fd, old_settings):
    termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)

def get_key_nonblocking():
    dr, _, _ = select.select([sys.stdin], [], [], 0)
    if dr:
        return sys.stdin.read(1)
    return None

# ==============================
# NETWORK SETTINGS
# ==============================
RECV_PORT = 49100
SEND_PORT = 49000
XPLANE_IP = "127.0.0.1"

# ==============================
# CREATE SOCKETS
# ==============================
recv_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
recv_sock.bind(("0.0.0.0", RECV_PORT))
recv_sock.setblocking(False)

send_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

print("Listening to X-Plane via DATA packets...")

# ==============================
# INITIALIZE STATE
# ==============================
phi   = 0.0   # roll (rad)
theta = 0.0   # pitch (rad)
psi   = 0.0   # true heading (rad)
p     = 0.0   # roll rate (rad/s)
q     = 0.0   # pitch rate (rad/s)
r     = 0.0   # yaw rate (rad/s)

# ==============================
# DESIRED HEADING
# ==============================
psi_des = np.deg2rad(30)

# ==============================
# CONTROL GAINS
# ==============================
K_psi        = 1.2
MAX_BANK_DEG = 30.0
Kp_roll      = 2.5
Kd_roll      = 1.0

# ==============================
# CONTROLLER TOGGLE
# ==============================
controller_active = False
print("Controller OFF. Press 'c' to toggle ON/OFF.")
print("Press 'q' to quit.")
print("NOTE: Make sure aircraft is AIRBORNE and in level flight before activating.\n")

fd, old_settings = setup_terminal()

try:
    # ==============================
    # MAIN LOOP
    # ==============================
    while True:
        aileron_cmd = 0.0

        # --- Keyboard toggle ---
        key = get_key_nonblocking()
        if key is not None:
            key = key.lower()
            if key == 'c':
                controller_active = not controller_active
                print(f"\nController is now {'ACTIVE' if controller_active else 'INACTIVE'}.")
            elif key == 'q':
                print("\nQuitting...")
                break

        # --- Drain all pending UDP packets ---
        while True:
            try:
                data, addr = recv_sock.recvfrom(4096)
            except BlockingIOError:
                break

            if not data or data[:4] != b'DATA':
                continue

            # DATA packet: 'DATA*' (5 bytes) + groups of 36 bytes
            payload = data[5:]
            num_groups = len(payload) // 36

            for i in range(num_groups):
                chunk = payload[i * 36:(i + 1) * 36]
                if len(chunk) < 36:
                    break

                group_id = struct.unpack('<i', chunk[:4])[0]
                values = struct.unpack('<8f', chunk[4:])

                if group_id == 17:
                    phi   = np.deg2rad(values[0])
                    theta = np.deg2rad(values[1])
                    psi   = np.deg2rad(values[2])

                elif group_id == 16:
                    p = np.deg2rad(values[0])
                    q = np.deg2rad(values[1])
                    r = np.deg2rad(values[2])

        # --- Controller ---
        if controller_active:
            # Outer loop: heading error -> desired bank
            e_psi = psi_des - psi
            e_psi = (e_psi + np.pi) % (2 * np.pi) - np.pi

            phi_des = K_psi * e_psi
            phi_des = np.clip(
                phi_des,
                np.deg2rad(-MAX_BANK_DEG),
                np.deg2rad(MAX_BANK_DEG)
            )

            # Inner loop: roll error -> aileron
            e_phi = phi_des - phi
            aileron_cmd = Kp_roll * e_phi - Kd_roll * p
            aileron_cmd = np.clip(aileron_cmd, -1.0, 1.0)

            # Send to X-Plane
            packet = b'DATA\x00' + struct.pack(
                '<i8f',
                11,
                -999.0,       # elevator
                aileron_cmd,  # aileron
                -999.0,       # rudder
                -999.0, -999.0, -999.0, -999.0, -999.0
            )
            send_sock.sendto(packet, (XPLANE_IP, SEND_PORT))

        # --- Debug print ---
        e_psi_deg = np.rad2deg((psi_des - psi + np.pi) % (2 * np.pi) - np.pi)
        print(
            f"\rHDG: {np.rad2deg(psi):6.1f}°  "
            f"Err: {e_psi_deg:+6.1f}°  "
            f"Roll: {np.rad2deg(phi):6.1f}°  "
            f"p: {np.rad2deg(p):+6.1f}°/s  "
            f"Ail: {aileron_cmd:+.3f}  "
            f"Ctrl: {'ON ' if controller_active else 'OFF'}",
            end="",
            flush=True
        )

        time.sleep(0.01)

finally:
    restore_terminal(fd, old_settings)
    recv_sock.close()
    send_sock.close()
    print("\nTerminal restored.")
    