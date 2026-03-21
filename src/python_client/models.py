from dataclasses import dataclass


@dataclass(frozen=True)
class XPlaneBeacon:
    ip: str
    port: int
    hostname: str
    xplane_version: int
    role: int
    raknet_port: int


@dataclass(frozen=True)
class AircraftState:
    lon_deg: float
    lat_deg: float
    ele_m: float
    agl_m: float
    pitch_deg: float
    heading_deg: float
    roll_deg: float
    vx_east: float
    vy_up: float
    vz_south: float
    p_rad_s: float
    q_rad_s: float
    r_rad_s: float


@dataclass(frozen=True)
class ControlCommand:
    elevator: float | None = None
    aileron: float | None = None
    rudder: float | None = None
    throttle: float | None = None
