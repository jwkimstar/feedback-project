from python_client.models import AircraftState


CSV_HEADER = [
    "lon_deg",
    "lat_deg",
    "ele_m",
    "agl_m",
    "pitch_deg",
    "heading_deg",
    "roll_deg",
    "vx_east",
    "vy_up",
    "vz_south",
    "p_rad_s",
    "q_rad_s",
    "r_rad_s",
]


def sample_to_row(state: AircraftState) -> list[float]:
    return [
        state.lon_deg,
        state.lat_deg,
        state.ele_m,
        state.agl_m,
        state.pitch_deg,
        state.heading_deg,
        state.roll_deg,
        state.vx_east,
        state.vy_up,
        state.vz_south,
        state.p_rad_s,
        state.q_rad_s,
        state.r_rad_s,
    ]
