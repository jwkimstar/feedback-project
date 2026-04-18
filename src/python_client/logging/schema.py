from dataclasses import dataclass, fields
from math import degrees

from python_client.models import AircraftState, ControlCommand


@dataclass(frozen=True)
class RecordingMetadata:
    hz: int
    control_mode: str | None = None
    yaw_controller_type: str | None = None
    roll_controller_type: str | None = None
    heading_controller_type: str | None = None
    yaw_damper_gain: float | None = None
    yaw_damper_integral_gain: float | None = None
    yaw_damper_derivative_gain: float | None = None
    roll_damper_gain: float | None = None
    roll_damper_integral_gain: float | None = None
    roll_damper_derivative_gain: float | None = None
    roll_damper_max_yaw_rate_deg_s: float | None = None
    heading_hold_gain: float | None = None
    heading_hold_integral_gain: float | None = None
    heading_hold_derivative_gain: float | None = None

    def to_comment_lines(self) -> list[str]:
        return [
            f"# {field.name}={self._format_value(getattr(self, field.name))}"
            for field in fields(self)
        ]

    @staticmethod
    def _format_value(value: object | None) -> str:
        return "" if value is None else str(value)


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
    "p_deg_s",
    "q_deg_s",
    "r_deg_s",
    "aileron_cmd",
    "elevator_cmd",
    "rudder_cmd",
]


def _command_value(value: float | None) -> float | str:
    return "" if value is None else value


def _deg_per_second(value_rad_s: float) -> float:
    return degrees(value_rad_s)


def sample_to_row(
    state: AircraftState,
    command: ControlCommand | None = None,
) -> list[float | str]:
    command = command or ControlCommand()
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
        _deg_per_second(state.p_rad_s),
        _deg_per_second(state.q_rad_s),
        _deg_per_second(state.r_rad_s),
        _command_value(command.aileron),
        _command_value(command.elevator),
        _command_value(command.rudder),
    ]
