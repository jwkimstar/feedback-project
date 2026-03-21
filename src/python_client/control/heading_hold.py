from dataclasses import dataclass
from math import pi

from python_client.models import AircraftState, ControlCommand


@dataclass(frozen=True)
class HeadingHoldGains:
    heading_gain: float = 1.2
    roll_proportional_gain: float = 2.5
    roll_derivative_gain: float = 1.0
    max_bank_deg: float = 30.0


class HeadingHoldController:
    """Prototype controller that commands aileron from heading error."""

    def __init__(self, gains: HeadingHoldGains | None = None) -> None:
        self.gains = gains or HeadingHoldGains()

    def compute(
        self,
        state: AircraftState,
        target_heading_deg: float,
    ) -> ControlCommand:
        heading_error_deg = ((target_heading_deg - state.heading_deg + 180.0) % 360.0) - 180.0
        desired_roll_deg = self.gains.heading_gain * heading_error_deg
        desired_roll_deg = max(
            -self.gains.max_bank_deg,
            min(self.gains.max_bank_deg, desired_roll_deg),
        )

        roll_error_deg = desired_roll_deg - state.roll_deg
        roll_rate_deg = state.p_rad_s * (180.0 / pi)
        aileron = (
            self.gains.roll_proportional_gain * roll_error_deg / self.gains.max_bank_deg
            - self.gains.roll_derivative_gain * roll_rate_deg / self.gains.max_bank_deg
        )
        aileron = max(-1.0, min(1.0, aileron))
        return ControlCommand(aileron=aileron)
