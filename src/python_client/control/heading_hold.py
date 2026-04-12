from dataclasses import dataclass
from math import radians
from typing import Literal

from python_client.models import AircraftState

ControllerType = Literal["p", "pi"]


def _wrapped_heading_error_deg(
    current_heading_deg: float,
    target_heading_deg: float,
) -> float:
    return (target_heading_deg - current_heading_deg + 180.0) % 360.0 - 180.0


@dataclass(frozen=True)
class HeadingHoldGains:
    proportional_gain: float = 0.25
    integral_gain: float = 0.05
    integral_limit: float = 1.0


class HeadingHoldController:
    """Prototype heading-hold block preserved from the standalone controller."""

    def __init__(self, gains: HeadingHoldGains | None = None) -> None:
        self.gains = gains or HeadingHoldGains()
        self._integral_error = 0.0

    def compute_output_p(
        self,
        current_heading_deg: float,
        target_heading_deg: float,
    ) -> float:
        return self.gains.proportional_gain * radians(
            _wrapped_heading_error_deg(current_heading_deg, target_heading_deg)
        )

    def compute_output(
        self,
        current_heading_deg: float,
        target_heading_deg: float,
    ) -> float:
        return self.compute_output_p(current_heading_deg, target_heading_deg)

    def compute_output_pi(
        self,
        current_heading_deg: float,
        target_heading_deg: float,
        dt_s: float = 0.0,
    ) -> float:
        error = radians(_wrapped_heading_error_deg(current_heading_deg, target_heading_deg))
        if dt_s > 0.0:
            self._integral_error += error * dt_s
            self._integral_error = max(
                -self.gains.integral_limit,
                min(self.gains.integral_limit, self._integral_error),
            )
        return (
            self.gains.proportional_gain * error
            + self.gains.integral_gain * self._integral_error
        )

    def compute(
        self,
        state: AircraftState,
        target_heading_deg: float,
        dt_s: float = 0.0,
        controller_type: ControllerType = "p",
    ) -> float:
        if controller_type == "pi":
            return self.compute_output_pi(
                state.heading_deg,
                target_heading_deg,
                dt_s=dt_s,
            )
        return self.compute_output_p(state.heading_deg, target_heading_deg)

    def reset(self) -> None:
        self._integral_error = 0.0


# Backward-compatible aliases while callers migrate off the old symbol names.
LegacyHeadingHoldGains = HeadingHoldGains
LegacyHeadingHoldController = HeadingHoldController
