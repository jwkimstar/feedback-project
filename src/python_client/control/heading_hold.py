from dataclasses import dataclass
from math import radians
from typing import Literal

from python_client.models import AircraftState

ControllerType = Literal["p", "pi", "pd", "pid"]


def _wrapped_heading_error_deg(
    current_heading_deg: float,
    target_heading_deg: float,
) -> float:
    return (target_heading_deg - current_heading_deg + 180.0) % 360.0 - 180.0


def _wrapped_heading_delta_deg(
    previous_heading_deg: float,
    current_heading_deg: float,
) -> float:
    return (current_heading_deg - previous_heading_deg + 180.0) % 360.0 - 180.0


@dataclass(frozen=True)
class HeadingHoldGains:
    proportional_gain: float = 0.25
    integral_gain: float = 0.05
    integral_limit: float = 1.0
    derivative_gain: float = 0.0
    derivative_filter_alpha: float = 1.0


class HeadingHoldController:
    """Prototype heading-hold block preserved from the standalone controller."""

    def __init__(self, gains: HeadingHoldGains | None = None) -> None:
        self.gains = gains or HeadingHoldGains()
        self._integral_error = 0.0
        self._previous_heading_deg: float | None = None
        self._filtered_heading_rate_rad_s = 0.0

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

    def _compute_filtered_heading_rate_rad_s(
        self,
        current_heading_deg: float,
        dt_s: float,
    ) -> float:
        if dt_s <= 0.0 or self._previous_heading_deg is None:
            self._previous_heading_deg = current_heading_deg
            return self._filtered_heading_rate_rad_s

        raw_heading_rate_rad_s = radians(
            _wrapped_heading_delta_deg(self._previous_heading_deg, current_heading_deg)
        ) / dt_s
        alpha = max(0.0, min(1.0, self.gains.derivative_filter_alpha))
        self._filtered_heading_rate_rad_s = (
            alpha * raw_heading_rate_rad_s
            + (1.0 - alpha) * self._filtered_heading_rate_rad_s
        )
        self._previous_heading_deg = current_heading_deg
        return self._filtered_heading_rate_rad_s

    def compute_output_pd(
        self,
        current_heading_deg: float,
        target_heading_deg: float,
        dt_s: float = 0.0,
    ) -> float:
        error = radians(_wrapped_heading_error_deg(current_heading_deg, target_heading_deg))
        heading_rate_rad_s = self._compute_filtered_heading_rate_rad_s(current_heading_deg, dt_s)
        return (
            self.gains.proportional_gain * error
            - self.gains.derivative_gain * heading_rate_rad_s
        )

    def compute_output_pid(
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
        heading_rate_rad_s = self._compute_filtered_heading_rate_rad_s(current_heading_deg, dt_s)
        return (
            self.gains.proportional_gain * error
            + self.gains.integral_gain * self._integral_error
            - self.gains.derivative_gain * heading_rate_rad_s
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
        if controller_type == "pd":
            return self.compute_output_pd(
                state.heading_deg,
                target_heading_deg,
                dt_s=dt_s,
            )
        if controller_type == "pid":
            return self.compute_output_pid(
                state.heading_deg,
                target_heading_deg,
                dt_s=dt_s,
            )
        self._compute_filtered_heading_rate_rad_s(state.heading_deg, dt_s)
        return self.compute_output_p(state.heading_deg, target_heading_deg)

    def reset(self) -> None:
        self._integral_error = 0.0
        self._previous_heading_deg = None
        self._filtered_heading_rate_rad_s = 0.0


# Backward-compatible aliases while callers migrate off the old symbol names.
LegacyHeadingHoldGains = HeadingHoldGains
LegacyHeadingHoldController = HeadingHoldController
