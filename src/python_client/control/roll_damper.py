from dataclasses import dataclass
from typing import Literal

from python_client.models import AircraftState

ControllerType = Literal["p", "pi"]


@dataclass(frozen=True)
class RollDamperGains:
    proportional_gain: float = 0.05
    integral_gain: float = 0.02
    integral_limit: float = 1.0


class RollDamperController:
    """Prototype roll-damper block preserved from the standalone controller."""

    def __init__(self, gains: RollDamperGains | None = None) -> None:
        self.gains = gains or RollDamperGains()
        self._integral_error = 0.0

    def compute_output_p(
        self,
        current_roll_rate_rad_s: float,
        signal: float = 0.0,
    ) -> float:
        return self.gains.proportional_gain * (signal - current_roll_rate_rad_s)

    def compute_output(
        self,
        current_roll_rate_rad_s: float,
        signal: float = 0.0,
    ) -> float:
        return self.compute_output_p(current_roll_rate_rad_s, signal=signal)

    def compute_output_pi(
        self,
        current_roll_rate_rad_s: float,
        signal: float = 0.0,
        dt_s: float = 0.0,
    ) -> float:
        error = signal - current_roll_rate_rad_s
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
        signal: float = 0.0,
        dt_s: float = 0.0,
        controller_type: ControllerType = "p",
    ) -> float:
        if controller_type == "pi":
            return self.compute_output_pi(state.p_rad_s, signal=signal, dt_s=dt_s)
        return self.compute_output_p(state.p_rad_s, signal)

    def reset(self) -> None:
        self._integral_error = 0.0
