from dataclasses import dataclass

from python_client.models import AircraftState


@dataclass(frozen=True)
class RollDamperGains:
    proportional_gain: float = 0.05


class RollDamperController:
    """Prototype roll-damper block preserved from the standalone controller."""

    def __init__(self, gains: RollDamperGains | None = None) -> None:
        self.gains = gains or RollDamperGains()

    def compute_output(
        self,
        current_roll_rate_rad_s: float,
        signal: float = 0.0,
    ) -> float:
        return self.gains.proportional_gain * (signal - current_roll_rate_rad_s)

    def compute(
        self,
        state: AircraftState,
        signal: float = 0.0,
    ) -> float:
        return self.compute_output(state.p_rad_s, signal)
