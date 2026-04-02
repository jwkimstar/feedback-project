from dataclasses import dataclass

from python_client.models import AircraftState


@dataclass(frozen=True)
class LegacyHeadingHoldGains:
    proportional_gain: float = 0.35


class LegacyHeadingHoldController:
    """Prototype heading-hold block preserved from the standalone controller."""

    def __init__(self, gains: LegacyHeadingHoldGains | None = None) -> None:
        self.gains = gains or LegacyHeadingHoldGains()

    def compute_output(
        self,
        current_heading_deg: float,
        target_heading_deg: float,
    ) -> float:
        return self.gains.proportional_gain * (target_heading_deg - current_heading_deg)

    def compute(
        self,
        state: AircraftState,
        target_heading_deg: float,
    ) -> float:
        return self.compute_output(state.heading_deg, target_heading_deg)
