from dataclasses import dataclass

from python_client.models import ControlCommand


@dataclass(frozen=True)
class PilotInputs:
    elevator: float = 0.0
    aileron: float = 0.0
    rudder: float = 0.0
    throttle: float = 0.0


class ManualFlyByWire:
    """Pass pilot inputs through a stable interface for future augmentation."""

    def mix(self, inputs: PilotInputs) -> ControlCommand:
        return ControlCommand(
            elevator=inputs.elevator,
            aileron=inputs.aileron,
            rudder=inputs.rudder,
            throttle=inputs.throttle,
        )
