from dataclasses import dataclass
from typing import Protocol

from python_client.models import AircraftState, ControlCommand


@dataclass(frozen=True)
class YawDamperGains:
    proportional_gain: float = 1.0
    min_aileron: float = -1.0
    max_aileron: float = 1.0


class ControlCommandSender(Protocol):
    def send_control_command(self, command: ControlCommand) -> None:
        """Transmit a control command to X-Plane."""


class YawDamperController:
    """Preserve the legacy yaw-damper control law from the prototype script."""

    def __init__(self, gains: YawDamperGains | None = None) -> None:
        self.gains = gains or YawDamperGains()

    def compute_output(
        self,
        current_yaw_rate_rad_s: float,
        signal: float = 0.0,
    ) -> float:
        aileron = self.gains.proportional_gain * (signal - current_yaw_rate_rad_s)
        return max(self.gains.min_aileron, min(self.gains.max_aileron, aileron))

    def compute(
        self,
        state: AircraftState,
        signal: float = 0.0,
    ) -> ControlCommand:
        aileron = self.compute_output(state.r_rad_s, signal=signal)
        return ControlCommand(aileron=aileron)


class YawDamperHandler:
    """Apply the yaw damper on each telemetry sample and send the aileron command."""

    def __init__(
        self,
        sender: ControlCommandSender,
        controller: YawDamperController | None = None,
        signal: float = 0.0,
        print_status: bool = False,
    ) -> None:
        self.sender = sender
        self.controller = controller or YawDamperController()
        self.signal = signal
        self.print_status = print_status
        self._latest_command: ControlCommand | None = None

    def handle_state(self, state: AircraftState) -> None:
        command = self.controller.compute(
            state,
            signal=self.signal,
        )
        self._latest_command = command
        self.sender.send_control_command(command)

        if self.print_status and command.aileron is not None:
            print(
                f"Yaw Rate: {state.r_rad_s:+.4f} rad/s | "
                f"aileron Cmd: {command.aileron:+.3f}",
                end="\r",
            )

    def get_latest_command(self) -> ControlCommand | None:
        return self._latest_command

    def close(self) -> None:
        return None
