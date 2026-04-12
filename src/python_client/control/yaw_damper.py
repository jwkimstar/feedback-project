from dataclasses import dataclass
from typing import Literal, Protocol

from python_client.models import AircraftState, ControlCommand

ControllerType = Literal["p", "pi"]


@dataclass(frozen=True)
class YawDamperGains:
    proportional_gain: float = 1.0
    integral_gain: float = 0.15
    integral_limit: float = 1.0
    anti_windup_gain: float = 1.0
    min_aileron: float = -1.0
    max_aileron: float = 1.0


class ControlCommandSender(Protocol):
    def send_control_command(self, command: ControlCommand) -> None:
        """Transmit a control command to X-Plane."""


class YawDamperController:
    """Preserve the legacy yaw-damper control law from the prototype script."""

    def __init__(self, gains: YawDamperGains | None = None) -> None:
        self.gains = gains or YawDamperGains()
        self._integral_term = 0.0

    def _clamp_aileron(self, aileron: float) -> float:
        return max(self.gains.min_aileron, min(self.gains.max_aileron, aileron))

    def _clamp_integral_term(self, integral_term: float) -> float:
        return max(-self.gains.integral_limit, min(self.gains.integral_limit, integral_term))

    def compute_output_p(
        self,
        current_yaw_rate_rad_s: float,
        signal: float = 0.0,
    ) -> float:
        aileron = self.gains.proportional_gain * (signal - current_yaw_rate_rad_s)
        return self._clamp_aileron(aileron)

    def compute_output(
        self,
        current_yaw_rate_rad_s: float,
        signal: float = 0.0,
    ) -> float:
        return self.compute_output_p(current_yaw_rate_rad_s, signal=signal)

    def compute_output_pi(
        self,
        current_yaw_rate_rad_s: float,
        signal: float = 0.0,
        dt_s: float = 0.0,
    ) -> float:
        error = signal - current_yaw_rate_rad_s
        proportional_term = self.gains.proportional_gain * error
        integral_term = self._integral_term

        if dt_s > 0.0:
            integral_term += self.gains.integral_gain * error * dt_s
            unsaturated_aileron = proportional_term + integral_term
            saturated_aileron = self._clamp_aileron(unsaturated_aileron)
            integral_term += self.gains.anti_windup_gain * (
                saturated_aileron - unsaturated_aileron
            )
            integral_term = self._clamp_integral_term(integral_term)
            self._integral_term = integral_term

        aileron = proportional_term + self._integral_term
        return self._clamp_aileron(aileron)

    def compute(
        self,
        state: AircraftState,
        signal: float = 0.0,
        dt_s: float = 0.0,
        controller_type: ControllerType = "p",
    ) -> ControlCommand:
        if controller_type == "pi":
            aileron = self.compute_output_pi(state.r_rad_s, signal=signal, dt_s=dt_s)
        else:
            aileron = self.compute_output_p(state.r_rad_s, signal=signal)
        return ControlCommand(aileron=aileron)

    def reset(self) -> None:
        self._integral_term = 0.0


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
