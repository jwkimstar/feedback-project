from dataclasses import dataclass
from enum import Enum
from math import degrees
from typing import Protocol

from python_client.control.legacy_heading_hold import (
    LegacyHeadingHoldController,
    LegacyHeadingHoldGains,
)
from python_client.control.roll_damper import RollDamperController, RollDamperGains
from python_client.control.yaw_damper import YawDamperController, YawDamperGains
from python_client.models import AircraftState, ControlCommand


class ControlCommandSender(Protocol):
    def send_control_command(self, command: ControlCommand) -> None:
        """Transmit a control command to X-Plane."""


class MasterControllerMode(str, Enum):
    YAW_DAMPER = "yaw-damper"
    YAW_ROLL_DAMPER = "yaw-roll-damper"
    YAW_ROLL_HEADING_HOLD = "yaw-roll-heading-hold"


@dataclass(frozen=True)
class MasterControllerGains:
    yaw_damper_gain: float = 2.0
    roll_damper_gain: float = 0.5
    heading_hold_gain: float = 0.35


@dataclass(frozen=True)
class MasterControllerTargets:
    desired_yaw_rate_rad_s: float = 0.0
    desired_roll_rate_rad_s: float = 0.0
    target_heading_deg: float = 0.0


@dataclass(frozen=True)
class MasterControllerTrace:
    heading_hold_output: float | None
    roll_damper_output: float | None
    yaw_damper_signal: float
    aileron_command: float


class MasterController:
    """Compose the prototype control blocks in the same order as actual_code.py."""

    def __init__(
        self,
        mode: MasterControllerMode = MasterControllerMode.YAW_DAMPER,
        gains: MasterControllerGains | None = None,
        targets: MasterControllerTargets | None = None,
    ) -> None:
        self.mode = mode
        self.gains = gains or MasterControllerGains()
        self.targets = targets or MasterControllerTargets()
        self.yaw_damper = YawDamperController(
            YawDamperGains(proportional_gain=self.gains.yaw_damper_gain)
        )
        self.roll_damper = RollDamperController(
            RollDamperGains(proportional_gain=self.gains.roll_damper_gain)
        )
        self.heading_hold = LegacyHeadingHoldController(
            LegacyHeadingHoldGains(proportional_gain=self.gains.heading_hold_gain)
        )
        self._latest_trace: MasterControllerTrace | None = None

    def compute(self, state: AircraftState) -> ControlCommand:
        heading_hold_output = None
        roll_damper_output = None
        yaw_damper_signal = self.targets.desired_yaw_rate_rad_s

        if self.mode is MasterControllerMode.YAW_ROLL_HEADING_HOLD:
            heading_hold_output = self.heading_hold.compute(
                state,
                target_heading_deg=self.targets.target_heading_deg,
            )
            roll_damper_output = self.roll_damper.compute(state, signal=heading_hold_output)
            yaw_damper_signal = roll_damper_output
        elif self.mode is MasterControllerMode.YAW_ROLL_DAMPER:
            roll_damper_output = self.roll_damper.compute(
                state,
                signal=self.targets.desired_roll_rate_rad_s,
            )
            yaw_damper_signal = roll_damper_output

        command = self.yaw_damper.compute(state, signal=yaw_damper_signal)
        self._latest_trace = MasterControllerTrace(
            heading_hold_output=heading_hold_output,
            roll_damper_output=roll_damper_output,
            yaw_damper_signal=yaw_damper_signal,
            aileron_command=command.aileron or 0.0,
        )
        return command

    def get_latest_trace(self) -> MasterControllerTrace | None:
        return self._latest_trace


class MasterControllerHandler:
    """Run the composed prototype controller on each telemetry sample."""

    def __init__(
        self,
        sender: ControlCommandSender,
        controller: MasterController,
        print_status: bool = False,
    ) -> None:
        self.sender = sender
        self.controller = controller
        self.print_status = print_status
        self._latest_command: ControlCommand | None = None

    def handle_state(self, state: AircraftState) -> None:
        command = self.controller.compute(state)
        self._latest_command = command
        self.sender.send_control_command(command)

        if self.print_status and command.aileron is not None:
            print(
                f"Yaw Rate: {degrees(state.r_rad_s):+.4f} deg/s | "
                f"Aileron Cmd: {command.aileron:+.3f}",
                end="\r",
            )

    def get_latest_command(self) -> ControlCommand | None:
        return self._latest_command

    def close(self) -> None:
        return None
