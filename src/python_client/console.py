from typing import Protocol

from python_client.models import AircraftState, ControlCommand


class ControlCommandProvider(Protocol):
    def get_latest_command(self) -> ControlCommand | None:
        """Return the most recent control command, if one exists."""


def _format_command_value(value: float | None) -> str:
    return "n/a" if value is None else f"{value:+.3f}"


def format_state_line(
    state: AircraftState,
    command: ControlCommand | None = None,
) -> str:
    command = command or ControlCommand()
    return (
        f"Pitch: {state.pitch_deg:8.3f} deg | "
        f"Heading: {state.heading_deg:8.3f} deg | "
        f"Roll: {state.roll_deg:8.3f} deg | "
        f"Aileron Cmd: {_format_command_value(command.aileron)} | "
        f"Elevator Cmd: {_format_command_value(command.elevator)} | "
        f"Rudder Cmd: {_format_command_value(command.rudder)}"
    )


class TelemetryPrinter:
    """Print a compact orientation line for each telemetry sample."""

    def __init__(
        self,
        command_provider: ControlCommandProvider | None = None,
    ) -> None:
        self.command_provider = command_provider

    def handle_state(self, state: AircraftState) -> None:
        command = None
        if self.command_provider is not None:
            command = self.command_provider.get_latest_command()
        print(format_state_line(state, command))

    def close(self) -> None:
        return None
