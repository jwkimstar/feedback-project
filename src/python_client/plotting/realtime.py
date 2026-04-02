from collections import deque
from time import monotonic

from python_client.console import ControlCommandProvider
from python_client.models import AircraftState, ControlCommand
from python_client.runtime import close_handlers, stream_to_handlers


class LivePlotter:
    def __init__(
        self,
        hz: int,
        history_seconds: int = 60,
        command_provider: ControlCommandProvider | None = None,
    ) -> None:
        try:
            import matplotlib.pyplot as plt
        except ImportError as exc:
            raise ImportError(
                "matplotlib is required for live plotting. Install the 'plots' extra."
            ) from exc

        self.plt = plt
        max_points = max(10, history_seconds * max(1, hz))
        self.timestamps = deque(maxlen=max_points)
        self.pitch_values = deque(maxlen=max_points)
        self.heading_values = deque(maxlen=max_points)
        self.roll_values = deque(maxlen=max_points)
        self.aileron_commands = deque(maxlen=max_points)
        self.elevator_commands = deque(maxlen=max_points)
        self.rudder_commands = deque(maxlen=max_points)
        self.command_provider = command_provider

        self.fig, axes = plt.subplots(2, 3, sharex=True, figsize=(14, 8))
        self.axes = axes.flatten()
        self.lines = [
            self.axes[0].plot([], [], label="Pitch (deg)")[0],
            self.axes[1].plot([], [], label="Heading (deg)")[0],
            self.axes[2].plot([], [], label="Roll (deg)")[0],
            self.axes[3].plot([], [], label="Aileron Cmd")[0],
            self.axes[4].plot([], [], label="Elevator Cmd")[0],
            self.axes[5].plot([], [], label="Rudder Cmd")[0],
        ]
        labels = [
            "Pitch (deg)",
            "Heading (deg)",
            "Roll (deg)",
            "Aileron Cmd",
            "Elevator Cmd",
            "Rudder Cmd",
        ]

        for axis, label in zip(self.axes, labels, strict=True):
            axis.set_ylabel(label)
            axis.grid(True, alpha=0.3)
            axis.legend(loc="upper left")

        for axis in self.axes[3:]:
            axis.set_xlabel("Elapsed time (s)")
        self.fig.suptitle("X-Plane Telemetry")
        plt.ion()
        plt.show(block=False)
        self.start_time = monotonic()

    def handle_state(self, state: AircraftState) -> None:
        elapsed = monotonic() - self.start_time
        self.timestamps.append(elapsed)
        self.pitch_values.append(state.pitch_deg)
        self.heading_values.append(state.heading_deg)
        self.roll_values.append(state.roll_deg)
        command = self._latest_command()
        self.aileron_commands.append(self._command_value(command.aileron))
        self.elevator_commands.append(self._command_value(command.elevator))
        self.rudder_commands.append(self._command_value(command.rudder))

        series = [
            self.pitch_values,
            self.heading_values,
            self.roll_values,
            self.aileron_commands,
            self.elevator_commands,
            self.rudder_commands,
        ]
        for axis, line, values in zip(self.axes, self.lines, series, strict=True):
            line.set_data(self.timestamps, values)
            axis.relim()
            axis.autoscale_view()

        self.fig.canvas.draw_idle()
        self.fig.canvas.flush_events()
        self.plt.pause(0.001)

    def _latest_command(self) -> ControlCommand:
        if self.command_provider is None:
            return ControlCommand()
        return self.command_provider.get_latest_command() or ControlCommand()

    @staticmethod
    def _command_value(value: float | None) -> float:
        return float("nan") if value is None else value

    def close(self) -> None:
        self.plt.close(self.fig)


def plot_live_stream(wait: float, hz: int, history_seconds: int = 60) -> None:
    plotter = LivePlotter(hz=hz, history_seconds=history_seconds)
    try:
        stream_to_handlers(wait=wait, hz=hz, handlers=[plotter])
    finally:
        close_handlers([plotter])
