from collections import deque
from time import monotonic

from python_client.models import AircraftState
from python_client.runtime import close_handlers, stream_to_handlers


class LivePlotter:
    def __init__(self, hz: int, history_seconds: int = 60) -> None:
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

        self.fig, self.axes = plt.subplots(3, 1, sharex=True, figsize=(10, 8))
        self.lines = [
            self.axes[0].plot([], [], label="Pitch (deg)")[0],
            self.axes[1].plot([], [], label="Heading (deg)")[0],
            self.axes[2].plot([], [], label="Roll (deg)")[0],
        ]
        labels = ["Pitch (deg)", "Heading (deg)", "Roll (deg)"]

        for axis, label in zip(self.axes, labels, strict=True):
            axis.set_ylabel(label)
            axis.grid(True, alpha=0.3)
            axis.legend(loc="upper left")

        self.axes[-1].set_xlabel("Elapsed time (s)")
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

        series = [self.pitch_values, self.heading_values, self.roll_values]
        for axis, line, values in zip(self.axes, self.lines, series, strict=True):
            line.set_data(self.timestamps, values)
            axis.relim()
            axis.autoscale_view()

        self.fig.canvas.draw_idle()
        self.fig.canvas.flush_events()
        self.plt.pause(0.001)

    def close(self) -> None:
        self.plt.close(self.fig)


def plot_live_stream(wait: float, hz: int, history_seconds: int = 60) -> None:
    plotter = LivePlotter(hz=hz, history_seconds=history_seconds)
    try:
        stream_to_handlers(wait=wait, hz=hz, handlers=[plotter])
    finally:
        close_handlers([plotter])
