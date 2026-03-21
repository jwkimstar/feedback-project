from python_client.models import AircraftState


def format_state_line(state: AircraftState) -> str:
    return (
        f"Pitch: {state.pitch_deg:8.3f} deg | "
        f"Heading: {state.heading_deg:8.3f} deg | "
        f"Roll: {state.roll_deg:8.3f} deg"
    )


class TelemetryPrinter:
    """Print a compact orientation line for each telemetry sample."""

    def handle_state(self, state: AircraftState) -> None:
        print(format_state_line(state))

    def close(self) -> None:
        return None
