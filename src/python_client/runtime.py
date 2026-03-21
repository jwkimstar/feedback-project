from typing import Protocol

from python_client.models import AircraftState
from python_client.xplane.client import XPlaneClient


class StateHandler(Protocol):
    def handle_state(self, state: AircraftState) -> None:
        """Consume one telemetry sample."""

    def close(self) -> None:
        """Release any held resources."""


def stream_to_handlers(wait: float, hz: int, handlers: list[StateHandler]) -> None:
    with XPlaneClient.discover(wait=wait, hz=hz) as client:
        while True:
            state = client.recv_state()
            for handler in handlers:
                handler.handle_state(state)


def close_handlers(handlers: list[StateHandler]) -> None:
    for handler in reversed(handlers):
        handler.close()
