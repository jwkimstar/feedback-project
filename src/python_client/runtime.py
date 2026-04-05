import signal
from contextlib import contextmanager
from typing import Callable, Iterator, Protocol

from python_client.models import AircraftState
from python_client.xplane.client import XPlaneClient


class StateHandler(Protocol):
    def handle_state(self, state: AircraftState) -> None:
        """Consume one telemetry sample."""

    def close(self) -> None:
        """Release any held resources."""


class StopRequested(Exception):
    """Raised when the streaming loop should stop cleanly."""


@contextmanager
def trap_sigint() -> Iterator[Callable[[], bool]]:
    stop_requested = False
    previous_handler = signal.getsignal(signal.SIGINT)

    def _handle_sigint(_signum: int, _frame: object | None) -> None:
        nonlocal stop_requested
        stop_requested = True

    signal.signal(signal.SIGINT, _handle_sigint)
    try:
        yield lambda: stop_requested
    finally:
        signal.signal(signal.SIGINT, previous_handler)


def stream_to_handlers(
    wait: float,
    hz: int,
    handlers: list[StateHandler],
    should_stop: Callable[[], bool] | None = None,
) -> None:
    with XPlaneClient.discover(wait=wait, hz=hz) as client:
        run_client_to_handlers(client, handlers, should_stop=should_stop)


def run_client_to_handlers(
    client: XPlaneClient,
    handlers: list[StateHandler],
    should_stop: Callable[[], bool] | None = None,
) -> None:
    while True:
        if should_stop is not None and should_stop():
            raise StopRequested
        state = client.recv_state()
        for handler in handlers:
            if should_stop is not None and should_stop():
                raise StopRequested
            handler.handle_state(state)


def close_handlers(handlers: list[StateHandler]) -> None:
    for handler in reversed(handlers):
        handler.close()
