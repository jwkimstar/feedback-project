import csv
from pathlib import Path
from typing import Protocol, TextIO

from python_client.logging.schema import CSV_HEADER, RecordingMetadata, sample_to_row
from python_client.models import AircraftState, ControlCommand


class ControlCommandProvider(Protocol):
    def get_latest_command(self) -> ControlCommand | None:
        """Return the most recent control command, if one exists."""


class SessionRecorder:
    def __init__(
        self,
        output_path: str | Path,
        command_provider: ControlCommandProvider | None = None,
        metadata: RecordingMetadata | None = None,
    ) -> None:
        self.output_path = Path(output_path)
        self.command_provider = command_provider
        self.metadata = metadata
        self._handle: TextIO | None = None
        self._writer: csv.writer | None = None

    def __enter__(self) -> "SessionRecorder":
        self.output_path.parent.mkdir(parents=True, exist_ok=True)
        self._handle = self.output_path.open("w", newline="", encoding="utf-8")
        if self.metadata is not None:
            for line in self.metadata.to_comment_lines():
                self._handle.write(f"{line}\n")
        self._writer = csv.writer(self._handle)
        self._writer.writerow(CSV_HEADER)
        return self

    def __exit__(self, exc_type, exc, tb) -> None:
        self.close()

    def write(self, state: AircraftState) -> None:
        if self._writer is None:
            raise RuntimeError("SessionRecorder must be entered before writing.")
        command = None
        if self.command_provider is not None:
            command = self.command_provider.get_latest_command()
        self._writer.writerow(sample_to_row(state, command))

    def handle_state(self, state: AircraftState) -> None:
        self.write(state)

    def close(self) -> None:
        if self._handle is not None:
            self._handle.close()
            self._handle = None
            self._writer = None
