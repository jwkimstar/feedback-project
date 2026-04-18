import csv
from dataclasses import dataclass
from io import StringIO
from pathlib import Path


@dataclass(frozen=True)
class RecordingData:
    metadata: dict[str, str]
    rows: list[dict[str, str]]


def _parse_metadata_line(line: str) -> tuple[str, str] | None:
    if not line.startswith("#"):
        return None
    key, separator, value = line[1:].strip().partition("=")
    if separator == "":
        return None
    return key.strip(), value.strip()


def load_recording(path: str | Path) -> RecordingData:
    metadata: dict[str, str] = {}
    csv_lines: list[str] = []

    with Path(path).open(newline="", encoding="utf-8") as handle:
        for line in handle:
            stripped = line.strip()
            if not csv_lines and stripped == "":
                continue
            if not csv_lines:
                parsed_metadata = _parse_metadata_line(line)
                if parsed_metadata is not None:
                    key, value = parsed_metadata
                    metadata[key] = value
                    continue
            csv_lines.append(line)

    reader = csv.DictReader(StringIO("".join(csv_lines)))
    return RecordingData(metadata=metadata, rows=list(reader))


@dataclass(frozen=True)
class RecordingSeries:
    time_s: list[float]
    roll_deg: list[float]
    pitch_deg: list[float]
    yaw_deg: list[float]
    aileron_cmd: list[float]
    elevator_cmd: list[float]
    rudder_cmd: list[float]
    p_deg_s: list[float]
    q_deg_s: list[float]
    r_deg_s: list[float]


def _optional_float(value: str | None) -> float:
    if value in (None, ""):
        return float("nan")
    return float(value)


def _required_rate(row: dict[str, str], deg_key: str, rad_key: str) -> float:
    if row.get(deg_key) not in (None, ""):
        return float(row[deg_key])
    if row.get(rad_key) not in (None, ""):
        from math import degrees

        return degrees(float(row[rad_key]))
    raise KeyError(f"Missing both {deg_key!r} and {rad_key!r} in recording row.")


def _resolve_recording_hz(metadata: dict[str, str], hz: int | None) -> int:
    if hz is None:
        raw_hz = metadata.get("hz")
        if raw_hz in (None, ""):
            raise ValueError(
                "Recording sample rate is missing. Pass --hz for older CSV files."
            )
        hz = int(raw_hz)
    if hz <= 0:
        raise ValueError("hz must be greater than 0 for offline plotting.")
    return hz


def load_recording_series(path: str | Path, hz: int | None = None) -> RecordingSeries:
    recording = load_recording(path)
    effective_hz = _resolve_recording_hz(recording.metadata, hz)
    rows = recording.rows

    time_s = [index / effective_hz for index in range(len(rows))]

    return RecordingSeries(
        time_s=time_s,
        roll_deg=[float(row["roll_deg"]) for row in rows],
        pitch_deg=[float(row["pitch_deg"]) for row in rows],
        yaw_deg=[float(row["heading_deg"]) for row in rows],
        aileron_cmd=[_optional_float(row.get("aileron_cmd")) for row in rows],
        elevator_cmd=[_optional_float(row.get("elevator_cmd")) for row in rows],
        rudder_cmd=[_optional_float(row.get("rudder_cmd")) for row in rows],
        p_deg_s=[_required_rate(row, "p_deg_s", "p_rad_s") for row in rows],
        q_deg_s=[_required_rate(row, "q_deg_s", "q_rad_s") for row in rows],
        r_deg_s=[_required_rate(row, "r_deg_s", "r_rad_s") for row in rows],
    )


def plot_recording(path: str | Path, hz: int | None = None) -> None:
    try:
        import matplotlib.pyplot as plt
    except ImportError as exc:
        raise ImportError(
            "matplotlib is required for offline plotting. Install the 'plots' extra."
        ) from exc

    series = load_recording_series(path, hz)

    fig, axes = plt.subplots(3, 3, sharex=True, figsize=(15, 10))
    flat_axes = axes.flatten()

    plot_specs = [
        ("Roll (deg)", series.roll_deg),
        ("Pitch (deg)", series.pitch_deg),
        ("Yaw / Heading (deg)", series.yaw_deg),
        ("Aileron Cmd", series.aileron_cmd),
        ("Elevator Cmd", series.elevator_cmd),
        ("Rudder Cmd", series.rudder_cmd),
        ("p (deg/s)", series.p_deg_s),
        ("q (deg/s)", series.q_deg_s),
        ("r (deg/s)", series.r_deg_s),
    ]

    for axis, (label, values) in zip(flat_axes, plot_specs, strict=True):
        axis.plot(series.time_s, values, label=label)
        axis.set_ylabel(label)
        axis.grid(True, alpha=0.3)
        axis.legend(loc="upper left")

    for axis in flat_axes[6:]:
        axis.set_xlabel("Elapsed time (s)")

    fig.suptitle("Recorded Aircraft State")
    fig.tight_layout()
    plt.show()
