from python_client.logging.recorder import SessionRecorder
from python_client.logging.schema import CSV_HEADER, RecordingMetadata, sample_to_row
from python_client.models import AircraftState, ControlCommand


def make_state() -> AircraftState:
    return AircraftState(
        lon_deg=1.0,
        lat_deg=2.0,
        ele_m=3.0,
        agl_m=4.0,
        pitch_deg=5.0,
        heading_deg=6.0,
        roll_deg=7.0,
        vx_east=8.0,
        vy_up=9.0,
        vz_south=10.0,
        p_rad_s=11.0,
        q_rad_s=12.0,
        r_rad_s=13.0,
    )


def test_csv_header_includes_control_commands() -> None:
    assert CSV_HEADER[-3:] == ["aileron_cmd", "elevator_cmd", "rudder_cmd"]


def test_sample_to_row_appends_command_values() -> None:
    row = sample_to_row(
        make_state(),
        ControlCommand(aileron=0.1, elevator=-0.2, rudder=0.3),
    )

    assert row[-3:] == [0.1, -0.2, 0.3]


def test_sample_to_row_uses_empty_fields_without_command() -> None:
    row = sample_to_row(make_state())

    assert row[-3:] == ["", "", ""]


def test_recording_metadata_serializes_to_comment_lines() -> None:
    metadata = RecordingMetadata(hz=15, yaw_damper_gain=2.0, control_mode="yaw-damper")

    assert metadata.to_comment_lines()[:3] == [
        "# hz=15",
        "# control_mode=yaw-damper",
        "# yaw_controller_type=",
    ]
    assert "# yaw_damper_gain=2.0" in metadata.to_comment_lines()


def test_session_recorder_writes_metadata_before_csv_header(tmp_path) -> None:
    path = tmp_path / "session.csv"

    with SessionRecorder(path, metadata=RecordingMetadata(hz=10, yaw_damper_gain=2.0)) as recorder:
        recorder.write(make_state())

    lines = path.read_text(encoding="utf-8").splitlines()

    assert lines[0] == "# hz=10"
    assert "# yaw_damper_gain=2.0" in lines
    assert "lon_deg,lat_deg,ele_m,agl_m,pitch_deg,heading_deg,roll_deg,vx_east,vy_up,vz_south,p_deg_s,q_deg_s,r_deg_s,aileron_cmd,elevator_cmd,rudder_cmd" in lines
