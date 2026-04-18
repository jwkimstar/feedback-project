from math import degrees

import pytest

from python_client.plotting.analysis import load_recording, load_recording_series


def test_load_recording_series_builds_elapsed_time_and_values(tmp_path) -> None:
    path = tmp_path / "session.csv"
    path.write_text(
        "\n".join(
            [
                "# hz=2",
                "lon_deg,lat_deg,ele_m,agl_m,pitch_deg,heading_deg,roll_deg,vx_east,vy_up,vz_south,p_rad_s,q_rad_s,r_rad_s,aileron_cmd,elevator_cmd,rudder_cmd",
                "1,2,3,4,5,6,7,8,9,10,11,12,13,0.1,,0.3",
                "1,2,3,4,15,16,17,8,9,10,21,22,23,,,-0.4",
            ]
        ),
        encoding="utf-8",
    )

    series = load_recording_series(path)

    assert series.time_s == [0.0, 0.5]
    assert series.roll_deg == [7.0, 17.0]
    assert series.pitch_deg == [5.0, 15.0]
    assert series.yaw_deg == [6.0, 16.0]
    assert series.p_deg_s == [degrees(11.0), degrees(21.0)]
    assert series.q_deg_s == [degrees(12.0), degrees(22.0)]
    assert series.r_deg_s == [degrees(13.0), degrees(23.0)]


def test_load_recording_reads_metadata_comments(tmp_path) -> None:
    path = tmp_path / "session.csv"
    path.write_text(
        "\n".join(
            [
                "# hz=15",
                "# yaw_damper_gain=2.0",
                "lon_deg,lat_deg,ele_m,agl_m,pitch_deg,heading_deg,roll_deg,vx_east,vy_up,vz_south,p_deg_s,q_deg_s,r_deg_s,aileron_cmd,elevator_cmd,rudder_cmd",
                "1,2,3,4,5,6,7,8,9,10,11,12,13,0.1,,0.3",
            ]
        ),
        encoding="utf-8",
    )

    recording = load_recording(path)

    assert recording.metadata == {"hz": "15", "yaw_damper_gain": "2.0"}
    assert len(recording.rows) == 1


def test_load_recording_series_accepts_hz_override_for_older_files(tmp_path) -> None:
    path = tmp_path / "legacy-session.csv"
    path.write_text(
        "\n".join(
            [
                "lon_deg,lat_deg,ele_m,agl_m,pitch_deg,heading_deg,roll_deg,vx_east,vy_up,vz_south,p_deg_s,q_deg_s,r_deg_s,aileron_cmd,elevator_cmd,rudder_cmd",
                "1,2,3,4,5,6,7,8,9,10,11,12,13,0.1,,0.3",
                "1,2,3,4,15,16,17,8,9,10,21,22,23,,,-0.4",
            ]
        ),
        encoding="utf-8",
    )

    series = load_recording_series(path, hz=4)

    assert series.time_s == [0.0, 0.25]


def test_load_recording_series_requires_hz_when_metadata_is_missing(tmp_path) -> None:
    path = tmp_path / "legacy-session.csv"
    path.write_text(
        "\n".join(
            [
                "lon_deg,lat_deg,ele_m,agl_m,pitch_deg,heading_deg,roll_deg,vx_east,vy_up,vz_south,p_deg_s,q_deg_s,r_deg_s,aileron_cmd,elevator_cmd,rudder_cmd",
                "1,2,3,4,5,6,7,8,9,10,11,12,13,0.1,,0.3",
            ]
        ),
        encoding="utf-8",
    )

    with pytest.raises(ValueError, match="Pass --hz for older CSV files"):
        load_recording_series(path)
