from math import degrees

from python_client.plotting.analysis import load_recording_series


def test_load_recording_series_builds_elapsed_time_and_values(tmp_path) -> None:
    path = tmp_path / "session.csv"
    path.write_text(
        "\n".join(
            [
                "lon_deg,lat_deg,ele_m,agl_m,pitch_deg,heading_deg,roll_deg,vx_east,vy_up,vz_south,p_rad_s,q_rad_s,r_rad_s,aileron_cmd,elevator_cmd,rudder_cmd",
                "1,2,3,4,5,6,7,8,9,10,11,12,13,0.1,,0.3",
                "1,2,3,4,15,16,17,8,9,10,21,22,23,,,-0.4",
            ]
        ),
        encoding="utf-8",
    )

    series = load_recording_series(path, hz=2)

    assert series.time_s == [0.0, 0.5]
    assert series.roll_deg == [7.0, 17.0]
    assert series.pitch_deg == [5.0, 15.0]
    assert series.yaw_deg == [6.0, 16.0]
    assert series.p_deg_s == [degrees(11.0), degrees(21.0)]
    assert series.q_deg_s == [degrees(12.0), degrees(22.0)]
    assert series.r_deg_s == [degrees(13.0), degrees(23.0)]
