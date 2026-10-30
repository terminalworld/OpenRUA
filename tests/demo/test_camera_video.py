"""Camera-only exports preserve sample timing without replaying a task."""

from __future__ import annotations

import json
import re
import subprocess
from pathlib import Path

import pytest
from PIL import Image
from imageio_ffmpeg import get_ffmpeg_exe

from openrua.demo import cameras


def _frames(directory: Path, times=(100.0, 100.4, 101.5), camera="front", size=(17, 19)):
    directory.mkdir(parents=True, exist_ok=True)
    rows = []
    for number, t in enumerate(times):
        name = f"{number:06d}_{camera}.png"
        Image.new("RGB", size, (number * 50 % 256, 50, 100)).save(directory / name)
        rows.append({"step": number, "t": t, "files": [name]})
    (directory / "index.jsonl").write_text("".join(json.dumps(row) + "\n" for row in rows))
    return rows


def _video_samples(path: str) -> list[tuple[float, float]]:
    result = subprocess.run(
        [get_ffmpeg_exe(), "-hide_banner", "-nostdin", "-i", path, "-vf", "showinfo",
         "-fps_mode", "passthrough", "-f", "null", "-"],
        capture_output=True, text=True, check=True,
    )
    return [(float(t), float(d)) for t, d in re.findall(
        r"pts_time:([0-9.e+-]+).*?duration_time:([0-9.e+-]+)", result.stderr)]


def test_export_preserves_irregular_wall_times_and_source_files(tmp_path):
    frames = tmp_path / "robot's project" / "frames"
    _frames(frames)
    original = {path.name: path.read_bytes() for path in frames.iterdir()}

    result = cameras.export_camera_videos(frames, tmp_path / "videos")

    assert result["status"] == "completed"
    info = result["cameras"]["front"]
    assert info["frame_count"] == info["indexed_frame_count"] == 3
    assert info["encoded_frame_count"] == 4
    assert info["first_t"] == 100.0
    assert info["last_t"] == 101.5
    assert info["final_frame_duration_s"] == pytest.approx(0.75)
    assert info["duration_s"] == pytest.approx(2.250001)
    assert info["max_frame_interval_s"] == pytest.approx(1.1)
    samples = _video_samples(info["file"])
    assert [t for t, _ in samples] == pytest.approx([0.0, 0.4, 1.5, 2.25])
    assert [d for _, d in samples] == pytest.approx([0.4, 1.1, 0.75, 0.000001])
    assert original == {path.name: path.read_bytes() for path in frames.iterdir()}
    assert list((tmp_path / "videos").iterdir()) == [Path(info["file"])]


def test_corrupt_missing_frames_and_partial_index_keep_available_footage(tmp_path):
    frames = tmp_path / "frames"
    rows = _frames(frames, (10.0, 10.5, 11.0, 11.5))
    (frames / rows[1]["files"][0]).write_bytes(b"truncated image")
    (frames / rows[2]["files"][0]).unlink()
    with (frames / "index.jsonl").open("a") as stream:
        stream.write('{"step":4,"t":')

    result = cameras.export_camera_videos(frames, tmp_path / "videos")

    assert result["status"] == "partial"
    info = result["cameras"]["front"]
    assert info["status"] == "partial"
    assert info["frame_count"] == 2
    assert info["indexed_frame_count"] == 4
    assert len(result["errors"]) == 3
    assert [t for t, _ in _video_samples(info["file"])] == pytest.approx([0.0, 1.5, 3.0])


def test_timestamp_precision_does_not_advertise_an_extreme_h264_level(tmp_path):
    frames = tmp_path / "frames"
    _frames(frames, (100.0, 100.400123, 101.500456), size=(640, 480))
    result = cameras.export_camera_videos(frames, tmp_path / "videos")
    info = result["cameras"]["front"]
    assert result["status"] == "completed"
    headers = subprocess.run(
        [get_ffmpeg_exe(), "-hide_banner", "-nostdin", "-i", info["file"],
         "-c:v", "copy", "-bsf:v", "trace_headers", "-f", "null", "-"],
        capture_output=True, text=True, check=True,
    ).stderr
    level = re.search(r"\blevel_idc\s+[01]+\s+=\s+(\d+)", headers)
    assert level is not None and int(level[1]) <= 41
    assert re.search(r"\bfixed_frame_rate_flag\s+0\s+=\s+0", headers)
    samples = _video_samples(info["file"])
    assert [t for t, _ in samples][:3] == pytest.approx([0.0, 0.400123, 1.500456], abs=1e-6)
    assert len(samples) == 4  # Three observations and the final hold endpoint.


def test_cameras_with_missing_ticks_keep_their_own_start_times(tmp_path):
    frames = tmp_path / "frames"
    rows = _frames(frames)
    Image.new("RGB", (12, 14)).save(frames / "000001_wrist_camera.jpg")
    rows[1]["files"].append("000001_wrist_camera.jpg")
    (frames / "index.jsonl").write_text("".join(json.dumps(row) + "\n" for row in rows))

    result = cameras.export_camera_videos(frames, tmp_path / "videos")

    assert result["status"] == "partial"
    assert result["cameras"]["front"]["status"] == "completed"
    info = result["cameras"]["wrist_camera"]
    assert info["status"] == "partial"
    assert info["missing_tick_count"] == 2
    assert info["first_t"] == info["last_t"] == 100.4
    assert info["frame_count"] == 1
    assert info["duration_s"] == pytest.approx(0.500001)


def test_empty_index_does_not_create_output(tmp_path):
    frames = tmp_path / "frames"
    frames.mkdir()
    (frames / "index.jsonl").write_text("")
    result = cameras.export_camera_videos(frames, tmp_path / "videos")
    assert result["status"] == "empty"
    assert result["cameras"] == {}
    assert result["errors"] == []
    assert not (tmp_path / "videos").exists()


@pytest.mark.parametrize("interrupt", [False, True])
def test_encoder_failure_and_interrupt_preserve_existing_video_and_clean_temporary_files(
    tmp_path, monkeypatch, interrupt,
):
    frames = tmp_path / "frames"
    _frames(frames)
    output = tmp_path / "videos"
    output.mkdir()
    prior = output / "front.mp4"
    prior.write_bytes(b"previous complete video")

    def fail(command, **kwargs):
        Path(command[-1]).write_bytes(b"unfinished")
        if interrupt:
            raise KeyboardInterrupt
        return subprocess.CompletedProcess(command, 7, "", "encoder failure details")

    monkeypatch.setattr(cameras.subprocess, "run", fail)
    if interrupt:
        with pytest.raises(KeyboardInterrupt):
            cameras.export_camera_videos(frames, output)
    else:
        result = cameras.export_camera_videos(frames, output)
        assert result["status"] == "failed"
        assert "status 7: encoder failure details" in result["errors"][0]
        assert "file" not in result["cameras"]["front"]
    assert prior.read_bytes() == b"previous complete video"
    assert list(output.iterdir()) == [prior]


def test_export_rejects_path_escape_without_reading_unindexed_images(tmp_path):
    frames = tmp_path / "frames"
    rows = _frames(frames, (10.0,))
    outside = tmp_path / "outside.png"
    Image.new("RGB", (17, 19)).save(outside)
    (frames / "000001_side.png").symlink_to(outside)
    rows.append({"step": 1, "t": 10.5,
                 "files": ["../outside.png", "000001_side.png", "000001_../escape.png"]})
    (frames / "index.jsonl").write_text("".join(json.dumps(row) + "\n" for row in rows))

    result = cameras.export_camera_videos(frames, tmp_path / "videos")

    assert result["status"] == "partial"
    assert result["cameras"]["side"]["status"] == "failed"
    assert len(result["errors"]) == 3
    assert not (tmp_path / "escape.mp4").exists()


def test_format_change_and_duplicate_timestamp_are_reported_correctly(tmp_path):
    frames = tmp_path / "frames"
    rows = _frames(frames, (10.0, 10.5, 10.5, 11.0))
    jpeg = "000001_front.jpg"
    Image.new("RGB", (17, 19)).save(frames / jpeg)
    rows[1]["files"] = [jpeg]
    (frames / "index.jsonl").write_text("".join(json.dumps(row) + "\n" for row in rows))

    result = cameras.export_camera_videos(frames, tmp_path / "videos")

    info = result["cameras"]["front"]
    assert result["status"] == "partial"
    assert info["frame_count"] == 3
    assert len(info["errors"]) == 1
    assert "timestamp does not advance" in info["errors"][0]
    assert [t for t, _ in _video_samples(info["file"])] == pytest.approx([0, 0.5, 1, 1.5])


def test_clock_regression_and_changed_image_size_do_not_erase_good_frames(tmp_path):
    frames = tmp_path / "frames"
    rows = _frames(frames, (10.0, 9.0, 11.0, 12.0))
    Image.new("RGB", (30, 40)).save(frames / rows[2]["files"][0])

    result = cameras.export_camera_videos(frames, tmp_path / "videos")

    info = result["cameras"]["front"]
    assert result["status"] == "partial"
    assert info["frame_count"] == 2
    assert len(info["errors"]) == 2
    assert [t for t, _ in _video_samples(info["file"])] == pytest.approx([0, 2, 4])
