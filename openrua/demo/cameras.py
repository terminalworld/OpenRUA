"""Export recorded camera images at their indexed wall-clock times.

Each image stays on screen until the next available sample. The last image
stays for the median sample interval, or ``single_frame_duration_s`` when
only one image exists. No robot, transcript or task result is required.
"""

from __future__ import annotations

import json
import math
import os
import re
import statistics
import subprocess
import tempfile
from pathlib import Path


_TIME_BASE = 1_000_000
_FRAME_NAME = re.compile(r"^[0-9]+_(.+)\.(jpg|jpeg|png)$", re.IGNORECASE)


def _read_index(index: Path) -> tuple[dict, set[int], list[str]]:
    cameras: dict[str, list[tuple[float, str, int]]] = {}
    ticks: set[int] = set()
    errors: list[str] = []
    with index.open(encoding="utf-8") as stream:
        for number, line in enumerate(stream, 1):
            if not line.strip():
                continue
            try:
                row = json.loads(line)
                t, files = row["t"], row["files"]
                if (isinstance(t, bool) or not isinstance(t, (int, float))
                        or not math.isfinite(t) or not isinstance(files, list)):
                    raise ValueError("expected a finite timestamp and a files list")
            except (ValueError, KeyError, TypeError, OverflowError) as exc:
                errors.append(f"Index line {number}: {exc}")
                continue
            ticks.add(number)
            for name in files:
                match = _FRAME_NAME.fullmatch(name) if isinstance(name, str) else None
                if (match is None or Path(name).name != name
                        or any(c in name for c in "\\\r\n\0")
                        or match[1] in {".", ".."}):
                    errors.append(f"Index line {number}: invalid frame filename {name!r}")
                    continue
                cameras.setdefault(match[1], []).append((float(t), name, number))
    return cameras, ticks, errors


def _quoted(path: Path) -> str:
    """Quote a path for FFmpeg's concat file syntax, without a shell."""
    value = str(path)
    if "\n" in value or "\r" in value:
        raise ValueError("Camera video paths cannot contain line breaks")
    return "'" + value.replace("'", "'\\''") + "'"


def _encode(frames: Path, samples: list, output: Path, info: dict,
            single_frame_duration_s: float, image, ffmpeg: str) -> None:
    with tempfile.TemporaryDirectory(prefix=".camera-", dir=output.parent) as directory:
        temporary = Path(directory)
        valid: list[tuple[float, Path]] = []
        size = image_format = None
        for t, name, _ in samples:
            path = frames / name
            try:
                if path.resolve().parent != frames:
                    raise ValueError("image resolves outside the recording directory")
                with image.open(path) as source:
                    source.load()
                    if source.format not in {"JPEG", "PNG"}:
                        raise ValueError("expected a JPEG or PNG image")
                    if size is not None and source.size != size:
                        raise ValueError(f"image size changed from {size} to {source.size}")
                    if valid and (round((t - valid[0][0]) * _TIME_BASE)
                                  <= round((valid[-1][0] - valid[0][0]) * _TIME_BASE)):
                        raise ValueError("timestamp does not advance by at least one microsecond")
                    if size is None:
                        size, image_format = source.size, source.format
                    # The concat demuxer needs one image codec. Normal recordings
                    # use one format, so only a format change needs a temporary copy.
                    if source.format != image_format:
                        path = temporary / f"normalized-{len(valid)}.{'jpg' if image_format == 'JPEG' else 'png'}"
                        source.convert("RGB").save(path, format=image_format)
                valid.append((t, path))
            except (OSError, ValueError) as exc:
                info["errors"].append(f"{name}: {exc}")

        info["frame_count"] = len(valid)
        if not valid:
            info["status"] = "failed"
            return

        first = valid[0][0]
        positions = [round((t - first) * _TIME_BASE) for t, _ in valid]
        intervals = [b - a for a, b in zip(positions, positions[1:])]
        final = max(1, round(statistics.median(intervals)) if intervals
                    else round(single_frame_duration_s * _TIME_BASE))
        lines = ["ffconcat version 1.0"]
        for i, (_, path) in enumerate(valid):
            duration = intervals[i] if i < len(intervals) else final
            lines.extend((f"file {_quoted(path)}", f"option framerate {_TIME_BASE}",
                          f"duration {duration / _TIME_BASE:.6f}"))
        # A final endpoint makes FFmpeg retain the last image's intended hold.
        # This is an encoded duplicate, never an additional source observation.
        lines.extend((f"file {_quoted(valid[-1][1])}", f"option framerate {_TIME_BASE}"))
        manifest = temporary / "frames.ffconcat"
        manifest.write_text("\n".join(lines) + "\n", encoding="utf-8")
        encoded = temporary / "video.mp4"
        # The demuxer's microsecond clock is not a million-frame capture rate.
        # Give x264 a normal nominal rate for codec level selection, while VFR
        # and the encoder time base preserve every indexed presentation time.
        proc = subprocess.run(
            [ffmpeg, "-hide_banner", "-nostdin", "-loglevel", "error", "-xerror", "-safe", "0",
             "-f", "concat", "-i", str(manifest), "-an",
             "-vf", "pad=ceil(iw/2)*2:ceil(ih/2)*2", "-fps_mode", "vfr",
             "-c:v", "libx264", "-preset", "veryfast", "-crf", "18", "-bf", "0",
             "-enc_time_base", f"1:{_TIME_BASE}", "-x264-params", "fps=30/1",
             "-pix_fmt", "yuv420p", "-video_track_timescale", str(_TIME_BASE),
             "-movflags", "+faststart", str(encoded)],
            capture_output=True, text=True, errors="replace", check=False,
        )
        if proc.returncode:
            raise RuntimeError(f"FFmpeg exited with status {proc.returncode}: {proc.stderr.strip()}")
        if not encoded.is_file() or not encoded.stat().st_size:
            raise RuntimeError("FFmpeg did not produce a video")
        os.replace(encoded, output)
        info.update(
            status="partial" if info["errors"] or info["missing_tick_count"] else "completed",
            file=str(output),
            first_t=first, last_t=valid[-1][0],
            duration_s=(positions[-1] + final + 1) / _TIME_BASE,
            final_frame_duration_s=final / _TIME_BASE,
            max_frame_interval_s=max(intervals, default=0) / _TIME_BASE,
            encoded_frame_count=len(valid) + 1,
        )


def export_camera_videos(frames: Path, output: Path, *,
                         single_frame_duration_s: float = 0.5) -> dict:
    """Write one MP4 per camera and return its export report.

    Source images and index stay unchanged. Errors are reported separately
    for each camera; valid images still produce a partial video. Missing or
    corrupt images extend the preceding image's hold rather than compressing
    time. ``KeyboardInterrupt`` propagates after temporary output is removed.
    """
    if not math.isfinite(single_frame_duration_s) or single_frame_duration_s <= 0:
        raise ValueError("single_frame_duration_s must be finite and positive")
    frames, output = Path(frames).resolve(), Path(output).absolute()
    report = {
        "status": "empty", "source_index": str(frames / "index.jsonl"),
        "timing": {"clock": "recording_wall_time", "playback": "hold_until_next_sample",
                   "time_base_s": 1 / _TIME_BASE, "final_frame_hold": "median_sample_interval",
                   "single_frame_duration_s": single_frame_duration_s,
                   "endpoint_duplicate_duration_s": 1 / _TIME_BASE},
        "cameras": {}, "errors": [],
    }
    try:
        cameras, ticks, errors = _read_index(frames / "index.jsonl")
        report["errors"].extend(errors)
        if not cameras:
            report["status"] = "failed" if errors else "empty"
            return report
        from PIL import Image
        from imageio_ffmpeg import get_ffmpeg_exe

        ffmpeg = get_ffmpeg_exe()
        output.mkdir(parents=True, exist_ok=True)
        for name, samples in cameras.items():
            info = {"status": "failed", "frame_count": 0, "indexed_frame_count": len(samples),
                    "missing_tick_count": len(ticks - {row for _, _, row in samples}), "errors": []}
            report["cameras"][name] = info
            try:
                _encode(frames, samples, output / f"{name}.mp4", info,
                        single_frame_duration_s, Image, ffmpeg)
            except (OSError, ValueError, RuntimeError) as exc:
                info["errors"].append(str(exc))
            report["errors"].extend(f"{name}: {error}" for error in info["errors"])
        complete = all(c["status"] == "completed" for c in report["cameras"].values())
        any_video = any("file" in c for c in report["cameras"].values())
        report["status"] = ("completed" if complete and not report["errors"] else
                            "partial" if any_video else "failed")
    except (OSError, ValueError, RuntimeError, ImportError) as exc:
        report["status"] = "failed"
        report["errors"].append(str(exc))
    return report
