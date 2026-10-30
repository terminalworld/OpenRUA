"""Record camera export outcomes without changing the task outcome.

The caller holds the trial's writer claim and supplies a redacted result.
"""

from __future__ import annotations

import shlex
from pathlib import Path

from openrua.demo.cameras import export_camera_videos
from openrua.runner import record


def _say(text: str) -> None:
    try:
        print(text, flush=True)
    except OSError:
        pass


def export_videos(trial_dir: Path, rec: dict, *, secrets=()) -> None:
    """Export after resource cleanup, keeping failures in ``video_export``."""
    retry = f"openrua demo {shlex.quote(str(trial_dir))} --camera-only"
    if (rec.get("cleanup") or {}).get("errors"):
        rec["video_export"] = {"status": "skipped", "errors": [
            "Resources could not be fully stopped. Export the saved frames after cleanup."]}
        record.write_result(trial_dir, rec)
        _say(f"[trial] Video export skipped until cleanup finishes. Then run: {retry}")
        return
    rec["video_export"] = {"status": "exporting"}
    record.write_result(trial_dir, rec)
    _say("[trial] Exporting camera videos. The task has ended and its resources are stopped.")
    try:
        rec["video_export"] = export_camera_videos(trial_dir / "frames", trial_dir / "videos")
    except KeyboardInterrupt:
        rec["video_export"] = {"status": "canceled", "errors": ["Video export was interrupted."]}
        _say(f"[trial] Video export stopped. Your task result and recorded frames are saved. Retry: {retry}")
        raise
    except Exception as exc:
        rec["video_export"] = {"status": "failed", "errors": [str(exc)]}
    finally:
        rec["video_export"] = record.scrub_value(rec["video_export"], secrets)
        record.write_result(trial_dir, rec)
    if rec["video_export"]["status"] == "completed":
        _say(f"[trial] Camera videos saved in {trial_dir / 'videos'}")
    else:
        _say("[trial] Camera video export is incomplete. The task result and recorded frames are saved.")
        for error in rec["video_export"].get("errors", []):
            _say(f"[trial] {error}")
        _say(f"[trial] Inspect video_export in {trial_dir / 'result.json'} for details.")
        _say(f"[trial] Retry video export: {retry}")
