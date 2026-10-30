"""``openrua demo <trial>``: a video of a recorded trial."""

from __future__ import annotations

import json
from pathlib import Path

from openrua import demo
from openrua.errors import UsageError
from openrua.runner import lock, record


def run(args) -> int:
    if args.camera_only:
        return _camera_videos(args)
    style = demo.Style(fps=args.fps, font_size=args.font_size, speed=args.speed,
                       typing=not args.no_typing, quality=args.quality,
                       gif_width=args.gif_width, gif_fps=args.gif_fps)
    if args.size:
        w, h = args.size.lower().split("x")
        style.width, style.height = int(w), int(h)
    if args.pace != 1.0:
        # The terminal's clock: how fast commands type and how long the
        # video rests between events. The robot's clock is --speed.
        style.typing_chars_per_s *= args.pace
        style.typing_max_s /= args.pace
        style.hold_s /= args.pace
    start, _, end = (args.ops or ":").partition(":")
    out = demo.render(Path(args.trial), Path(args.out) if args.out else None,
                      cameras=tuple(c for c in args.cameras.split(",") if c),
                      style=style, gif=args.gif,
                      ops_range=(int(start) if start else None, int(end) if end else None),
                      from_motion_s=args.from_motion)
    print(out)
    if args.gif:
        print(out.with_suffix(".gif"))
    return 0


def _camera_videos(args) -> int:
    from openrua.demo.cameras import export_camera_videos

    presentation = dict(gif=False, cameras="", pace=1.0, from_motion=None,
                        ops=None, size=None, fps=20, quality=18, speed=1.0,
                        font_size=13, gif_width=640, gif_fps=8, no_typing=False)
    for name, default in presentation.items():
        if getattr(args, name) != default:
            raise UsageError(f"--{name.replace('_', '-')} is for the terminal and camera demo",
                             hint="use --camera-only alone to export every camera at its recorded times")
    trial = Path(args.trial).expanduser().resolve()
    if not trial.is_dir():
        raise UsageError(f"Trial directory does not exist: {trial}")
    claim = lock.acquire(trial, stem="")
    if claim is None:
        raise UsageError("This trial is still running or another export is using it",
                         hint="retry after it finishes")
    try:
        result_path = trial / "result.json"
        try:
            result = json.loads(result_path.read_text()) if result_path.exists() else None
        except (OSError, ValueError) as exc:
            raise UsageError(f"Could not read {result_path}: {exc}") from exc
        if result is not None and not isinstance(result, dict):
            raise UsageError(f"Expected an object in {result_path}")
        output = Path(args.out).expanduser().resolve() if args.out else trial / "videos"
        try:
            report = export_camera_videos(trial / "frames", output)
        except KeyboardInterrupt:
            if result is not None:
                result["video_export"] = {"status": "canceled", "errors": ["Video export was interrupted."]}
                record.write_result(trial, result)
            raise
        if result is not None:
            result["video_export"] = report
            record.write_result(trial, result)
        for name, camera in report.get('cameras', {}).items():
            if camera.get('file'):
                print(f"{name}: {camera['file']}")
        if report['status'] == 'completed':
            print('Camera videos exported at their recorded times. Original frames are kept.')
        else:
            print('Camera video export is incomplete. Original frames are kept.')
            for error in report.get('errors', []):
                print(error)
        if result is not None:
            print(f"Export details: {result_path}")
        return 0 if report["status"] == "completed" else 1
    finally:
        claim.release()


def add_parser(sub) -> None:
    p = sub.add_parser(
        "demo", help="export camera videos or a terminal and camera demo from a recorded trial",
        description="Compose demo.mp4 from a trial that ran with --record: the "
        "commands typed on the left, the robot's cameras on the right, one "
        "sim step per frame. This edited demo needs the demo extra "
        "(pip install 'openrua[demo]'). Use --camera-only to export one MP4 "
        "per camera at its recorded times; no extra installation is needed.")
    p.add_argument("trial", help="a trial directory under runs/ that ran with --record")
    p.add_argument("--out", default=None,
                   help="output file (default: <trial>/demo.mp4); with --camera-only, "
                        "output directory (default: <trial>/videos/)")
    p.add_argument("--camera-only", action="store_true",
                   help="export every recorded camera to its own MP4 using the frame timestamps")
    p.add_argument("--gif", action="store_true",
                   help="also write demo.gif next to it (640 px wide, 8 fps; keep it "
                   "short with --ops and --speed)")
    p.add_argument("--cameras", default="",
                   help="main view and inset by name (default: the first two recorded)")
    p.add_argument("--pace", type=float, default=1.0, metavar="FACTOR",
                   help="the terminal's clock: commands type FACTOR times faster "
                        "and the rests between events shrink by FACTOR (default 1; "
                        "the robot's clock is --speed). Most of a clip is typing: "
                        "a trial of a few hundred sim steps and fifty commands is "
                        "seconds of motion and minutes of terminal")
    p.add_argument("--from-motion", type=float, default=None, metavar="SECONDS",
                   help="start the clip this many seconds (on the trial's clock) "
                        "before the robot first moves, skipping the agent's "
                        "reading of the robot and the scene; the terminal notes "
                        "how many commands came earlier")
    p.add_argument("--ops", default=None, metavar="START:END",
                   help="only the operations with index in [START, END) as numbered "
                   "in ops.jsonl (default all; a README clip wants the last few)")
    p.add_argument("--size", default=None, metavar="WxH", help="video size (default 1280x720)")
    p.add_argument("--fps", type=int, default=20, help="frames per second (default 20)")
    p.add_argument("--quality", type=int, default=18, metavar="CRF",
                   help="x264 constant rate factor, 0 lossless to 51 worst (default 18, "
                   "visually lossless; 23 halves the file again)")
    p.add_argument("--speed", type=float, default=1.0,
                   help="sim steps per video frame (default 1: real-time robot motion; "
                        "a trial recorded one step in N plays N at least)")
    p.add_argument("--font-size", type=int, default=13, help="terminal font size (default 13)")
    p.add_argument("--gif-width", type=int, default=640, help="gif width in pixels (default 640)")
    p.add_argument("--gif-fps", type=int, default=8, help="gif frames per second (default 8)")
    p.add_argument("--no-typing", action="store_true",
                   help="show each command at once instead of typing it out")
    p.set_defaults(fn=run)
