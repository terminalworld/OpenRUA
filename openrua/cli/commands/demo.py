"""``openrua demo <trial>``: a video of a recorded trial."""

from __future__ import annotations

from pathlib import Path

from openrua import demo


def run(args) -> int:
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


def add_parser(sub) -> None:
    p = sub.add_parser(
        "demo", help="render a recorded trial as a video (terminal + cameras)",
        description="Compose demo.mp4 from a trial that ran with --record: the "
        "commands typed on the left, the robot's cameras on the right, one "
        "sim step per frame. Needs the demo extra "
        "(pip install 'openrua[demo]').")
    p.add_argument("trial", help="a trial directory under runs/ that ran with --record")
    p.add_argument("--out", default=None, help="output file (default: <trial>/demo.mp4)")
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
