"""``robocli demo <trial>``: a video of a recorded trial."""

from __future__ import annotations

from pathlib import Path

from robocli import demo


def run(args) -> int:
    style = demo.Style(fps=args.fps, font_size=args.font_size, speed=args.speed,
                       typing=not args.no_typing)
    if args.size:
        w, h = args.size.lower().split("x")
        style.width, style.height = int(w), int(h)
    start, _, end = (args.ops or ":").partition(":")
    out = demo.render(Path(args.trial), Path(args.out) if args.out else None,
                      cameras=tuple(c for c in args.cameras.split(",") if c),
                      style=style, gif=args.gif,
                      ops_range=(int(start) if start else None, int(end) if end else None))
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
        "(pip install 'robocli-harness[demo]').")
    p.add_argument("trial", help="a trial directory under runs/ that ran with --record")
    p.add_argument("--out", default=None, help="output file (default: <trial>/demo.mp4)")
    p.add_argument("--gif", action="store_true",
                   help="also write demo.gif next to it (640 px wide, 8 fps; keep it "
                   "short with --ops and --speed)")
    p.add_argument("--cameras", default="",
                   help="main view and inset by name (default: the first two recorded)")
    p.add_argument("--ops", default=None, metavar="START:END",
                   help="only the operations with index in [START, END) as numbered "
                   "in ops.jsonl (default all; a README clip wants the last few)")
    p.add_argument("--size", default=None, metavar="WxH", help="video size (default 1280x720)")
    p.add_argument("--fps", type=int, default=20, help="frames per second (default 20)")
    p.add_argument("--speed", type=float, default=1.0,
                   help="sim steps per video frame (default 1: real-time robot motion)")
    p.add_argument("--font-size", type=int, default=13, help="terminal font size (default 13)")
    p.add_argument("--no-typing", action="store_true",
                   help="show each command at once instead of typing it out")
    p.set_defaults(fn=run)
