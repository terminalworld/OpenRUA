"""Compose ``demo.mp4`` (and ``demo.gif``) from a trial's frames and
timed operations.

Timeline: one beat per operation. A beat types the command, prints the
head of its output, then plays the frames the robot wrote while that
operation ran (matched by wall time: under the paused clock nothing
moves between operations), one sim step per video frame. An operation
that moved nothing holds briefly. The task sentence sits above the
camera; the verdict closes the video.
"""

from __future__ import annotations

import json
from dataclasses import dataclass, field
from pathlib import Path

from robocli.errors import NotFound, UnavailableError

MONO_FONTS = (
    "/usr/share/fonts/truetype/dejavu/DejaVuSansMono.ttf",
    "/usr/share/fonts/truetype/liberation/LiberationMono-Regular.ttf",
    "/usr/share/fonts/TTF/DejaVuSansMono.ttf",
    "/System/Library/Fonts/Menlo.ttc",
)


@dataclass
class Style:
    """Everything about how the video looks. Sizes in pixels, times in
    seconds; ``speed`` plays that many sim steps per video frame."""
    width: int = 1280
    height: int = 720
    fps: int = 20
    font_size: int = 13
    speed: float = 1.0
    typing_chars_per_s: float = 80.0
    typing_max_s: float = 2.5
    output_lines: int = 8
    hold_s: float = 0.7
    closing_s: float = 2.0
    font: str | None = None
    background: tuple = (12, 12, 16)
    panel: tuple = (24, 24, 28)
    text: tuple = (222, 222, 222)
    dim: tuple = (130, 130, 140)
    accent: tuple = (97, 175, 111)
    title: tuple = (240, 220, 130)
    gif_width: int = 640
    gif_fps: int = 8
    extra: dict = field(default_factory=dict)


def load(trial: Path) -> tuple[list[dict], list[dict], dict]:
    """The trial's frames index, its timed operations and its result.
    Frames come from ``robocli run --record``; a trial without them is
    refused with the replay command that produces them."""
    trial = Path(trial)
    index = trial / "frames" / "index.jsonl"
    ops = trial / "ops.jsonl"
    result = trial / "result.json"
    if not result.is_file():
        raise NotFound(f"{trial} is not a trial directory (no result.json)")
    if not index.is_file():
        raise NotFound(f"{trial} has no recorded frames",
                       hint=replay_hint(trial))
    if not ops.is_file():
        raise NotFound(f"{trial} has no ops.jsonl (a trial written before "
                       "recording existed)", hint=replay_hint(trial))
    return (_jsonl(index), _jsonl(ops), json.loads(result.read_text()))


def replay_hint(trial: Path) -> str:
    """The command that replays this trial with recording on."""
    trial = Path(trial)
    try:
        prov = json.loads((trial / "provenance.json").read_text())
        r = json.loads((trial / "result.json").read_text())
    except (OSError, json.JSONDecodeError):
        return "replay it with: robocli run ... --operator script --script <trial>/commands.sh --record"
    run_id = trial.parents[2].name if len(trial.parents) > 2 else "demo"
    return (f"robocli run --config {prov.get('config_file', '<benchmark>')} "
            f"--run-id {run_id}-demo --task-suite {r.get('task_suite')} "
            f"--task-ids {r.get('task_id')} --seeds {r.get('init_state_id')} "
            f"--operator script --script {trial / 'commands.sh'} --record")


def _jsonl(path: Path) -> list[dict]:
    out = []
    for line in path.read_text().splitlines():
        if line.strip():
            out.append(json.loads(line))
    return out


def beats(index: list[dict], ops: list[dict]) -> list[dict]:
    """Each operation with the frame records written while it ran.
    Frames before the first operation (the opening state) belong to
    nobody and are the video's first image."""
    out = []
    for op in ops:
        t0, t1 = op.get("t0"), op.get("t1")
        mine = [f for f in index
                if t0 is not None and t1 is not None and t0 <= f["t"] <= t1]
        out.append({**op, "frames": mine})
    return out


def render(trial: Path, out: Path | None = None, *, cameras: tuple[str, ...] = (),
           style: Style | None = None, gif: bool = False,
           ops_range: tuple[int | None, int | None] = (None, None)) -> Path:
    """Write the video and return its path. ``cameras`` picks the main
    view and the inset by name (default: the first two recorded);
    ``ops_range`` renders only the operations whose index falls in
    ``[start, end)`` (a README clip wants the last few)."""
    try:
        import imageio.v2 as imageio
        import numpy as np
        from PIL import Image, ImageDraw, ImageFont
    except ImportError as exc:
        raise UnavailableError(f"rendering needs the demo extra ({exc.name} missing)",
                               hint="pip install 'robocli-harness[demo]'") from exc
    style = style or Style()
    trial = Path(trial)
    index, ops, result = load(trial)
    start, end = ops_range
    ops = [op for op in ops
           if (start is None or op["i"] >= start) and (end is None or op["i"] < end)]
    if not ops:
        raise NotFound(f"no operations in the range {start}:{end}")
    frames_dir = trial / "frames"
    recorded = _camera_names(index)
    main_cam, inset_cam = _pick(cameras, recorded)
    out = Path(out) if out else trial / "demo.mp4"

    font = _font(style, style.font_size, ImageFont)
    title_font = _font(style, style.font_size + 2, ImageFont)
    term_w = style.width // 2
    line_h = style.font_size + 3
    cols = max(20, int((term_w - 20) / (style.font_size * 0.6)))
    rows = max(5, (style.height - 20) // line_h)
    cam_box = (term_w + 20, 90, style.width - 20, style.height - 20)
    cam_w, cam_h = cam_box[2] - cam_box[0], cam_box[3] - cam_box[1]
    inset_w = cam_w * 3 // 10
    inset_h = inset_w * 3 // 4
    term = _Terminal(cols, rows)
    writer = imageio.get_writer(str(out), fps=style.fps, codec="libx264",
                                quality=8, macro_block_size=None)
    gif_frames: list = []
    gif_every = max(1, round(style.fps / style.gif_fps))
    count = 0

    pictures: dict[tuple[str, tuple], object] = {}

    def picture(frame: dict | None, cam: str, size: tuple[int, int]):
        """The camera image of a frame at a size, resized once."""
        if frame is None:
            return None
        name = f"{frame['step']:06d}_{cam}.jpg"
        key = (name, size)
        if key not in pictures:
            p = frames_dir / name
            pictures.clear() if len(pictures) > 8 else None
            pictures[key] = Image.open(p).resize(size) if p.is_file() else None
        return pictures[key]

    # The opening image is the last frame written before the first
    # rendered operation (the reset's, or where a skipped prefix left
    # the robot).
    first_t = ops[0].get("t0")
    before = [f for f in index if first_t is None or f["t"] < first_t]
    last_frame = before[-1] if before else (index[0] if index else None)

    def emit(n: int, cursor: bool = False) -> None:
        nonlocal count
        for _ in range(n):
            canvas = Image.new("RGB", (style.width, style.height), style.background)
            draw = ImageDraw.Draw(canvas)
            term.paint(draw, font, line_h, cursor, style)
            draw.rectangle([term_w, 0, style.width, style.height], fill=style.panel)
            y = 14
            for tl in _wrap(result.get("task_language", ""), cols * 2 // 3)[:3]:
                draw.text((term_w + 16, y), tl, font=title_font, fill=style.title)
                y += line_h + 4
            main = picture(last_frame, main_cam, (cam_w, cam_h))
            if main is not None:
                canvas.paste(main, cam_box[:2])
            inset = (picture(last_frame, inset_cam, (inset_w, inset_h))
                     if inset_cam else None)
            if inset is not None:
                x0, y0 = style.width - 20 - inset_w, style.height - 20 - inset_h
                canvas.paste(inset, (x0, y0))
                draw.rectangle([x0, y0, x0 + inset_w, y0 + inset_h], outline=style.text)
            arr = np.asarray(canvas)
            writer.append_data(arr)
            if gif and count % gif_every == 0:
                gif_frames.append(canvas.resize(
                    (style.gif_width, style.gif_width * style.height // style.width)))
            count += 1

    for beat in beats(index, ops):
        shown = beat["command"]
        if len(shown) > 3 * cols:
            shown = shown[:3 * cols - 3] + "..."
        n_type = max(4, int(style.fps * min(len(shown) / style.typing_chars_per_s,
                                            style.typing_max_s)))
        mark = term.mark()
        for k in range(n_type):
            partial = shown[: max(1, int(len(shown) * (k + 1) / n_type))]
            term.reset(mark)
            term.add("$ " + partial, style.accent)
            emit(1, cursor=True)
        term.reset(mark)
        term.add("$ " + shown, style.accent)
        lines = [ln for ln in beat.get("output", "").splitlines() if ln.strip()]
        if lines[: style.output_lines]:
            term.add("\n".join(lines[: style.output_lines]), style.dim)
        if len(lines) > style.output_lines:
            term.add(f"... ({len(lines) - style.output_lines} more lines)", style.dim)
        moved = False
        stride = max(1, int(round(style.speed)))
        for f in beat["frames"][::stride]:
            last_frame = f
            emit(1)
            moved = True
        if not moved:
            emit(int(style.fps * style.hold_s))

    verdict = "TASK SUCCESS" if result.get("success") else "REPLAY END"
    term.add("", style.text)
    term.add(f"== {verdict} ==", style.accent)
    emit(int(style.fps * style.closing_s))
    writer.close()
    if gif and gif_frames:
        gif_frames[0].save(out.with_suffix(".gif"), save_all=True,
                           append_images=gif_frames[1:], loop=0,
                           duration=int(1000 / style.gif_fps))
    return out


def _camera_names(index: list[dict]) -> list[str]:
    names: list[str] = []
    for f in index:
        for name in f.get("files", []):
            cam = name.split("_", 1)[1].rsplit(".", 1)[0]
            if cam not in names:
                names.append(cam)
    return names


def _pick(cameras: tuple[str, ...], recorded: list[str]) -> tuple[str, str | None]:
    if not recorded:
        raise NotFound("the frames index names no camera files")
    chosen = list(cameras) or recorded[:2]
    missing = [c for c in chosen if c not in recorded]
    if missing:
        raise NotFound(f"camera(s) {', '.join(missing)} were not recorded "
                       f"(recorded: {', '.join(recorded)})")
    return chosen[0], (chosen[1] if len(chosen) > 1 else None)


def _font(style: Style, size: int, ImageFont):
    for path in ([style.font] if style.font else []) + list(MONO_FONTS):
        try:
            return ImageFont.truetype(path, size)
        except OSError:
            continue
    return ImageFont.load_default(size)


def _wrap(text: str, cols: int) -> list[str]:
    out, line = [], ""
    for word in text.split():
        if len(line) + len(word) + 1 > cols:
            out.append(line)
            line = word
        else:
            line = f"{line} {word}".strip()
    if line:
        out.append(line)
    return out


class _Terminal:
    """Scrollback of (text, color) lines, wrapped to the panel."""

    def __init__(self, cols: int, rows: int):
        self.cols, self.rows = cols, rows
        self.lines: list[tuple[str, tuple]] = []

    def add(self, text: str, color) -> None:
        for raw in text.split("\n"):
            while len(raw) > self.cols:
                self.lines.append((raw[: self.cols], color))
                raw = raw[self.cols:]
            self.lines.append((raw, color))
        self.lines = self.lines[-400:]

    def mark(self) -> int:
        return len(self.lines)

    def reset(self, mark: int) -> None:
        del self.lines[mark:]

    def paint(self, draw, font, line_h: int, cursor: bool, style: Style) -> None:
        view = self.lines[-self.rows:]
        y = 10
        for text, color in view:
            draw.text((10, y), text, font=font, fill=color)
            y += line_h
        if cursor and view:
            w = draw.textlength(view[-1][0], font=font)
            draw.rectangle([12 + w, y - line_h, 12 + w + style.font_size * 0.6, y - 3],
                           fill=style.text)
