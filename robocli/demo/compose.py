"""Compose ``demo.mp4`` (and ``demo.gif``) from a trial's frames and
timed operations.

Timeline: one beat per operation. A beat types the command, prints the
head of its output, then plays the frames the robot wrote while that
operation ran (matched by wall time: under the paused clock nothing
moves between operations), one sim step per video frame. An operation
that moved nothing holds briefly. The task sentence sits above the
camera; the verdict closes the video.

``load`` and ``beats`` are pure file logic; ``Canvas`` paints one frame
of the layout; ``render`` walks the beats and feeds the encoder.
"""

from __future__ import annotations

import json
from dataclasses import dataclass
from pathlib import Path

from robocli.errors import NotFound, UnavailableError

MONO_FONTS = (
    "/usr/share/fonts/truetype/dejavu/DejaVuSansMono.ttf",
    "/usr/share/fonts/truetype/liberation/LiberationMono-Regular.ttf",
    "/usr/share/fonts/TTF/DejaVuSansMono.ttf",
    "/System/Library/Fonts/Menlo.ttc",
)
MARGIN = 10
CAMERA_TOP = 90


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


# ------------------------------------------------------------ trial files

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
        raise NotFound(f"{trial} has no recorded frames", hint=replay_hint(trial))
    if not ops.is_file():
        raise NotFound(f"{trial} has no ops.jsonl (a trial written before "
                       "recording existed)", hint=replay_hint(trial))
    return _jsonl(index), _jsonl(ops), json.loads(result.read_text())


def replay_hint(trial: Path) -> str:
    """The command that replays this trial with recording on."""
    trial = Path(trial)
    try:
        prov = json.loads((trial / "provenance.json").read_text())
        r = json.loads((trial / "result.json").read_text())
    except (OSError, json.JSONDecodeError):
        return ("replay it with: robocli run ... --operator script "
                "--script <trial>/commands.sh --record")
    run_id = trial.parents[2].name if len(trial.parents) > 2 else "demo"
    return (f"robocli run --config {prov.get('config_file', '<benchmark>')} "
            f"--run-id {run_id}-demo --task-suite {r.get('task_suite')} "
            f"--task-ids {r.get('task_id')} --seeds {r.get('init_state_id')} "
            f"--operator script --script {trial / 'commands.sh'} --record")


def _jsonl(path: Path) -> list[dict]:
    return [json.loads(line) for line in path.read_text().splitlines() if line.strip()]


def beats(index: list[dict], ops: list[dict]) -> list[dict]:
    """Each operation with the frame records written while it ran."""
    out = []
    for op in ops:
        t0, t1 = op.get("t0"), op.get("t1")
        mine = [f for f in index
                if t0 is not None and t1 is not None and t0 <= f["t"] <= t1]
        out.append({**op, "frames": mine})
    return out


def camera_names(index: list[dict]) -> list[str]:
    """The cameras the index names, in recording order."""
    names: list[str] = []
    for f in index:
        for name in f.get("files", []):
            cam = name.split("_", 1)[1].rsplit(".", 1)[0]
            if cam not in names:
                names.append(cam)
    return names


def pick_cameras(cameras: tuple[str, ...], recorded: list[str]) -> tuple[str, str | None]:
    """The main view and the inset: the names asked for, else the first
    two recorded."""
    if not recorded:
        raise NotFound("the frames index names no camera files")
    chosen = list(cameras) or recorded[:2]
    missing = [c for c in chosen if c not in recorded]
    if missing:
        raise NotFound(f"camera(s) {', '.join(missing)} were not recorded "
                       f"(recorded: {', '.join(recorded)})")
    return chosen[0], (chosen[1] if len(chosen) > 1 else None)


# --------------------------------------------------------------- painting

class Terminal:
    """Scrollback of (text, color) lines, wrapped to the panel's columns."""

    def __init__(self, cols: int, rows: int):
        self.cols, self.rows = cols, rows
        self.lines: list[tuple[str, tuple]] = []

    def add(self, text: str, color: tuple) -> None:
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

    def visible(self) -> list[tuple[str, tuple]]:
        return self.lines[-self.rows:]


class Canvas:
    """One frame of the layout: terminal on the left, task sentence and
    main camera on the right, inset bottom-right. Holds the fonts, the
    terminal scrollback and a small cache of resized camera images."""

    def __init__(self, style: Style, frames_dir: Path, task: str,
                 main_cam: str, inset_cam: str | None, pil):
        self.style, self.frames_dir, self.pil = style, frames_dir, pil
        self.main_cam, self.inset_cam = main_cam, inset_cam
        self.font = self._font(style.font_size)
        self.title_font = self._font(style.font_size + 2)
        self.line_h = style.font_size + 3
        self.term_w = style.width // 2
        cols = max(20, int((self.term_w - 2 * MARGIN) / (style.font_size * 0.6)))
        rows = max(5, (style.height - 2 * MARGIN) // self.line_h)
        self.terminal = Terminal(cols, rows)
        self.title = _wrap(task, cols * 2 // 3)[:3]
        x0, y0 = self.term_w + 2 * MARGIN, CAMERA_TOP
        self.camera_box = (x0, y0, style.width - 2 * MARGIN - x0, style.height - 2 * MARGIN - y0)
        inset_w = self.camera_box[2] * 3 // 10
        self.inset_size = (inset_w, inset_w * 3 // 4)
        self._pictures: dict = {}

    @property
    def cols(self) -> int:
        return self.terminal.cols

    def _font(self, size: int):
        ImageFont = self.pil.ImageFont
        for path in ([self.style.font] if self.style.font else []) + list(MONO_FONTS):
            try:
                return ImageFont.truetype(path, size)
            except OSError:
                continue
        return ImageFont.load_default(size)

    def picture(self, frame: dict | None, cam: str | None, size: tuple[int, int]):
        """A frame's camera image at a size, resized once per frame."""
        if frame is None or cam is None:
            return None
        key = (frame["step"], cam, size)
        if key not in self._pictures:
            if len(self._pictures) > 8:
                self._pictures.clear()
            path = self.frames_dir / f"{frame['step']:06d}_{cam}.jpg"
            self._pictures[key] = (self.pil.Image.open(path).resize(size)
                                   if path.is_file() else None)
        return self._pictures[key]

    def paint(self, frame: dict | None, cursor: bool = False):
        """The whole frame as a PIL image."""
        st = self.style
        Image, ImageDraw = self.pil.Image, self.pil.ImageDraw
        canvas = Image.new("RGB", (st.width, st.height), st.background)
        draw = ImageDraw.Draw(canvas)
        # terminal
        y = MARGIN
        view = self.terminal.visible()
        for text, color in view:
            draw.text((MARGIN, y), text, font=self.font, fill=color)
            y += self.line_h
        if cursor and view:
            w = draw.textlength(view[-1][0], font=self.font)
            draw.rectangle([MARGIN + 2 + w, y - self.line_h,
                            MARGIN + 2 + w + st.font_size * 0.6, y - 3], fill=st.text)
        # panel: task sentence, main camera, inset
        draw.rectangle([self.term_w, 0, st.width, st.height], fill=st.panel)
        y = 14
        for line in self.title:
            draw.text((self.term_w + 16, y), line, font=self.title_font, fill=st.title)
            y += self.line_h + 4
        x0, y0, w, h = self.camera_box
        main = self.picture(frame, self.main_cam, (w, h))
        if main is not None:
            canvas.paste(main, (x0, y0))
        inset = self.picture(frame, self.inset_cam, self.inset_size)
        if inset is not None:
            ix, iy = (st.width - 2 * MARGIN - self.inset_size[0],
                      st.height - 2 * MARGIN - self.inset_size[1])
            canvas.paste(inset, (ix, iy))
            draw.rectangle([ix, iy, ix + self.inset_size[0], iy + self.inset_size[1]],
                           outline=st.text)
        return canvas


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


# ---------------------------------------------------------------- render

class _Encoder:
    """The mp4 writer, plus the gif's downsampled frames when asked."""

    def __init__(self, out: Path, style: Style, gif: bool, imageio, np):
        self.out, self.style, self.np = out, style, np
        self.writer = imageio.get_writer(str(out), fps=style.fps, codec="libx264",
                                         quality=8, macro_block_size=None)
        self.gif_frames: list = [] if gif else None
        self.gif_every = max(1, round(style.fps / style.gif_fps))
        self.count = 0

    def add(self, image) -> None:
        self.writer.append_data(self.np.asarray(image))
        if self.gif_frames is not None and self.count % self.gif_every == 0:
            st = self.style
            self.gif_frames.append(image.resize(
                (st.gif_width, st.gif_width * st.height // st.width)))
        self.count += 1

    def close(self) -> None:
        self.writer.close()
        if self.gif_frames:
            self.gif_frames[0].save(self.out.with_suffix(".gif"), save_all=True,
                                    append_images=self.gif_frames[1:], loop=0,
                                    duration=int(1000 / self.style.gif_fps))


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
        import PIL.Image
        import PIL.ImageDraw
        import PIL.ImageFont
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
    main_cam, inset_cam = pick_cameras(cameras, camera_names(index))
    out = Path(out) if out else trial / "demo.mp4"
    canvas = Canvas(style, trial / "frames", result.get("task_language", ""),
                    main_cam, inset_cam, PIL)
    term = canvas.terminal
    encoder = _Encoder(out, style, gif, imageio, np)

    # The opening image: the last frame written before the first rendered
    # operation (the reset's, or where a skipped prefix left the robot).
    first_t = ops[0].get("t0")
    before = [f for f in index if first_t is None or f["t"] < first_t]
    frame = before[-1] if before else (index[0] if index else None)

    def emit(n: int, cursor: bool = False) -> None:
        for _ in range(n):
            encoder.add(canvas.paint(frame, cursor))

    stride = max(1, int(round(style.speed)))
    for beat in beats(index, ops):
        shown = beat["command"]
        if len(shown) > 3 * canvas.cols:
            shown = shown[: 3 * canvas.cols - 3] + "..."
        n_type = max(4, int(style.fps * min(len(shown) / style.typing_chars_per_s,
                                            style.typing_max_s)))
        mark = term.mark()
        for k in range(n_type):
            term.reset(mark)
            term.add("$ " + shown[: max(1, int(len(shown) * (k + 1) / n_type))], style.accent)
            emit(1, cursor=True)
        term.reset(mark)
        term.add("$ " + shown, style.accent)
        lines = [ln for ln in beat.get("output", "").splitlines() if ln.strip()]
        if lines[: style.output_lines]:
            term.add("\n".join(lines[: style.output_lines]), style.dim)
        if len(lines) > style.output_lines:
            term.add(f"... ({len(lines) - style.output_lines} more lines)", style.dim)
        motion = beat["frames"][::stride]
        for frame in motion:
            emit(1)
        if not motion:
            emit(int(style.fps * style.hold_s))

    verdict = "TASK SUCCESS" if result.get("success") else "REPLAY END"
    term.add("", style.text)
    term.add(f"== {verdict} ==", style.accent)
    emit(int(style.fps * style.closing_s))
    encoder.close()
    return out
