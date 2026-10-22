"""A robot's cameras seen from the host: which topics they publish on, and
a tap that reads their frames through the robot's sandbox.

``camera_topics`` reads the cameras from the resolved config, the way
machine.yaml states them. ``CameraTap`` runs camera_tap.py in the sandbox
and keeps each camera's latest frame; given a directory it also writes
every frame there, in the frames/ layout ``openrua demo`` reads (one
``index.jsonl`` line per tick with its wall time). ``CameraView`` is the
tap for a person watching: it runs while frames are asked for and stops
when nobody has looked for a while.
"""

from __future__ import annotations

import base64
import json
import subprocess
import threading
import time
from pathlib import Path
from typing import Callable

TAP = Path(__file__).with_name("camera_tap.py")
EXTENSION = {"image/jpeg": "jpg", "image/png": "png"}


def camera_topics(cfg: dict) -> tuple[dict[str, str], str | None]:
    """(camera name -> color image topic, pattern): a scene that decides
    its own cameras names none and gives the topic pattern instead
    ({name} for the camera's name)."""
    from openrua.sandbox.workspace import camera_entries

    cams = (cfg.get("machine") or {}).get("cameras") or {}
    if not cams:
        return {}, None   # a machine that says nothing about cameras has none to show
    entries = camera_entries(cams)
    named = {e["name"]: e["color"] for e in entries if e["kind"] == "camera" and e.get("color")}
    pattern = next((e["color"].replace("<name>", "{name}") for e in entries
                    if e["kind"] == "camera_set" and e.get("color")), None)
    return named, pattern


class CameraTap:
    """camera_tap.py running in ``sandbox``: the latest frame of each
    camera, and every frame written under ``record`` when one is given."""

    def __init__(self, sandbox: str, cameras: dict[str, str], fps: float = 2.0,
                 record: Path | None = None):
        self.sandbox, self.cameras, self.fps, self.record = sandbox, dict(cameras), fps, record
        self.latest: dict[str, dict] = {}
        self.ticks = 0
        self._proc: subprocess.Popen | None = None
        self._reader: threading.Thread | None = None
        self.lock = threading.Lock()

    def start(self) -> None:
        args = [f"--camera={n}={t}" for n, t in self.cameras.items()]
        if self.record is not None:
            self.record.mkdir(parents=True, exist_ok=True)
        self._proc = subprocess.Popen(
            ["docker", "exec", "-i", "-u", "robot", self.sandbox, "python3", "-c",
             TAP.read_text(), *args, f"--fps={self.fps}"],
            stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.DEVNULL,
            start_new_session=True)
        self._reader = threading.Thread(target=self._read, daemon=True)
        self._reader.start()

    @property
    def running(self) -> bool:
        return self._proc is not None and self._proc.poll() is None

    def _read(self) -> None:
        out = self._proc.stdout
        index = (self.record / "index.jsonl").open("a") if self.record is not None else None
        try:
            for line in iter(out.readline, b""):
                head = json.loads(line)
                frames = []
                for f in head.get("frames", []):
                    frames.append((f["name"], f["mime"], out.read(f["bytes"])))
                with self.lock:
                    for name, mime, data in frames:
                        self.latest[name] = {"t": head["t"], "mime": mime, "data": data}
                if index is not None and frames:
                    files = []
                    for name, mime, data in frames:
                        file = f"{self.ticks:06d}_{name}.{EXTENSION.get(mime, 'jpg')}"
                        (self.record / file).write_bytes(data)
                        files.append(file)
                    index.write(json.dumps({"step": self.ticks, "t": head["t"], "files": files}) + "\n")
                    index.flush()
                if frames:
                    self.ticks += 1
        except (OSError, ValueError, KeyError):
            pass
        finally:
            if index is not None:
                index.close()

    def stop(self) -> None:
        """Close the tap's stdin (it exits on its own) and wait for it."""
        proc, self._proc = self._proc, None
        if proc is None:
            return
        try:
            proc.stdin.close()
            proc.wait(timeout=10)
        except (OSError, subprocess.TimeoutExpired):
            proc.kill()
        if self._reader is not None:
            self._reader.join(timeout=5)


def list_cameras(sandbox: str, pattern: str) -> dict[str, str]:
    """The cameras a scene decided for itself: the topics matching
    ``pattern`` on the robot's graph, by name."""
    r = subprocess.run(["docker", "exec", "-u", "robot", sandbox, "python3", "-c",
                        TAP.read_text(), f"--pattern={pattern}", "--list"],
                       capture_output=True, timeout=60)
    first = r.stdout.split(b"\n", 1)[0]
    return json.loads(first).get("cameras", {}) if first.strip() else {}


class CameraView:
    """The cameras for a person watching. ``robot`` returns what the
    session holds now (it changes when a robot just connected moves to
    its own terminal): an object with ``sandbox`` and ``cameras`` (the
    ``camera_topics`` pair). One camera is read at a time, the one being
    watched (a simulated camera is rendered only while read); the tap
    starts on the first request, follows the camera asked for, and stops
    ``idle_s`` after the last request."""

    def __init__(self, robot: Callable[[], object], fps: float = 5.0, idle_s: float = 20.0):
        self.robot, self.fps, self.idle_s = robot, fps, idle_s
        self._tap: CameraTap | None = None
        self._listed: tuple[str, dict[str, str]] | None = None   # (sandbox, cameras)
        self._asked = 0.0
        self._lock = threading.Lock()
        self._stop = threading.Event()
        threading.Thread(target=self._idle, daemon=True).start()

    def _all(self) -> tuple[str, dict[str, str]]:
        """(sandbox, camera name -> topic) for the robot held now."""
        robot = self.robot()
        named, pattern = getattr(robot, "cameras", ({}, None))
        if named or not pattern:
            return robot.sandbox, dict(named)
        if self._listed is None or self._listed[0] != robot.sandbox:
            self._listed = (robot.sandbox, list_cameras(robot.sandbox, pattern))
        return self._listed

    def names(self) -> list[str]:
        return sorted(self._all()[1])

    def frame(self, name: str) -> dict:
        sandbox, cameras = self._all()
        if name not in cameras:
            raise ValueError(f"no camera named {name!r}" if cameras
                             else "this robot declares no cameras")
        with self._lock:
            self._asked = time.monotonic()
            tap = self._tap
            if tap is not None and (not tap.running or tap.sandbox != sandbox
                                    or set(tap.cameras) != {name}):
                tap.stop()
                tap = self._tap = None
            if tap is None:
                tap = self._tap = CameraTap(sandbox, {name: cameras[name]}, fps=self.fps)
                tap.start()
        with tap.lock:
            got = tap.latest.get(name)
        if got is None:
            return {"name": name, "waiting": True}
        return {"name": name, "t": got["t"], "age_s": round(time.time() - got["t"], 1),
                "mime": got["mime"], "data": base64.b64encode(got["data"]).decode()}

    def _idle(self) -> None:
        while not self._stop.wait(2.0):
            with self._lock:
                if self._tap is not None and time.monotonic() - self._asked > self.idle_s:
                    self._tap.stop()
                    self._tap = None

    def close(self) -> None:
        self._stop.set()
        with self._lock:
            if self._tap is not None:
                self._tap.stop()
                self._tap = None
