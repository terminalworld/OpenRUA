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
import logging
import math
import subprocess
import threading
import time
from pathlib import Path
from typing import Callable

TAP = Path(__file__).with_name("camera_tap.py")
EXTENSION = {"image/jpeg": "jpg", "image/png": "png"}


def _command(sandbox: str, *args: str, interactive: bool = False) -> list[str]:
    """Use the sandbox's Bash setup while keeping its output off the protocol."""
    # The outer POSIX shell leaves BASH_ENV to Bash. Keep the protocol
    # descriptor aside until setup has finished, routing its output to stderr.
    launch = ('exec bash --noprofile --norc -c \'exec python3 "$@" >&3 3>&-\' '
              'openrua-camera "$@" 3>&1 1>&2')
    return ["docker", "exec", *(["-i"] if interactive else []), "-u", "robot",
            sandbox, "sh", "-c", launch, "openrua-camera", "-c", TAP.read_text(), *args]


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
        self._errors_reader: threading.Thread | None = None
        self._errors: list[str] = []
        self._stderr = b""
        self._received: dict[str, dict] = {}
        self._started: float | None = None
        self._stopped: float | None = None
        self._stopping = False
        self.lock = threading.Lock()
        self._changed = threading.Condition(self.lock)

    def start(self) -> None:
        args = [f"--camera={n}={t}" for n, t in self.cameras.items()]
        if self.record is not None:
            self.record.mkdir(parents=True, exist_ok=True)
        self._started = time.time()
        self._proc = subprocess.Popen(
            _command(self.sandbox, *args, f"--fps={self.fps}", interactive=True),
            stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.PIPE,
            start_new_session=True)
        self._errors_reader = threading.Thread(target=self._read_errors, args=(self._proc,), daemon=True)
        self._errors_reader.start()
        self._reader = threading.Thread(target=self._read, args=(self._proc,), daemon=True)
        self._reader.start()

    @property
    def running(self) -> bool:
        return self._proc is not None and self._proc.poll() is None

    def _fail(self, message: str) -> None:
        with self._changed:
            if message not in self._errors:
                self._errors.append(message)
            self._changed.notify_all()

    def wait_for_frames(self, timeout_s: float) -> None:
        """Wait for a complete saved frame from every requested camera.

        This checks recording startup, not coverage of the subsequent task.
        Exceptions leave ownership with the caller, which must stop the tap.
        """
        if isinstance(timeout_s, bool) or not math.isfinite(timeout_s) or timeout_s <= 0:
            raise ValueError("The recording wait must be a positive number of seconds.")
        if self.record is None or not self.cameras:
            raise ValueError("Waiting for saved frames requires recording at least one camera.")
        deadline = time.monotonic() + timeout_s
        with self._changed:
            while True:
                missing = sorted(set(self.cameras) - set(self._received))
                error = "; ".join(self._errors)
                if not error and (self._stopping or self._stopped is not None):
                    error = "Camera recording stopped before the startup check completed."
                if not error and self._proc is None:
                    error = "Camera recording has not started."
                if not error and self._proc.poll() is not None:
                    error = f"The camera process exited with code {self._proc.returncode}."
                if error:
                    stderr = self._stderr.decode(errors="replace").strip()
                    if stderr:
                        error += "\n" + stderr
                    raise RuntimeError(f"Camera recording could not start: {error}\n"
                                       f"Camera log: {self.record / 'camera.log'}")
                if not missing:
                    return
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    raise TimeoutError(
                        f"No saved frame from {', '.join(missing)} after {timeout_s:g} seconds. "
                        "The task has not started. Check the camera topics and log before retrying.\n"
                        f"Camera log: {self.record / 'camera.log'}")
                self._changed.wait(remaining)

    def _read_errors(self, proc) -> None:
        log = None
        try:
            if self.record is not None:
                log = (self.record / "camera.log").open("ab")
            while data := proc.stderr.read1(4096):
                with self.lock:
                    self._stderr = (self._stderr + data)[-8192:]
                if log is not None:
                    log.write(data)
                    log.flush()
        except OSError as exc:
            self._fail(f"Could not retain camera diagnostics: {exc}")
        finally:
            if log is not None:
                log.close()
            proc.stderr.close()

    def _read(self, proc=None) -> None:
        out = (proc or self._proc).stdout
        index = None
        try:
            if self.record is not None:
                index = (self.record / "index.jsonl").open("a")
            for line in iter(out.readline, b""):
                head = json.loads(line)
                if not isinstance(head, dict) or not isinstance(head.get("frames"), list):
                    raise ValueError("expected a camera frame record")
                frames = []
                for f in head.get("frames", []):
                    name, mime, size = f["name"], f["mime"], f["bytes"]
                    if name not in self.cameras or mime not in EXTENSION:
                        raise ValueError(f"unexpected camera frame: {name!r}, {mime!r}")
                    if not isinstance(size, int) or isinstance(size, bool) or size <= 0:
                        raise ValueError(f"invalid camera frame size: {size!r}")
                    data = out.read(size)
                    if len(data) != size:
                        raise ValueError(f"incomplete frame from {name}: received {len(data)} of {size} bytes")
                    frames.append((name, mime, data))
                if index is not None and frames:
                    files = []
                    for name, mime, data in frames:
                        file = f"{self.ticks:06d}_{name}.{EXTENSION[mime]}"
                        (self.record / file).write_bytes(data)
                        files.append(file)
                    index.write(json.dumps({"step": self.ticks, "t": head["t"], "files": files}) + "\n")
                    index.flush()
                with self._changed:
                    for name, mime, data in frames:
                        self.latest[name] = {"t": head["t"], "mime": mime, "data": data}
                        received = self._received.setdefault(name, {"count": 0, "first_t": head["t"]})
                        received.update(count=received["count"] + 1, last_t=head["t"])
                    self._changed.notify_all()
                if frames:
                    self.ticks += 1
        except (OSError, ValueError, KeyError, TypeError) as exc:
            self._fail(f"Could not read camera frames: {exc}")
        finally:
            if index is not None:
                index.close()
            if proc is not None and not self._stopping:
                self._fail("The camera stream ended unexpectedly.")
            if proc is not None:
                out.close()

    def status(self) -> dict:
        """Received frames and recording errors, without inferring task success."""
        with self.lock:
            received = {name: dict(value) for name, value in self._received.items()}
            errors, stderr = list(self._errors), self._stderr.decode(errors="replace").strip()
        missing = sorted(set(self.cameras) - set(received))
        if errors:
            state = "failed"
        elif self._stopped is not None:
            state = "incomplete" if missing else "completed"
        else:
            state = "recording"
        result = {"status": state, "started_at": self._started, "stopped_at": self._stopped,
                  "cameras": received, "missing_cameras": missing, "errors": errors}
        if stderr:
            result["stderr"] = stderr
        if self.record is not None:
            result["log"] = str(self.record / "camera.log")
        return result

    def stop(self) -> None:
        """Close the tap's stdin (it exits on its own) and wait for it."""
        proc = self._proc
        if proc is None:
            return
        with self._changed:
            self._stopping = True
            self._changed.notify_all()
        shutdown_errors = []
        exited = False
        try:
            proc.stdin.close()
            proc.wait(timeout=10)
            exited = True
        except (OSError, subprocess.TimeoutExpired):
            shutdown_errors.append("The camera process did not stop after its input was closed.")
            try:
                proc.kill()
                proc.wait(timeout=5)
                exited = True
            except (OSError, subprocess.TimeoutExpired) as exc:
                shutdown_errors.append(f"Could not confirm that the camera process stopped: {exc}")
        readers_stopped = True
        if self._reader is not None:
            self._reader.join(timeout=5)
            if self._reader.is_alive():
                shutdown_errors.append("The camera frame reader did not stop.")
                readers_stopped = False
        if self._errors_reader is not None:
            self._errors_reader.join(timeout=5)
            if self._errors_reader.is_alive():
                shutdown_errors.append("The camera diagnostic reader did not stop.")
                readers_stopped = False
        for message in shutdown_errors:
            self._fail(message)
        if proc.returncode:
            self._fail(f"The camera process exited with code {proc.returncode}.")
        if exited and readers_stopped:
            self._proc = None
            self._stopped = time.time()
        status = self.status()
        if shutdown_errors:
            detail = "; ".join(status["errors"])
            if status.get("stderr"):
                detail += "\n" + status["stderr"]
            if self.record is not None:
                detail += f"\nCamera log: {self.record / 'camera.log'}"
            raise RuntimeError(detail)


def list_cameras(sandbox: str, pattern: str) -> dict[str, str]:
    """The cameras a scene decided for itself: the topics matching
    ``pattern`` on the robot's graph, by name."""
    r = subprocess.run(_command(sandbox, f"--pattern={pattern}", "--list"),
                       capture_output=True, timeout=60)
    if r.returncode:
        raise RuntimeError(f'camera discovery failed (exit {r.returncode}): '
                           + r.stderr.decode(errors='replace').strip())
    first = r.stdout.split(b"\n", 1)[0]
    try:
        return json.loads(first)['cameras']
    except (ValueError, KeyError, TypeError) as exc:
        raise RuntimeError('camera discovery returned no valid camera record; '
                           + r.stderr.decode(errors='replace').strip()) from exc


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
        self._generation = None
        self._lock = threading.Lock()
        self._stop = threading.Event()
        threading.Thread(target=self._idle, daemon=True).start()

    def _all(self) -> tuple[str, dict[str, str]]:
        """(sandbox, camera name -> topic) for the robot held now."""
        robot = self.robot()
        generation = (robot.sandbox, getattr(getattr(robot, 'connection', None), 'generation', 0))
        with self._lock:
            if generation != self._generation:
                if self._tap is not None:
                    self._tap.stop()
                    self._tap = None
                self._listed = None
                self._generation = generation
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
            if tap is not None and (tap.sandbox != sandbox or set(tap.cameras) != {name}):
                tap.stop()
                tap = self._tap = None
            if tap is None:
                tap = self._tap = CameraTap(sandbox, {name: cameras[name]}, fps=self.fps)
                tap.start()
            if not tap.running:
                tap.stop()
                self._tap = None
                failure = tap.status()
                detail = "; ".join(failure["errors"])
                if failure.get("stderr"):
                    detail += "\n" + failure["stderr"]
                raise RuntimeError("The camera stream stopped. Check the camera driver."
                                   + ("\n" + detail if detail else ""))
            failure = tap.status()
            if failure["errors"]:
                raise RuntimeError("; ".join(failure["errors"]) +
                                   ("\n" + failure["stderr"] if failure.get("stderr") else ""))
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
                    try:
                        self._tap.stop()
                    except Exception as exc:  # noqa: BLE001; retain the tap for cleanup retry
                        logging.getLogger(__name__).warning("Could not stop the camera viewer: %s", exc)
                        self._asked = time.monotonic()
                    else:
                        self._tap = None

    def close(self) -> None:
        self._stop.set()
        with self._lock:
            if self._tap is not None:
                self._tap.stop()
                self._tap = None
