"""The bridge's private control line: the host asks, the bridge answers.

The runner talks to the bridge process over its stdio: one JSON object
per line on stdin, one JSON reply per line on stdout (logs go to
stderr). The line carries the truth-side verbs only (reset to an
initial state, ask the original predicate, shutdown) and is unreachable
from the agent's sandbox: no exec path into the sim container, nothing
here touches DDS.

Protocol:
    {"cmd": "reset", "init_state_id": 3}   -> {"ok": true}
    {"cmd": "success"}                     -> {"ok": true, "success": false}
    {"cmd": "shutdown"}                    -> {"ok": true}  (then exit)

Two classes: ``ControlChannel`` is the server loop; ``Monitor`` holds
the verbs and the per-step success latch. One Monitor per env: after
every sim step it asks the benchmark's original predicate once and
remembers the first success (which step, what time); official eval
semantics latch success per step (RoboCasa's runner does
``successes |= check``, LIBERO/robosuite eval loops end on first
success), and the end-state verdict is reported alongside. All benchmark
knowledge arrives through the loader parameter and all engine knowledge
through the bound engine parameter (time, renders, poses); ``on_reset``
is a plain callable slot fired inside the same sim-thread job right
after the world is restored (the ROS graph re-aligns itself there).
"""

from __future__ import annotations

import json
import sys
import threading
import time
from typing import Callable


class Recording:
    """Where and what the monitor records: a directory and the cameras
    (name, width, height). ``frames/<step>_<camera>.jpg`` per camera and
    one ``index.jsonl`` line per step with its wall time, which is what
    a later rendering aligns against the timed command stream."""

    def __init__(self, directory: str, cameras: list[tuple[str, int, int]]):
        import os

        self.directory = directory
        self.cameras = cameras
        os.makedirs(directory, exist_ok=True)
        self._index = open(os.path.join(directory, "index.jsonl"), "a")

    def frame(self, step: int, engine) -> None:
        from PIL import Image

        files = []
        for cam, w, h in self.cameras:
            px = engine.render(cam, w, h)
            name = f"{step:06d}_{cam}.jpg"
            Image.fromarray(px).save(f"{self.directory}/{name}", quality=85)
            files.append(name)
        self._index.write(json.dumps({"step": step, "t": time.time(),
                                      "files": files}) + "\n")
        self._index.flush()


class ControlChannel:
    def __init__(
        self,
        handlers: dict[str, Callable[[dict], dict]],
        on_eof: Callable[[], None],
        out=None,
    ):
        self._handlers = handlers
        self._on_eof = on_eof  # runner gone -> bridge exits cleanly (no orphans)
        # `out` is the PRIVATE handle to the real stdout (see boot.main's fd
        # redirection); simulator prints can never pollute the channel.
        self._out = out if out is not None else sys.stdout
        self._thread = threading.Thread(target=self._loop, daemon=True)

    def start(self) -> None:
        self._thread.start()

    def _loop(self) -> None:
        for line in sys.stdin:
            line = line.strip()
            if not line:
                continue
            rid = None
            try:
                req = json.loads(line)
                rid = req.get("id")
                handler = self._handlers[req["cmd"]]
                resp = {"ok": True, **(handler(req) or {})}
            except Exception as exc:  # noqa: BLE001; report, never die
                resp = {"ok": False, "error": f"{type(exc).__name__}: {exc}"}
            if rid is not None:
                # Echoed request id: lets the host discard late answers to
                # abandoned (timed-out) requests instead of mistaking them
                # for the next reply.
                resp["id"] = rid
            try:
                self._out.write(json.dumps(resp) + "\n")
                self._out.flush()
            except (OSError, ValueError):
                # Runner died mid-rpc: the answer pipe broke before stdin
                # reported EOF. Same meaning, same exit; an unhandled
                # BrokenPipeError here would kill this thread and skip
                # the no-orphans shutdown below.
                break
        self._on_eof()


class Monitor:
    def __init__(self, env, task_ctx: dict, loader, engine, sim,
                 on_reset=None, record: "Recording | None" = None,
                 hand_body: str = "robot0_right_hand"):
        self._env = env
        self._hand_body = hand_body  # the debug hand verb's body (the first arm's)
        self._ctx = task_ctx
        self._loader = loader  # the environment package's plug (parameter)
        self._engine = engine  # the engine package's plug (parameter)
        self._sim = sim  # Worker: all env access goes through it
        self.on_reset = on_reset or (lambda: None)
        # Camera recording, off unless a Recording is handed in: then
        # every sim step (and the opening state after a reset) writes
        # the named cameras as JPEGs plus one index line. Evaluation
        # runs pass None and take this branch zero times.
        self._record = record

        self._latched = False
        self._at_step = None
        self._at_sim_time = None
        self._at_wall = None
        self._steps = 0

        self._inner_step = env.step
        env.step = self._latching_step

    # ------------------------------------------------------------- stepping
    def _record_frame(self) -> None:
        self._record.frame(self._steps, self._engine)

    def _latching_step(self, action, **kwargs):  # kwargs: cap-x fork's
        r = self._inner_step(action, **kwargs)  # skip_render_images etc.
        self._steps += 1
        if self._record is not None:
            self._record_frame()
        if not self._latched and self._loader.success(self._env):
            self._latched = True
            # Time-to-success: the agent knows none of the budgets, so a
            # generous run re-judged under any tighter budget is
            # behaviorally exact; these stamps are what post-hoc judging
            # looks up.
            self._at_step = self._steps
            self._at_sim_time = self._engine.time()
            self._at_wall = time.time()
        return r

    # ----------------------------------------------------- answering verbs
    def reset(self, req: dict) -> dict:
        state = self._loader.init_state(self._ctx, int(req["init_state_id"]))

        def job():
            # How a world restores is the loader's protocol (benchmark
            # knowledge); the monitor only says when.
            self._loader.reset(self._env, self._ctx, state)
            self._latched = False  # new episode, fresh latch
            # Episode-scoped step counter: simulator resets may pump
            # settling steps through env.step; zeroing here makes
            # success_at.step mean "steps since reset" uniformly across
            # benchmarks.
            self._steps = 0
            if self._record is not None:
                self._record_frame()  # opening frame before any motion
            self.on_reset()

        self._sim.submit(job)
        return {}

    def success(self, _req: dict) -> dict:
        def job():
            now = bool(self._loader.success(self._env))
            out = {"success": bool(self._latched) or now,
                   "success_end_state": now}
            if self._latched:
                out["success_at"] = {
                    "step": self._at_step,
                    "sim_time_s": self._at_sim_time,
                    "wall_unix": self._at_wall,
                }
            return out

        return self._sim.submit(job)

    def task_info(self, _req: dict) -> dict:
        # Sentence semantics are the loader's (bddl-authoritative on
        # libero, per-episode ep_meta on robocasa); may touch the env,
        # so it runs as a sim-thread job. Loaders use .get, never bare
        # indexing: the channel reports handler exceptions instead of
        # dying, so a KeyError here would silently hand the agent an
        # empty task.
        return self._sim.submit(
            lambda: self._loader.task_info(self._env, self._ctx))

    def steps(self, _req: dict) -> dict:
        """Steps since reset and the latch, for step-economy statistics."""
        return {"steps": self._steps, "success_latched": bool(self._latched)}

    # ------------------------------------------------------ debug truth reads
    # Truth belongs to the evaluator; used for anomaly forensics; never
    # by the agent (the channel these ride is physically unreachable
    # from the sandbox).
    def objects(self, _req: dict) -> dict:
        return {"objects": self._sim.submit(self._engine.objects)}

    def hand(self, _req: dict) -> dict:
        def job():
            # Grasp point between the fingertips (right reference for
            # servoing; the hand body origin sits ~10 cm above it). Site
            # name differs across simulator generations: robosuite 1.4
            # (LIBERO fork) vs 1.5 (capbench/robocasa), which prefixes
            # gripper parts per arm.
            grip = None
            for site in ("gripper0_grip_site", "gripper0_right_grip_site"):
                try:
                    grip = [float(v) for v in self._engine.site_pos(site)]
                    break
                except KeyError:  # try the next generation
                    continue
            pos, _, _ = self._engine.body_pose(self._hand_body)
            return {"pos": [float(v) for v in pos], "grip": grip}

        return self._sim.submit(job)

    def handlers(self) -> dict:
        """The truth-verb table, ready for the control channel."""
        return {
            "reset": self.reset,
            "success": self.success,
            "task_info": self.task_info,
            "steps": self.steps,
            "objects": self.objects,
            "hand": self.hand,
        }
