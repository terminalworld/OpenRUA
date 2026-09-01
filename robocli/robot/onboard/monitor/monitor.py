"""Monitor: watches every step of the world and answers the host's line.

One Monitor per env. Two duties: (1) after every sim step it asks the
benchmark's original predicate once and remembers the first success
(which step, what time); (2) it answers the host's stdin questions
(reset / success / task_info / steps and the forensic reads
objects / hand). All benchmark knowledge arrives through the loader
parameter (the environment package's plug); this file holds only the
measurement protocol, which is exactly where the audit history lives.

Official eval semantics latch success per step (any-step success counts
even if the state later degrades; RoboCasa's runner does
``successes |= check``, LIBERO/robosuite eval loops end on first
success). Every step checks and latches; the end-state verdict is
reported alongside (2026-08-12, Zhaoyang: latch = the scored number,
end-state kept for ourselves).

``on_reset`` is a plain callable slot: whoever composes the machine may
need to act right after the world is restored (the ROS graph re-aligns
itself); this module only fires the slot, inside the same sim-thread
job, and knows nothing about who filled it.
"""

from __future__ import annotations

import os
import time

import numpy as np


class Monitor:
    def __init__(self, env, task_ctx: dict, loader, sim,
                 on_reset=None, environ=None):
        self._env = env
        self._ctx = task_ctx
        self._loader = loader  # the environment package's plug (parameter)
        self._sim = sim  # SimJobRunner: all env access goes through it
        self.on_reset = on_reset or (lambda: None)

        self._latched = False
        self._at_step = None
        self._at_sim_time = None
        self._at_wall = None
        self._steps = 0

        # Offline demo-replay recording (demo/): when ROBOCLI_RECORD_DIR
        # is set, every sim step renders the requested cameras to JPEGs.
        # Never set by experiment configs; evaluation runs take this
        # branch zero times (env var absent).
        env_vars = os.environ if environ is None else environ
        self._rec_dir = env_vars.get("ROBOCLI_RECORD_DIR")
        self._rec_cams = []
        if self._rec_dir:
            os.makedirs(self._rec_dir, exist_ok=True)
            for spec in env_vars.get(
                    "ROBOCLI_RECORD_CAMERAS", "agentview:640x480").split(","):
                cam, _, wh = spec.strip().partition(":")
                w, h = (wh or "640x480").split("x")
                self._rec_cams.append((cam, int(w), int(h)))

        self._inner_step = env.step
        env.step = self._latching_step

    # ------------------------------------------------------------- stepping
    def _record_frame(self) -> None:
        from PIL import Image
        raw = self._env.env if hasattr(self._env, "env") else self._env
        for cam, w, h in self._rec_cams:
            px = raw.sim.render(width=w, height=h, camera_name=cam)[::-1]
            Image.fromarray(px).save(
                f"{self._rec_dir}/{self._steps:06d}_{cam}.jpg", quality=85)

    def _latching_step(self, action, **kwargs):  # kwargs: cap-x fork's
        r = self._inner_step(action, **kwargs)  # skip_render_images etc.
        self._steps += 1
        if self._rec_dir:
            self._record_frame()
        if not self._latched and self._loader.success(self._env):
            self._latched = True
            # Time-to-success telemetry: with closed-book budgets (the
            # agent knows none of them), a generous run re-judged under
            # any tighter budget is behaviorally EXACT; these stamps
            # are what post-hoc judging looks up (2026-08-12, Zhaoyang).
            self._at_step = self._steps
            self._at_sim_time = float(self._env.sim.data.time)
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
            # Episode-scoped step counter (audit 2026-08-14 F7/F10):
            # substrate resets may pump settling steps through env.step
            # (cap-x nut runs ~100); zeroing HERE makes success_at.step
            # mean "steps since reset" uniformly across benchmarks.
            self._steps = 0
            if self._rec_dir:
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
        # indexing: channel.py reports handler exceptions instead of
        # dying, so a KeyError here silently hands the agent an EMPTY
        # MISSION (2026-08-11 capbench canary).
        return self._sim.submit(
            lambda: self._loader.task_info(self._env, self._ctx))

    def steps(self, _req: dict) -> dict:  # demo-replay sync anchor
        return {"steps": self._steps, "success_latched": bool(self._latched)}

    # ------------------------------------------------------ debug truth reads
    # Truth belongs to the evaluator; used for anomaly forensics; never
    # by the agent (the channel these ride is physically unreachable
    # from the sandbox).
    def objects(self, _req: dict) -> dict:
        def job():
            raw = self._env.env if hasattr(self._env, "env") else self._env
            obs = (raw._get_observations()
                   if hasattr(raw, "_get_observations") else {})
            return {
                k: [float(v) for v in np.asarray(val).flatten()]
                for k, val in obs.items()
                if k.endswith("_pos") or k.endswith("_quat")
            }

        return {"objects": self._sim.submit(job)}

    def hand(self, _req: dict) -> dict:
        def job():
            sim_ = self._env.sim
            # Grasp point between the fingertips (right reference for
            # servoing; the hand body origin sits ~10 cm above it). Site
            # name differs across substrate generations: robosuite 1.4
            # (LIBERO fork) vs 1.5 (capbench/robocasa), which prefixes
            # gripper parts per arm.
            grip = None
            for site in ("gripper0_grip_site", "gripper0_right_grip_site"):
                try:
                    sid = sim_.model.site_name2id(site)
                    grip = [float(v) for v in sim_.data.site_xpos[sid]]
                    break
                except Exception:  # noqa: BLE001; try the next generation
                    continue
            bid = sim_.model.body_name2id("robot0_right_hand")
            return {
                "pos": [float(v) for v in sim_.data.body_xpos[bid]],
                "grip": grip,
            }

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
