"""Monitor unit contract (pure python; the measurement protocol itself).

The scored number is the LATCH (official any-step semantics); the step
counter is episode-scoped; success_at stamps exist exactly when latched.
These are the semantics every post-hoc decision leans on, so they get unit
guards independent of any simulator.
"""

from __future__ import annotations

from types import SimpleNamespace

from openrua.robot.sim.bridge.environments.worker import Worker
from openrua.robot.sim.bridge.rpc import Monitor


class _Env:
    def __init__(self):
        self.stepped = 0

    def step(self, action, **kwargs):
        self.stepped += 1
        return "obs"


class _Loader:
    """success() follows a scripted per-check sequence; reset pumps
    settling steps through env.step like the cap-x simulators do."""

    def __init__(self, script):
        self.script = list(script)
        self.resets = 0

    def success(self, env):
        return self.script.pop(0) if self.script else False

    def init_state(self, ctx, i):
        return i

    def reset(self, env, ctx, state):
        self.resets += 1
        env.step(None)  # simulator settling step during restore

    def task_info(self, env, ctx):
        return {"task_language": "probe"}


def _monitor(script):
    sim = Worker()
    sim.bind_current_thread()
    env = _Env()
    engine = SimpleNamespace(time=lambda: 1.5)
    return env, Monitor(env, {}, _Loader(script), engine, sim)


def test_latch_survives_state_degradation():
    env, mon = _monitor([False, True, False])
    for _ in range(3):
        env.step(None)
    v = mon.success({})
    assert v["success"] is True  # latched at step 2 ...
    assert v["success_end_state"] is False  # ... though the state degraded
    assert v["success_at"]["step"] == 2
    assert v["success_at"]["sim_time_s"] == 1.5


def test_no_latch_means_no_stamps():
    env, mon = _monitor([False, False])
    env.step(None)
    v = mon.success({})
    assert v["success"] is False and "success_at" not in v


def test_reset_scopes_the_counter_and_clears_the_latch():
    env, mon = _monitor([True])
    env.step(None)  # latch
    assert mon.steps({})["success_latched"] is True
    mon.reset({"init_state_id": 0})
    # settling steps pumped DURING restore must not leak into the count
    assert mon.steps({}) == {"steps": 0, "success_latched": False}


def test_step_wrap_returns_inner_result_and_counts():
    env, mon = _monitor([False, False, False])
    assert env.step(None) == "obs"
    env.step(None)
    assert mon.steps({})["steps"] == 2 and env.stepped == 2
