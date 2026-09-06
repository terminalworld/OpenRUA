"""The kitchen draw stays RoboCasa's own; we only seed it.

Their kitchen env picks (layout, style) with rng.choice at every reset,
from a list it has already filtered down to the layouts and styles this
task can legally run in. Our loader reproduces their gym wrapper's
reset(seed=...) and nothing else: seed the env rng, call reset.

What must not drift: no scene is ever pinned (on any split), the rng is
seeded from the seed alone, and the seed fully determines the episode.
"""

import numpy as np
import pytest

from robocli.robot.onboard.environment.robocasa import RoboCasaLoader

LOADER = RoboCasaLoader()
TARGET = {"split": "target"}


class FakeEnv:
    """Records what the loader does to an env, without building one."""

    def __init__(self):
        self.rng = None
        self.meta = None
        self.resets = 0

    def set_ep_meta(self, meta):
        self.meta = meta

    def unset_ep_meta(self):
        self.meta = None

    def reset(self):
        self.resets += 1


def test_the_seed_is_the_whole_state():
    assert LOADER.init_state(TARGET, 7) == {"seed": 7}


@pytest.mark.parametrize("split", ["target", "pretrain", "all"])
def test_no_split_has_its_scene_pinned(split):
    # Pinning a (layout, style) would both change the sampling their
    # numbers come from and risk forcing a kitchen this task excludes
    # (2026-09-04: that killed every composite task).
    env = FakeEnv()
    ctx = {"split": split}
    LOADER.reset(env, ctx, LOADER.init_state(ctx, 3))
    assert env.meta is None
    assert env.resets == 1


def test_reset_seeds_the_rng_from_the_seed():
    env = FakeEnv()
    LOADER.reset(env, TARGET, LOADER.init_state(TARGET, 13))
    assert env.rng.random() == np.random.default_rng(13).random()


def test_different_seeds_give_different_draws():
    draws = []
    for seed in range(10):
        env = FakeEnv()
        LOADER.reset(env, TARGET, LOADER.init_state(TARGET, seed))
        draws.append(env.rng.random())
    assert len(set(draws)) == 10
