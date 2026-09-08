"""Environment loaders: the plug shape, machine-enforced.

Mirror of the agents adapter guards: every registered loader implements
every name in LOADER_INTERFACE, the registry resolves all three
benchmarks' config names, and an unknown name fails loud with the known
list. No simulator boots here; simulator imports live inside create().
"""

from __future__ import annotations

import pytest

from openrua.robot.sim.bridge import environments as environment


def test_every_loader_implements_the_plug_shape():
    for loader in environment.LOADERS:
        for name in environment.LOADER_INTERFACE:
            assert hasattr(loader, name), (
                f"{type(loader).__name__} missing {name}")


def test_registry_resolves_every_benchmark_config_name():
    for benchmark in ("libero_pro", "libero", "capbench", "robocasa365"):
        assert benchmark in environment.get(benchmark).BENCHMARKS


def test_unknown_benchmark_fails_loud():
    with pytest.raises(ValueError, match="unknown benchmark.*capbench"):
        environment.get("no-such-benchmark")
