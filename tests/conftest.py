"""Shared test helpers.

Some tests exercise live containers and need an image built on this
machine (``openrua build sandbox`` / ``openrua build proxy``). They skip
where the image is absent, so the suite stays green on a checkout that
has never built anything (CI included) and still runs the live checks
wherever the images exist.
"""

from __future__ import annotations

import functools
import subprocess

import pytest


@functools.lru_cache(maxsize=None)
def image_present(name: str) -> bool:
    try:
        r = subprocess.run(["docker", "image", "inspect", name],
                           capture_output=True, timeout=30)
    except (OSError, subprocess.TimeoutExpired):
        return False
    return r.returncode == 0


def requires_image(name: str):
    """Skip the test unless the docker image ``name`` is built."""
    return pytest.mark.skipif(not image_present(name),
                              reason=f"docker image {name} not built here")


@pytest.fixture
def docker_daemon():
    """Live tests require a reachable daemon before creating their resources."""
    try:
        result = subprocess.run(['docker', 'info'], capture_output=True, timeout=15)
    except (OSError, subprocess.TimeoutExpired):
        pytest.skip('Docker daemon is unavailable')
    if result.returncode:
        pytest.skip('Docker daemon is unavailable')


@pytest.fixture
def proxy_address(monkeypatch):
    """A real robot's sandbox sits on the host network and reaches the
    proxy by address, which OpenRUA reads from the running proxy
    container. Tests that start a real robot with the bring-up faked
    stand in for that container, so they pass where none runs (CI)."""
    from openrua.runner import bringup
    monkeypatch.setattr(bringup, "proxy_url_from_network",
                        lambda name=None, *a, **k: "http://172.17.0.2:8888")
    return "http://172.17.0.2:8888"


@pytest.fixture
def agent_cli_ok(monkeypatch):
    """An interactive session checks the agent CLI inside its sandbox
    before starting the agent. Tests that fake the sandbox have no CLI to
    ask; this stands in for one that passes."""
    from openrua.runner import live
    monkeypatch.setattr(live, "check_agent_cli", lambda *a, **k: None)


@pytest.fixture(autouse=True)
def isolated_default_user_directory(tmp_path, monkeypatch):
    """Tests never load the developer's personal configuration or credentials."""
    from openrua.config import paths
    monkeypatch.setattr(paths, 'DEFAULT_HOME', str(tmp_path / 'default-user'))


def pytest_addoption(parser):
    parser.addoption('--native-agent-image', default='openrua-sandbox-jazzy',
                     help='image carrying native CLIs for offline discovery checks')
