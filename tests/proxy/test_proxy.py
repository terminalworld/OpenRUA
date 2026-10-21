"""Proxy package tests: the route out for a sandbox limited to its model service.

Contract: build (-> image, no whitelist), up (whitelist -> IDEMPOTENT
ensure -> url; one standing proxy per whitelist, named for it, unlike
sandbox.up which always births anew), down. Zero agent facts; zero layer
imports.
"""

from __future__ import annotations

import ast
import subprocess
import uuid

import pytest
from pathlib import Path

from tests.conftest import requires_image

PKG = Path(__file__).resolve().parents[2] / "openrua" / "proxy"


@pytest.fixture
def network():
    """Every live proxy test owns its network, including on a clean CI runner."""
    name = "openrua-proxy-check-" + uuid.uuid4().hex[:12]
    subprocess.run(["docker", "network", "create", "--internal", name],
                   capture_output=True, text=True, check=True)
    try:
        yield name
    finally:
        subprocess.run(["docker", "network", "rm", name],
                       capture_output=True, text=True, check=True)


def test_recipe_is_generic_and_agent_free():
    df = (PKG / "proxy.Dockerfile").read_text()
    # The image holds no whitelist: it arrives at start and is written to
    # the filter file there (empty = deny all).
    assert "ARG WHITELIST" not in df
    assert 'ENV WHITELIST=""' in df and "/etc/tinyproxy/filter" in df
    for text in (df, (PKG / "tinyproxy.conf").read_text()):
        low = text.lower()
        assert "claude" not in low and "anthropic" not in low


def test_sandbox_package_no_longer_carries_the_wall():
    sandbox = PKG.parent / "sandbox"
    assert not (sandbox / "proxy").exists()
    for py in sandbox.glob("*.py"):
        low = py.read_text().lower()
        # no proxy-building parameters (prose mentions of the proxy are fine)
        assert "--whitelist" not in low and "proxy_tag" not in low, py.name


def test_proxy_imports_no_layer():
    for py in PKG.glob("*.py"):
        for node in ast.walk(ast.parse(py.read_text())):
            mods = [a.name for a in node.names] if isinstance(node, ast.Import) \
                else [node.module] if isinstance(node, ast.ImportFrom) and node.module else []
            for m in mods:
                assert not any(m.startswith(f"openrua.{layer}") for layer in
                               ("robot.sim.bridge", "runner", "sandbox", "agents")), \
                    f"{py.name} imports {m}"


@requires_image("openrua-proxy")
def test_up_is_idempotent_ensure_and_down_removes(network):
    # Live singleton semantics on a throwaway name: two ensures return the
    # same URL AND the same container id (second call must not recreate).
    from openrua.proxy import down as pdown
    from openrua.proxy import name_for
    from openrua.proxy import up as pup

    whitelist = "^" + network + r"\.invalid$"   # unique per test: its own proxy
    name = name_for(whitelist)
    subprocess.run(["docker", "rm", "-f", name], capture_output=True)
    try:
        url1 = pup.ensure(network=network, whitelist=whitelist)
        cid1 = subprocess.run(["docker", "ps", "-q", "--filter",
                               f"name=^{name}$"], capture_output=True,
                              text=True).stdout.strip()
        url2 = pup.ensure(network=network, whitelist=whitelist)
        cid2 = subprocess.run(["docker", "ps", "-q", "--filter",
                               f"name=^{name}$"], capture_output=True,
                              text=True).stdout.strip()
        assert url1 == url2 == f"http://{name}:8888"
        assert cid1 and cid1 == cid2  # ensured, not recreated
        # dual-homed: serves the internal net AND keeps its own internet route
        nets = subprocess.run(
            ["docker", "inspect", "--format",
             "{{range $k, $_ := .NetworkSettings.Networks}}{{$k}} {{end}}",
             name], capture_output=True, text=True).stdout
        assert network in nets and "bridge" in nets
        # the whitelist given at start is the one the daemon filters by
        filt = subprocess.run(["docker", "exec", name, "cat", "/etc/tinyproxy/filter"],
                              capture_output=True, text=True).stdout
        assert filt.strip() == whitelist
    finally:
        assert pdown.down(name)
        assert not subprocess.run(["docker", "ps", "-q", "--filter",
                                   f"name=^{name}$"], capture_output=True,
                                  text=True).stdout.strip()


def test_runner_consumes_the_proxy_package():
    runner = (PKG.parent / "runner" / "trial.py").read_text()
    assert "openrua.proxy" in runner
    assert "def ensure_proxy" not in runner  # the split home is gone


def test_port_parameter_travels_build_to_up(network):
    # Full chain: --port at build -> label in image -> URL from up.
    from openrua.proxy import build as pbuild
    from openrua.proxy import down as pdown
    from openrua.proxy import name_for
    from openrua.proxy import up as pup

    tag, whitelist = network + "-image", "^" + network + r"\.invalid$"
    name = name_for(whitelist)
    subprocess.run(["docker", "rm", "-f", name], capture_output=True)
    try:
        pbuild.build(tag=tag, port=9999)
        url = pup.ensure(network=network, whitelist=whitelist, image=tag)
        assert url == f"http://{name}:9999"
        conf = subprocess.run(["docker", "exec", name, "cat",
                               "/etc/tinyproxy/tinyproxy.conf"],
                              capture_output=True, text=True).stdout
        assert "Port 9999" in conf  # the daemon really listens there
    finally:
        pdown.down(name)
        subprocess.run(["docker", "rmi", "-f", tag], capture_output=True)


def test_ensure_fails_loudly_on_missing_image(network):
    import pytest
    from openrua.proxy import name_for
    from openrua.proxy import up as pup
    whitelist = "^" + network + r"\.invalid$"
    name = name_for(whitelist)
    subprocess.run(["docker", "rm", "-f", name], capture_output=True)
    with pytest.raises(Exception) as e:
        pup.ensure(network=network, whitelist=whitelist,
                   image="openrua-definitely-missing-proxy")
    assert "is the image built" in str(e.value)
    subprocess.run(["docker", "rm", "-f", name], capture_output=True)


@requires_image("openrua-proxy")
def test_ensure_revives_a_stopped_wall_same_container(network):
    from openrua.proxy import down as pdown
    from openrua.proxy import name_for
    from openrua.proxy import up as pup
    whitelist = "^" + network + r"\.invalid$"
    name = name_for(whitelist)
    subprocess.run(["docker", "rm", "-f", name], capture_output=True)
    try:
        pup.ensure(network=network, whitelist=whitelist)
        cid = subprocess.run(["docker", "ps", "-q", "--filter",
                              f"name=^{name}$"], capture_output=True,
                             text=True).stdout.strip()
        subprocess.run(["docker", "stop", name], capture_output=True)
        url = pup.ensure(network=network, whitelist=whitelist)
        cid2 = subprocess.run(["docker", "ps", "-q", "--filter",
                               f"name=^{name}$"], capture_output=True,
                              text=True).stdout.strip()
        assert cid2 == cid and url.startswith(f"http://{name}:")
    finally:
        pdown.down(name)


@requires_image("openrua-proxy")
def test_ensure_reconnects_a_detached_wall(network):
    from openrua.proxy import down as pdown
    from openrua.proxy import name_for
    from openrua.proxy import up as pup
    whitelist = "^" + network + r"\.invalid$"
    name = name_for(whitelist)
    subprocess.run(["docker", "rm", "-f", name], capture_output=True)
    try:
        pup.ensure(network=network, whitelist=whitelist)
        subprocess.run(["docker", "network", "disconnect",
                        network, name], capture_output=True)
        pup.ensure(network=network, whitelist=whitelist)
        nets = subprocess.run(
            ["docker", "inspect", "--format",
             "{{range $k, $_ := .NetworkSettings.Networks}}{{$k}} {{end}}",
             name], capture_output=True, text=True).stdout
        assert network in nets
    finally:
        pdown.down(name)


def test_package_front_door():
    import subprocess, sys
    h = subprocess.run([sys.executable, "-m", "openrua.proxy", "--help"],
                       capture_output=True, text=True)
    assert h.returncode == 0 and "up" in h.stdout


def test_failed_network_attachment_preserves_docker_error_and_standing_proxy(monkeypatch):
    import pytest
    from types import SimpleNamespace
    from openrua.proxy import ProxyError, name_for, up

    calls = []
    monkeypatch.setattr(up, "_running", lambda name: "existing")
    monkeypatch.setattr(up, "_inspect", lambda *args: '{"bridge": {}}')
    def run(argv, **kwargs):
        calls.append(argv)
        return SimpleNamespace(returncode=1, stderr="network target not found")
    monkeypatch.setattr(up.subprocess, "run", run)
    with pytest.raises(ProxyError, match="network target not found"):
        up.ensure("target", whitelist="^api\\.example\\.com$")
    assert calls == [["docker", "network", "connect", "target",
                      name_for("^api\\.example\\.com$")]]


def test_duplicate_network_attachment_is_only_accepted_with_membership(monkeypatch):
    from types import SimpleNamespace
    from openrua.proxy import up

    monkeypatch.setattr(up, "_running", lambda name: "existing")
    monkeypatch.setattr(up, "_inspect", lambda kind, name, fmt:
                        '{"target": {}}' if "Networks" in fmt else "same-image")
    monkeypatch.setattr(up, "_port", lambda name: "8888")
    monkeypatch.setattr(up.subprocess, "run", lambda *a, **kw:
                        SimpleNamespace(returncode=1, stderr="endpoint already exists"))
    from openrua.proxy import name_for
    assert up.ensure("target", whitelist="x") == f"http://{name_for('x')}:8888"


def test_container_start_failure_keeps_original_docker_error(monkeypatch):
    import pytest
    from types import SimpleNamespace
    from openrua.proxy import ProxyError, up

    monkeypatch.setattr(up, "_running", lambda name: "")
    monkeypatch.setattr(up, "_exists", lambda name: "")
    monkeypatch.setattr(up.time, "sleep", lambda seconds: None)
    monkeypatch.setattr(up.subprocess, "run", lambda *a, **kw:
                        SimpleNamespace(returncode=125, stderr="permission denied by daemon"))
    with pytest.raises(ProxyError, match="permission denied by daemon"):
        up.ensure("target")


def test_each_whitelist_has_its_own_standing_proxy():
    # Switching a rig from Claude Code to Codex once left a proxy that
    # allowed only Anthropic's hosts, and Codex met a 403 mid-session. A
    # proxy's name now carries its hosts, so other hosts get another proxy
    # and none is replaced under a running session.
    from openrua.proxy import NAME, name_for
    from openrua.proxy import up as proxy_up
    a, b = "^api\\.anthropic\\.com$", "^api\\.openai\\.com$"
    assert name_for(a) == name_for(a)
    assert name_for(a) != name_for(b)
    assert name_for(a).startswith(NAME + "-")
    assert not hasattr(proxy_up, "_serves")


def test_a_new_proxy_is_given_its_whitelist_at_start(monkeypatch):
    from types import SimpleNamespace
    from openrua.proxy import name_for, up

    whitelist = "^api\\.example\\.com$\n^login\\.example\\.com$"
    name = name_for(whitelist)
    runs, state = [], {"up": False}
    def run(argv, **kwargs):
        runs.append(argv)
        if argv[:3] == ["docker", "run", "-d"]:
            state["up"] = True
        return SimpleNamespace(returncode=0, stderr="")
    monkeypatch.setattr(up, "_running", lambda n: "id" if state["up"] else "")
    monkeypatch.setattr(up, "_exists", lambda n: "")
    monkeypatch.setattr(up, "_inspect", lambda kind, n, fmt:
                        '{"target": {}}' if "Networks" in fmt else "same-image")
    monkeypatch.setattr(up, "_port", lambda n: "8888")
    monkeypatch.setattr(up.subprocess, "run", run)
    assert up.ensure("target", whitelist=whitelist) == f"http://{name}:8888"
    started = next(argv for argv in runs if argv[:3] == ["docker", "run", "-d"])
    assert started[started.index("--name") + 1] == name
    assert started[started.index("-e") + 1] == f"WHITELIST={whitelist}"


def test_build_bakes_no_whitelist(monkeypatch):
    from types import SimpleNamespace
    from openrua.proxy import build as pbuild
    calls = []
    def run(argv, **kwargs):
        calls.append(argv)
        return SimpleNamespace(returncode=0, stdout="sha256:x", stderr="")
    monkeypatch.setattr(pbuild.subprocess, "run", run)
    assert pbuild.build(tag="t", port=8888) == ("t", "sha256:x")
    built = calls[0]
    assert "PORT=8888" in built and not any("WHITELIST" in a for a in built)
