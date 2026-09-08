"""Proxy package tests: the one shared route to the internet.

Contract: build (whitelist -> image), up (IDEMPOTENT ensure -> url; the
one singleton-infra verb, unlike sandbox.up which always births anew),
down. Zero agent facts; zero layer imports.
"""

from __future__ import annotations

import ast
import subprocess
from pathlib import Path

from tests.conftest import requires_image

PKG = Path(__file__).resolve().parents[2] / "openrua" / "proxy"


def test_recipe_is_generic_and_agent_free():
    df = (PKG / "proxy.Dockerfile").read_text()
    assert 'ARG WHITELIST=""' in df  # optional; empty = deny all
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
def test_up_is_idempotent_ensure_and_down_removes():
    # Live singleton semantics on a throwaway name: two ensures return the
    # same URL AND the same container id (second call must not recreate).
    from openrua.proxy import down as pdown
    from openrua.proxy import up as pup

    name = "openrua-proxy-testsingleton"
    subprocess.run(["docker", "rm", "-f", name], capture_output=True)
    try:
        url1 = pup.ensure(network="openrua-internal", name=name)
        cid1 = subprocess.run(["docker", "ps", "-q", "--filter",
                               f"name=^{name}$"], capture_output=True,
                              text=True).stdout.strip()
        url2 = pup.ensure(network="openrua-internal", name=name)
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
        assert "openrua-internal" in nets and "bridge" in nets
    finally:
        assert pdown.down(name)
        assert not subprocess.run(["docker", "ps", "-q", "--filter",
                                   f"name=^{name}$"], capture_output=True,
                                  text=True).stdout.strip()


def test_runner_consumes_the_proxy_package():
    runner = (PKG.parent / "runner" / "trial.py").read_text()
    assert "openrua.proxy" in runner
    assert "def ensure_proxy" not in runner  # the split home is gone


def test_port_parameter_travels_build_to_up():
    # Full chain: --port at build -> label in image -> URL from up.
    from openrua.proxy import build as pbuild
    from openrua.proxy import down as pdown
    from openrua.proxy import up as pup

    tag, name = "openrua-proxy-porttest", "openrua-proxy-porttest-c"
    subprocess.run(["docker", "rm", "-f", name], capture_output=True)
    try:
        pbuild.build(tag=tag, port=9999)
        url = pup.ensure(network="openrua-internal", name=name, image=tag)
        assert url == f"http://{name}:9999"
        conf = subprocess.run(["docker", "exec", name, "cat",
                               "/etc/tinyproxy/tinyproxy.conf"],
                              capture_output=True, text=True).stdout
        assert "Port 9999" in conf  # the daemon really listens there
    finally:
        pdown.down(name)
        subprocess.run(["docker", "rmi", "-f", tag], capture_output=True)


def test_ensure_fails_loudly_on_missing_image():
    import pytest
    from openrua.proxy import up as pup
    name = "openrua-proxy-noimg-test"
    subprocess.run(["docker", "rm", "-f", name], capture_output=True)
    with pytest.raises(Exception) as e:
        pup.ensure(network="openrua-internal", name=name,
                   image="openrua-definitely-missing-proxy")
    assert "is the image built" in str(e.value)
    subprocess.run(["docker", "rm", "-f", name], capture_output=True)


@requires_image("openrua-proxy")
def test_ensure_revives_a_stopped_wall_same_container():
    from openrua.proxy import down as pdown
    from openrua.proxy import up as pup
    name = "openrua-proxy-revive-test"
    subprocess.run(["docker", "rm", "-f", name], capture_output=True)
    try:
        pup.ensure(network="openrua-internal", name=name)
        cid = subprocess.run(["docker", "ps", "-q", "--filter",
                              f"name=^{name}$"], capture_output=True,
                             text=True).stdout.strip()
        subprocess.run(["docker", "stop", name], capture_output=True)
        url = pup.ensure(network="openrua-internal", name=name)
        cid2 = subprocess.run(["docker", "ps", "-q", "--filter",
                               f"name=^{name}$"], capture_output=True,
                              text=True).stdout.strip()
        assert cid2 == cid and url.startswith(f"http://{name}:")
    finally:
        pdown.down(name)


@requires_image("openrua-proxy")
def test_ensure_reconnects_a_detached_wall():
    from openrua.proxy import down as pdown
    from openrua.proxy import up as pup
    name = "openrua-proxy-reconnect-test"
    subprocess.run(["docker", "rm", "-f", name], capture_output=True)
    try:
        pup.ensure(network="openrua-internal", name=name)
        subprocess.run(["docker", "network", "disconnect",
                        "openrua-internal", name], capture_output=True)
        pup.ensure(network="openrua-internal", name=name)
        nets = subprocess.run(
            ["docker", "inspect", "--format",
             "{{range $k, $_ := .NetworkSettings.Networks}}{{$k}} {{end}}",
             name], capture_output=True, text=True).stdout
        assert "openrua-internal" in nets
    finally:
        pdown.down(name)


def test_package_front_door():
    import subprocess, sys
    h = subprocess.run([sys.executable, "-m", "openrua.proxy", "--help"],
                       capture_output=True, text=True)
    assert h.returncode == 0 and "up" in h.stdout
