"""run_args reach the container's docker run, verbatim, before the image."""
from pathlib import Path
from types import SimpleNamespace

from openrua.config import load_config
from openrua.sandbox import up as hup


def test_run_args_are_appended_to_docker_run(tmp_path, monkeypatch):
    cfg = load_config("libero_pro")
    calls = []

    def fake_run(argv, **kw):
        calls.append(argv)
        out = "alive" if argv[:2] == ["docker", "ps"] else ""
        return SimpleNamespace(returncode=0, stdout=out, stderr="")

    monkeypatch.setattr(hup.subprocess, "run", fake_run)
    monkeypatch.setattr(hup, "seed_workspace", lambda *a, **k: None, raising=False)
    hup.up(cfg, tmp_path / "ws", image=cfg["machine"]["backend"]["sandbox_image"],
           name="t", seed_workspace=False,
           run_args=("--userns=keep-id", "--pids-limit=0"))
    run = next(c for c in calls if c[:2] == ["docker", "run"])
    i = run.index("--userns=keep-id")
    assert run[i + 1] == "--pids-limit=0"
    assert run[i + 2] == "openrua-sandbox-jazzy" and run[i + 3] == "bash"   # then the image


def test_further_networks_are_joined_after_creation(tmp_path, monkeypatch):
    # A sandbox on the internal network reaches the internet through a
    # routed network it joins once it exists.
    cfg = load_config("libero_pro")
    calls = []

    def fake_run(argv, **kw):
        calls.append(argv)
        out = "alive" if argv[:2] == ["docker", "ps"] else ""
        return SimpleNamespace(returncode=0, stdout=out, stderr="")

    monkeypatch.setattr(hup.subprocess, "run", fake_run)
    hup.up(cfg, tmp_path / "ws", image=cfg["machine"]["backend"]["sandbox_image"],
           name="t", seed_workspace=False, network="openrua-internal",
           internet="open", networks=("openrua-internet",))
    run = next(i for i, c in enumerate(calls) if c[:2] == ["docker", "run"])
    join = calls.index(["docker", "network", "connect", "openrua-internet", "t"])
    assert join > run
    assert not any(a.startswith("HTTPS_PROXY") for a in calls[run])


def test_a_network_that_cannot_be_joined_is_loud(tmp_path, monkeypatch):
    import pytest
    from openrua.sandbox import SandboxError
    cfg = load_config("libero_pro")

    def fake_run(argv, **kw):
        if argv[:3] == ["docker", "network", "connect"]:
            return SimpleNamespace(returncode=1, stdout="", stderr="network openrua-internet not found")
        out = "alive" if argv[:2] == ["docker", "ps"] else ""
        return SimpleNamespace(returncode=0, stdout=out, stderr="")

    monkeypatch.setattr(hup.subprocess, "run", fake_run)
    with pytest.raises(SandboxError, match="network openrua-internet not found"):
        hup.up(cfg, tmp_path / "ws", image=cfg["machine"]["backend"]["sandbox_image"],
               name="t", seed_workspace=False, network="openrua-internal",
               internet="open", networks=("openrua-internet",))
