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
