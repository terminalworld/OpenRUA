"""doctor: structured findings, one report rendered two ways, exit on errors only."""
import json
from pathlib import Path

from openrua import agents, doctor
from openrua.config import paths


def _fake_docker(present: dict[str, dict[str, str]]):
    """present: image name -> labels. Returns a docker_inspect stand-in."""
    def inspect(kind, name, fmt):
        if name not in present:
            return None
        if "Labels" in fmt:
            label = fmt.split('"')[1]
            return present[name].get(label, "")
        return "sha256:deadbeef"
    return inspect


def test_all_green_when_everything_is_in_place(tmp_path, monkeypatch):
    import hashlib
    a = agents.get("claude-code")
    want_wl = hashlib.sha256("\n".join(a.whitelist).encode()).hexdigest()
    want_pi = hashlib.sha256(a.install.encode()).hexdigest()
    monkeypatch.setattr(doctor.checks, "docker_inspect", _fake_docker({
        "openrua-proxy": {"openrua.whitelist_sha256": want_wl},
        "openrua-sim-jazzy": {},
        "openrua-sandbox-jazzy": {"openrua.preinstall_sha256": want_pi}}))
    monkeypatch.setattr(doctor.checks.shutil, "which", lambda _: "/usr/bin/docker")
    (tmp_path / "simulators" / "cap-x" / ".venv-libero").mkdir(parents=True)
    creds = paths.credentials_dir(tmp_path) / a.name
    creds.mkdir(parents=True)
    (creds / a.credentials.filename).write_text("{}")
    r = doctor.run(robot="panda", home=tmp_path, bench="libero_pro")
    assert r.ok, r.render()
    assert r.summary["error"] == 0 and r.summary["warning"] == 0
    ids = [c.id for c in r.checks]
    assert ids[:2] == ["docker", "home"]
    assert {"proxy-image", "robot-image", "sandbox-image", "simulator", "login-claude-code"} <= set(ids)


def test_rootless_podman_wants_keep_id_in_the_defaults_file(tmp_path, monkeypatch):
    monkeypatch.setattr(doctor.checks.shutil, "which", lambda _: "/usr/bin/docker")
    monkeypatch.setattr(doctor.checks, "engine_version", lambda: "podman version 6.1.1")
    monkeypatch.setattr(doctor.checks, "engine_rootless", lambda: True)
    by = {c.id: c for c in doctor.run(home=tmp_path).checks}
    assert by["docker"].detail == "podman version 6.1.1"
    assert by["sandbox-userns"].severity == "error"
    assert "--userns=keep-id" in by["sandbox-userns"].hint
    assert str(paths.config_path(tmp_path)) in by["sandbox-userns"].hint
    (tmp_path / "config.yaml").write_text("sandbox:\n  run_args: ['--userns=keep-id']\n")
    by = {c.id: c for c in doctor.run(home=tmp_path).checks}
    assert by["sandbox-userns"].severity == "ok"


def test_docker_engine_reports_no_userns_row(tmp_path, monkeypatch):
    monkeypatch.setattr(doctor.checks.shutil, "which", lambda _: "/usr/bin/docker")
    monkeypatch.setattr(doctor.checks, "engine_version", lambda: "Docker version 27.1.1, build 6312585")
    monkeypatch.setattr(doctor.checks, "engine_rootless", lambda: False)
    ids = [c.id for c in doctor.run(home=tmp_path).checks]
    assert "docker" in ids and "sandbox-userns" not in ids


def test_missing_pieces_are_errors_with_a_fix_and_stale_labels_are_warnings(tmp_path, monkeypatch):
    monkeypatch.setattr(doctor.checks, "docker_inspect", _fake_docker({
        "openrua-proxy": {"openrua.whitelist_sha256": "stale"},
        "openrua-sandbox-jazzy": {"openrua.preinstall_sha256": "stale"}}))
    monkeypatch.setattr(doctor.checks.shutil, "which", lambda _: None)
    r = doctor.run(robot="panda", home=tmp_path, bench="libero_pro")
    by = {c.id: c for c in r.checks}
    assert by["docker"].severity == "error" and "docs.docker.com" in by["docker"].hint
    assert by["robot-image"].severity == "error" and by["robot-image"].hint == "openrua build robot --distro jazzy"
    assert by["proxy-image"].severity == "warning" and "build proxy --agent" in by["proxy-image"].hint
    assert by["sandbox-image"].severity == "warning" and "build sandbox --distro jazzy" in by["sandbox-image"].hint
    assert by["simulator"].severity == "error" and str(tmp_path / "simulators") in by["simulator"].hint
    assert by["login-claude-code"].severity == "error"
    assert "claude login" in by["login-claude-code"].hint
    assert "setup-token" in by["login-claude-code"].hint        # the token route too
    assert not r.ok


def test_user_directory_problems_are_reported_not_fatal(tmp_path, monkeypatch):
    monkeypatch.setattr(doctor.checks, "docker_inspect", _fake_docker({"openrua-proxy": {}}))
    monkeypatch.setattr(doctor.checks.shutil, "which", lambda _: "/usr/bin/docker")
    (tmp_path / "robots").mkdir()
    (tmp_path / "robots" / "panda.yaml").write_text("machine: {}\n")     # shadows a bundled name
    (tmp_path / "robots" / "bad.yaml").write_text("machine: [unclosed\n")    # not yaml
    (tmp_path / "agents").mkdir()
    (tmp_path / "agents" / "broken.yaml").write_text("name: broken\ndefault_model: m\nhooks: broken\n")
    (tmp_path / "plugins" / "agents").mkdir(parents=True)
    (tmp_path / "plugins" / "agents" / "broken.py").write_text("raise RuntimeError('nope')\n")
    r = doctor.run(home=tmp_path)
    by = {c.id: c for c in r.checks}
    assert by["robots-panda-shadowed"].severity == "warning"
    assert by["robots-bad-unreadable"].severity == "error"
    assert by["agents-broken-broken"].severity == "error" and "nope" in by["agents-broken-broken"].detail
    assert by["proxy-image"].detail.startswith("unlabelled")


def test_unknown_robot_or_agent_is_a_finding(tmp_path, monkeypatch):
    monkeypatch.setattr(doctor.checks, "docker_inspect", _fake_docker({}))
    r = doctor.run(robot="no-such-robot", agent_names=["no-such-agent"], home=tmp_path)
    by = {c.id: c for c in r.checks}
    assert by["robot-profile"].severity == "error"
    assert by["agent-no-such-agent"].severity == "error"


def test_a_crashing_check_becomes_a_finding(tmp_path):
    def boom(ctx):
        raise ZeroDivisionError("x")
    r = doctor.run(home=tmp_path, checks=(boom,))
    assert r.checks[0].id == "boom" and r.checks[0].severity == "error"
    assert "ZeroDivisionError" in r.checks[0].detail


def test_json_and_text_are_the_same_report(tmp_path, monkeypatch, capsys):
    monkeypatch.setattr(doctor.checks, "docker_inspect", _fake_docker({}))
    r = doctor.run(home=tmp_path)
    d = json.loads(r.to_json())
    assert set(d) == {"ok", "summary", "checks"}
    assert [c["id"] for c in d["checks"]] == [c.id for c in r.checks]
    text = r.render()
    assert text.startswith("openrua doctor") and "fix:" in text     # proxy image missing
    assert doctor.print_report(r, as_json=True) == 1
    assert json.loads(capsys.readouterr().out)["ok"] is False
