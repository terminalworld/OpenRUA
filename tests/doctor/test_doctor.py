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
    monkeypatch.setattr(doctor.checks, "engine_error", lambda: "")
    import hashlib
    a = agents.get("claude-code")
    want_wl = hashlib.sha256("\n".join(a.whitelist).encode()).hexdigest()
    want_pi = hashlib.sha256(a.install.encode()).hexdigest()
    monkeypatch.setattr(doctor.checks, "docker_inspect", _fake_docker({
        "openrua-proxy": {"openrua.whitelist_sha256": want_wl},
        "openrua-sim-libero_pro": {},
        "openrua-sandbox-jazzy": {"openrua.preinstall_sha256": want_pi}}))
    monkeypatch.setattr(doctor.checks.shutil, "which", lambda _: "/usr/bin/docker")
    creds = paths.credentials_dir(tmp_path) / a.name
    creds.mkdir(parents=True)
    (creds / a.credentials.filename).write_text("{}")
    r = doctor.run(robot="panda", home=tmp_path, bench="libero_pro")
    assert r.ok, r.render()
    assert r.summary["error"] == 0 and r.summary["warning"] == 0
    ids = [c.id for c in r.checks]
    assert ids[:2] == ["docker", "home"]
    assert {"proxy-image", "robot-image", "sandbox-image", "login-claude-code"} <= set(ids)


def test_rootless_podman_wants_keep_id_in_the_defaults_file(tmp_path, monkeypatch):
    monkeypatch.setattr(doctor.checks, "engine_error", lambda: "")
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
    monkeypatch.setattr(doctor.checks, "engine_error", lambda: "")
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
    assert by["robot-image"].severity == "error" and by["robot-image"].hint == "openrua build --bench libero_pro"
    assert by["proxy-image"].severity == "warning" and "build proxy --agent" in by["proxy-image"].hint
    assert by["sandbox-image"].severity == "warning" and "build sandbox --distro jazzy" in by["sandbox-image"].hint
    assert by["login-claude-code"].severity == "error"
    assert "claude login" in by["login-claude-code"].hint
    assert "setup-token" in by["login-claude-code"].hint        # the token route too
    assert not r.ok


def test_an_unwritable_user_directory_is_an_error(tmp_path, monkeypatch):
    monkeypatch.setattr(doctor.checks, "docker_inspect", _fake_docker({"openrua-proxy": {}}))
    monkeypatch.setattr(doctor.checks.shutil, "which", lambda _: "/usr/bin/docker")
    r = doctor.run(home=tmp_path)
    by = {c.id: c for c in r.checks}
    assert by["home"].severity == "ok"
    assert by["proxy-image"].detail.startswith("unlabelled")
    assert not [c for c in r.checks if c.id.startswith("agents-")]   # bundled agents load


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


def test_standing_proxy_checked_separately_from_rebuilt_image(tmp_path, monkeypatch):
    from openrua.doctor import checks
    a = agents.get("codex")
    current = agents.fact_sha256(a, "whitelist")
    def inspect(kind, name, fmt):
        if "Labels" not in fmt:
            return "container-id" if kind == "container" else "new-image-id"
        if 'openrua.agent.codex.' in fmt:
            return current if kind == "image" else "old-policy"
        return "<no value>"
    monkeypatch.setattr(checks, "docker_inspect", inspect)
    ctx = checks.Context(home=tmp_path, agents=[a], cfg=None, robot=None)
    assert checks.check_proxy_image(ctx)[0].severity == "ok"
    finding = checks.check_proxy_container(ctx)[0]
    assert finding.severity == "warning" and "end sessions" in finding.hint
    report = doctor.Report([finding])
    assert report.ok
    assert finding.hint in report.render()


def test_absent_agent_label_uses_legacy_aggregate_hash(tmp_path, monkeypatch):
    from openrua.doctor import checks
    a = agents.get("codex")
    import hashlib
    aggregate = hashlib.sha256("\n".join(a.whitelist).encode()).hexdigest()
    monkeypatch.setattr(checks, "docker_inspect", lambda kind, name, fmt:
                        aggregate if 'openrua.whitelist_sha256' in fmt else "<no value>")
    ctx = checks.Context(home=tmp_path, agents=[a], cfg=None, robot=None)
    assert checks._agents_in_artifact("legacy", ctx, "whitelist") == ("ok", "")


def test_robot_image_is_checked_against_the_declaration_it_was_rendered_from(tmp_path, monkeypatch):
    from openrua import config
    from openrua.doctor import checks
    from openrua.robot.sim import build as sim_build, install as installer
    inst = config.install_for("robosuite", "libero_pro")
    dockerfile, files = installer.render(inst, "libero_pro", paths.code_root())
    want = installer.fingerprint(dockerfile, files)
    cfg = {"machine": {"backend": {"kind": "sim", "ros_distro": "jazzy",
                                   "image": "openrua-sim-libero_pro",
                                   "sandbox_image": "openrua-sandbox-jazzy"}}}
    ctx = checks.Context(home=tmp_path, agents=[], cfg=cfg, robot="panda", install=inst,
                         bench="libero_pro", owner="libero_pro")
    monkeypatch.setattr(checks, "docker_inspect", _fake_docker({
        "openrua-sim-libero_pro": {sim_build.LABEL_FINGERPRINT: want,
                                   sim_build.LABEL_VERSION: "0.0.7"},
        "openrua-sandbox-jazzy": {}}))
    by = {c.id: c for c in checks.check_robot_images(ctx)}
    assert by["robot-image"].severity == "ok" and "0.0.7" in by["robot-image"].detail
    monkeypatch.setattr(checks, "docker_inspect", _fake_docker({
        "openrua-sim-libero_pro": {sim_build.LABEL_FINGERPRINT: "stale"},
        "openrua-sandbox-jazzy": {}}))
    by = {c.id: c for c in checks.check_robot_images(ctx)}
    assert by["robot-image"].severity == "warning"
    assert "declaration changed" in by["robot-image"].detail
    assert by["robot-image"].hint == "openrua build --bench libero_pro"


def test_each_selected_agent_uses_its_own_login_and_version(tmp_path):
    import yaml
    from openrua.doctor.checks import check_login
    locations = {name: tmp_path / name for name in ("claude-code", "codex")}
    for name, location in locations.items():
        location.mkdir()
        (location / agents.get(name).credentials.filename).write_text("{}")
    (tmp_path / "config.yaml").write_text(yaml.safe_dump({
        "agent": "claude-code",
        "agents": {name: {"credentials_dir": str(location), "version": version}
                   for (name, location), version in zip(locations.items(), ("2.1.251", "0.153.4"))},
    }))
    seen = []
    def inspect_context(ctx):
        seen.extend((a.name, a.version) for a in ctx.agents)
        return check_login(ctx)
    report = doctor.run(robot="panda", sim="robosuite", home=tmp_path,
                        agent_names=["claude-code", "codex"], checks=(inspect_context,))
    assert report.ok, report.render()
    assert seen == [("claude-code", "2.1.251"), ("codex", "0.153.4")]
    assert all(str(locations[row.id.removeprefix("login-")]) in row.label for row in report.checks)
    seen.clear()
    report = doctor.run(home=tmp_path, agent_names=["codex@0.159.1"], checks=(inspect_context,))
    assert report.ok
    assert seen == [("codex", "0.159.1")]


def test_doctor_without_robot_honors_selected_default_and_custom_manifest(tmp_path):
    import yaml
    from openrua.doctor.checks import check_login
    manifest = tmp_path / "custom.yaml"
    data = yaml.safe_load(paths.find("agents", "codex").read_text())
    data["name"] = "custom-agent"
    manifest.write_text(yaml.safe_dump(data))
    login = tmp_path / "custom-login"
    login.mkdir()
    (login / data["credentials"]["filename"]).write_text("{}")
    (tmp_path / "config.yaml").write_text(yaml.safe_dump({
        "agent": str(manifest),
        "agents": {str(manifest): {"credentials_dir": str(login)}},
    }))
    report = doctor.run(home=tmp_path, checks=(check_login,))
    assert report.ok, report.render()
    assert report.checks[0].id == "login-custom-agent"
    assert str(login) in report.checks[0].label


def test_invalid_user_config_is_a_finding_not_a_traceback(tmp_path):
    (tmp_path / "config.yaml").write_text("unknown_setting: true\n")
    report = doctor.run(home=tmp_path)
    assert not report.ok
    assert report.checks[0].id == "configuration"
    assert "unknown_setting" in report.checks[0].detail


def test_daemon_access_fails_loudly_even_if_cli_is_installed(tmp_path, monkeypatch):
    from openrua.doctor import checks
    from types import SimpleNamespace
    monkeypatch.setattr(checks.shutil, 'which', lambda _: '/usr/bin/docker')
    monkeypatch.setattr(checks, 'engine_version', lambda: 'Docker version 29')
    monkeypatch.setattr(checks.subprocess, 'run', lambda *a, **kw: SimpleNamespace(
        returncode=1, stderr='permission denied connecting to socket', stdout=''))
    report = checks.run(home=tmp_path, checks=(checks.check_docker,))
    assert not report.ok
    assert 'permission denied' in report.render()
    assert 'restore access' in report.render()


def test_login_requires_a_readable_nonempty_regular_file(tmp_path):
    from openrua.doctor.checks import Context, check_login
    agent = agents.get('codex')
    home = tmp_path / 'login'
    home.mkdir()
    ctx = Context(tmp_path, [agent], None, None, login_directories={agent: home})
    file = home / agent.credentials.filename
    for kind in ('missing', 'directory', 'empty', 'present'):
        if kind == 'directory': file.mkdir()
        if kind == 'empty':
            file.rmdir()
            file.touch()
        if kind == 'present': file.write_text('{}')
        result = check_login(ctx)[0]
        assert result.severity == ('ok' if kind == 'present' else 'error')
        if kind == 'present': assert 'authentication and quota' in result.detail
        else: assert result.hint
