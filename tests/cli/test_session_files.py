"""Stopping preserves user work; deletion and replacement must be explicit."""

from types import SimpleNamespace

import pytest

from openrua.cli import build_parser, state
from openrua.cli.commands import clean, down, up
from openrua.config import paths
from openrua.errors import NotFound, UnavailableError, UsageError


def saved_session(home, name="saved", workspace=None):
    directory = paths.sandbox_dir(name, home)
    workspace = workspace or directory / "workspace" / "workspace"
    workspace.mkdir(parents=True)
    (workspace / "control.py").write_text("# user's program\n")
    (directory / "profile").mkdir(parents=True, exist_ok=True)
    (directory / "profile" / "conversation.json").write_text('{"message": "keep me"}')
    state.save(name, home, workspace=str(workspace), backend="sim", status="running")
    return directory, workspace


def test_down_keeps_workspace_and_native_records_and_rejects_attach(monkeypatch, tmp_path):
    directory, workspace = saved_session(tmp_path)
    calls = []
    monkeypatch.setattr(down, "sandbox_down", lambda name: calls.append(name))
    monkeypatch.setattr(down.robot, "down", lambda name, kind: calls.append((name, kind)))
    args = build_parser().parse_args(["--home", str(tmp_path), "down", "--name", "saved"])
    assert down.run(args) == 0
    assert (workspace / "control.py").read_text() == "# user's program\n"
    assert (directory / "profile" / "conversation.json").exists()
    assert state.load("saved", tmp_path, require_running=False)["status"] == "stopped"
    with pytest.raises(NotFound, match="stopped"):
        state.load("saved", tmp_path)
    assert calls == ["saved-sandbox", ("saved-sim", "sim")]
    assert down.run(args) == 0  # repeat shutdown must also retain the files


def test_new_session_cannot_overwrite_a_retained_handle(tmp_path):
    directory, workspace = saved_session(tmp_path)
    state.stopped("saved", tmp_path)
    with pytest.raises(UsageError, match="already exists"):
        state.reserve("saved", tmp_path)
    assert (workspace / "control.py").exists()
    assert (directory / "profile" / "conversation.json").exists()


def test_handle_claim_is_exclusive(tmp_path):
    state.reserve("new", tmp_path)
    with pytest.raises(UsageError):
        state.reserve("new", tmp_path)


@pytest.mark.parametrize("name", ["..", "../other", "/tmp/other", "x/y", ""])
def test_session_names_cannot_escape_the_session_root(tmp_path, name):
    with pytest.raises(UsageError):
        paths.sandbox_dir(name, tmp_path)


def test_up_preserves_nonempty_external_directory_before_launch(monkeypatch, tmp_path):
    external = tmp_path / "external"
    external.mkdir()
    program = external / "my-program.py"
    program.write_text("keep")
    profile = tmp_path / "robot.yaml"
    profile.write_text("type: panda\nmachine:\n  backend: {kind: real, discovery: {network: host}}\n")
    monkeypatch.setattr(up, "ensure_internal_network", lambda: pytest.fail("must not start"))
    args = build_parser().parse_args(["--home", str(tmp_path / "home"), "up", str(profile),
                                      "--workspace", str(external)])
    with pytest.raises(UsageError, match="not empty"):
        up.open_session(args)
    assert program.read_text() == "keep"


def test_clean_name_deletes_only_selected_session_and_keeps_external_workspace(monkeypatch, tmp_path):
    home = tmp_path / "home"
    selected, external = saved_session(home, workspace=tmp_path / "outside")
    other, _ = saved_session(home, "other")
    monkeypatch.setattr(clean, "running", lambda _: False)
    args = build_parser().parse_args(["--home", str(home), "clean", "--name", "saved"])
    assert clean.run(args) == 0
    assert not selected.exists() and other.exists()
    assert (external / "control.py").exists()


def test_clean_keeps_running_session_without_all(monkeypatch, tmp_path):
    directory, _ = saved_session(tmp_path)
    monkeypatch.setattr(clean, "running", lambda _: True)
    args = build_parser().parse_args(["--home", str(tmp_path), "clean", "--name", "saved"])
    assert clean.run(args) == 0 and directory.exists()


def test_clean_does_not_delete_when_docker_is_unreachable(monkeypatch, tmp_path):
    directory, _ = saved_session(tmp_path)
    monkeypatch.setattr(clean.subprocess, "run", lambda *a, **kw:
                        SimpleNamespace(returncode=1, stderr="daemon unavailable"))
    args = build_parser().parse_args(["--home", str(tmp_path), "clean"])
    with pytest.raises(UnavailableError, match="daemon unavailable"):
        clean.run(args)
    assert directory.exists()


def test_clean_all_checks_shutdown_before_deleting(monkeypatch, tmp_path):
    directory, _ = saved_session(tmp_path)
    monkeypatch.setattr(clean, "running", lambda _: True)
    monkeypatch.setattr(clean, "sandbox_down", lambda _: False)
    monkeypatch.setattr(clean.robot, "down", lambda *a: None)
    args = build_parser().parse_args(["--home", str(tmp_path), "clean", "--all"])
    with pytest.raises(UnavailableError, match="still has running"):
        clean.run(args)
    assert directory.exists()


def test_clean_does_not_follow_session_symlink(monkeypatch, tmp_path):
    external = tmp_path / "external"
    external.mkdir()
    root = paths.sandboxes_dir(tmp_path)
    root.mkdir()
    (root / "link").symlink_to(external, target_is_directory=True)
    monkeypatch.setattr(clean, "running", lambda _: pytest.fail("must reject link first"))
    args = build_parser().parse_args(["--home", str(tmp_path), "clean", "--name", "link"])
    with pytest.raises(UsageError, match="symlink"):
        clean.run(args)
    assert external.exists()


@pytest.mark.parametrize("module", ["openrua.sandbox.down", "openrua.robot.sim.down"])
def test_shutdown_reports_daemon_errors_but_tolerates_missing_container(monkeypatch, module):
    import importlib
    shutdown = importlib.import_module(module)
    monkeypatch.setattr(shutdown.subprocess, "run", lambda *a, **kw:
                        SimpleNamespace(returncode=1, stderr="daemon unavailable"))
    with pytest.raises(RuntimeError, match="daemon unavailable"):
        shutdown.down("saved-sandbox")
    monkeypatch.setattr(shutdown.subprocess, "run", lambda *a, **kw:
                        SimpleNamespace(returncode=1, stderr="Error: No such container: saved-sandbox"))
    shutdown.down("saved-sandbox")


def test_failed_shutdown_keeps_running_status_and_files(monkeypatch, tmp_path):
    directory, _ = saved_session(tmp_path)
    def fail(name):
        raise RuntimeError("could not stop")
    monkeypatch.setattr(down, "sandbox_down", fail)
    args = build_parser().parse_args(["--home", str(tmp_path), "down", "--name", "saved"])
    with pytest.raises(RuntimeError, match="could not stop"):
        down.run(args)
    assert state.load("saved", tmp_path)["status"] == "running"
    assert directory.exists()
