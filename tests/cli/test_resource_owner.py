"""A separate CLI stops resources through their actual owning process."""

import json
import subprocess
import sys
import time
from pathlib import Path

import pytest

from openrua.cli import build_parser, state
from openrua.cli.commands import clean, down
from openrua.errors import UnavailableError


@pytest.mark.parametrize("mode", ["up", "run"])
def test_down_reaches_native_resource_owner_and_retains_workspace(tmp_path, mode):
    bootstrap = tmp_path / "owner.py"
    bootstrap.write_text('''
import json, shlex, sys
from pathlib import Path
from types import SimpleNamespace
from openrua import robot
from openrua.cli import build_parser, state
from openrua.cli.commands import up, run
root = Path(sys.argv[1])
mode = sys.argv[2]
directory = root / 'sandboxes' / 'owned'
workspace = directory / 'workspace'
workspace.mkdir(parents=True)
(workspace / 'control.py').write_text('# keep this program')
launch = shlex.join([sys.executable, '-c', 'import time; time.sleep(120)'])
handle = robot.up({'kind': 'real', 'launch': 'exec ' + launch}, name='owned-sim',
                  config_path='', task_suite='', task_id=0, log_path=directory / 'driver.log')
(root / 'driver.json').write_text(json.dumps({'pid': handle.proc.pid}))
state.save('owned', root, status='running', backend='real', agent='fake',
           sandbox='unused', model='fake', proxy='', workspace=str(workspace))
def stop():
    handle.shutdown()
    assert handle.proc.poll() is not None
    (workspace / 'driver-stopped').write_text(str(handle.proc.returncode))
    state.stopped('owned', root)
up.open_session = lambda args: SimpleNamespace(power_off=stop,
    banner=lambda: 'resource owner ready', header=lambda *args: 'resource owner ready')
run.agents.get = lambda *args, **kwargs: SimpleNamespace(interactive_argv=lambda *a, **k: [
    sys.executable, '-c',
    'import time,sys; from pathlib import Path; p=Path(sys.argv[1]); '
    'exec("while not p.exists(): time.sleep(.02)")', str(workspace / 'driver-stopped')])
args = build_parser().parse_args(['--home', str(root), mode, 'fake', '--name', 'owned'])
try:
    raise SystemExit(args.fn(args))
finally:
    handle.shutdown()
''')
    output = tmp_path / "owner.log"
    with output.open("w") as log:
        owner = subprocess.Popen([sys.executable, str(bootstrap), str(tmp_path), mode], stdout=log, stderr=log)
    try:
        endpoint = tmp_path / "sandboxes/owned/control.json"
        deadline = time.monotonic() + 10
        while not endpoint.exists():
            assert owner.poll() is None, output.read_text()
            assert time.monotonic() < deadline, output.read_text()
            time.sleep(.02)
        ended = subprocess.run([sys.executable, "-m", "openrua", "--home", str(tmp_path),
                                "down", "--name", "owned"], capture_output=True, text=True, timeout=10)
        assert ended.returncode == 0, ended.stderr + output.read_text()
        assert owner.wait(timeout=10) == 0, output.read_text()
        assert not endpoint.exists()
        workspace = endpoint.parent / "workspace"
        assert (workspace / "control.py").read_text() == "# keep this program"
        assert (workspace / "driver-stopped").exists()
        assert state.load("owned", tmp_path, require_running=False)["status"] == "stopped"
        pid = json.loads((tmp_path / "driver.json").read_text())["pid"]
        assert not Path(f"/proc/{pid}").exists()
    finally:
        if owner.poll() is None:
            owner.terminate()
            try:
                owner.wait(timeout=5)
            except subprocess.TimeoutExpired:
                owner.kill()
                owner.wait()


def test_real_session_without_owner_cannot_claim_shutdown_or_delete(tmp_path, monkeypatch):
    directory = tmp_path / "sandboxes/unknown"
    directory.mkdir(parents=True)
    state.save("unknown", tmp_path, status="running", backend="real")
    monkeypatch.setattr(down, "sandbox_down", lambda _: pytest.fail("owner must be resolved first"))
    monkeypatch.setattr(clean, "running", lambda _: pytest.fail("containers cannot prove a host driver stopped"))
    for argv in (["down", "--name", "unknown"], ["clean", "--all", "--name", "unknown"]):
        args = build_parser().parse_args(["--home", str(tmp_path), *argv])
        with pytest.raises(UnavailableError):
            args.fn(args)
    assert state.load("unknown", tmp_path)["status"] == "running"
    assert directory.exists()


def test_clean_preserves_a_native_owner_endpoint(tmp_path, monkeypatch):
    directory = tmp_path / "sandboxes/owned"
    directory.mkdir(parents=True)
    (directory / "control.json").write_text("{}")
    monkeypatch.setattr(clean, "running", lambda _: pytest.fail("do not bypass the owner"))
    args = build_parser().parse_args(["--home", str(tmp_path), "clean", "--all", "--name", "owned"])
    with pytest.raises(UnavailableError, match="owner endpoint"):
        clean.run(args)
    assert directory.exists()
