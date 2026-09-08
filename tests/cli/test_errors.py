"""errors: one family, a fix on each, sysexits codes at the entry point."""
import subprocess
import sys

from openrua import errors


def test_family_and_exit_codes():
    e = errors.ConfigError("bad", hint="fix it")
    assert str(e) == "bad" and e.hint == "fix it" and e.exit_code == 78
    assert isinstance(e, ValueError)                       # old handlers still catch it
    assert isinstance(errors.NotFound("x"), FileNotFoundError)
    assert errors.NotFound("x").exit_code == 66
    assert errors.UnavailableError("x").exit_code == 69
    assert errors.AuthError("x").exit_code == 77
    assert errors.UsageError("x").exit_code == 2
    from openrua.sandbox import SandboxError
    from openrua.proxy import ProxyError
    assert issubclass(SandboxError, errors.UnavailableError)
    assert issubclass(ProxyError, errors.UnavailableError)


def test_cli_prints_message_and_hint_and_exits_with_the_code(tmp_path):
    r = subprocess.run([sys.executable, "-m", "openrua", "--home", str(tmp_path),
                        "agent", "--name", "ghost"], capture_output=True, text=True)
    assert r.returncode == 66
    assert "error: no live robot named 'ghost'" in r.stderr
    assert "hint: openrua up <robot> --name ghost" in r.stderr
    r = subprocess.run([sys.executable, "-m", "openrua", "--home", str(tmp_path),
                        "doctor", "--json", "nope"], capture_output=True, text=True)
    assert r.returncode == 1 and '"robot-profile"' in r.stdout
