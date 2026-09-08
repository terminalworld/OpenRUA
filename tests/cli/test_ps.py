"""openrua ps: the claims under a runs root, as a table or JSON."""
import json
import subprocess
import sys

from openrua.runner import lock


def _ps(*argv):
    return subprocess.run([sys.executable, "-m", "openrua", "ps", *argv],
                          capture_output=True, text=True)


def test_ps_prints_claims_and_their_state(tmp_path):
    trial = tmp_path / "bench" / "run" / "trials" / "suite-0" / "seed3"
    trial.mkdir(parents=True)
    held = lock.acquire(trial, "rc-run-0-3-abc123")
    r = _ps("--runs-root", str(tmp_path), "--json")
    assert r.returncode == 0, r.stderr
    rows = json.loads(r.stdout)
    assert rows[0]["trial"] == "bench/run/trials/suite-0/seed3"
    assert rows[0]["live"] is True and rows[0]["stem"] == "rc-run-0-3-abc123"
    r = _ps("--runs-root", str(tmp_path))
    assert "bench/run/trials/suite-0/seed3" in r.stdout and "live" in r.stdout
    held.release()
    r = _ps("--runs-root", str(tmp_path))
    assert r.returncode == 0 and "no live attempts under" in r.stdout


def test_ps_hides_stale_claims_unless_asked(tmp_path):
    import json as _json, os
    trial = tmp_path / "bench" / "run" / "trials" / "suite-0" / "seed0"
    trial.mkdir(parents=True)
    (trial / lock.LOCK_NAME).write_text(_json.dumps(
        {"pid": 2 ** 22 + 7, "host": os.uname().nodename, "stem": "rc-gone-000000", "since": 0}))
    assert json.loads(_ps("--runs-root", str(tmp_path), "--json").stdout) == []
    rows = json.loads(_ps("--runs-root", str(tmp_path), "--json", "--all").stdout)
    assert len(rows) == 1 and rows[0]["live"] is False
