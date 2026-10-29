"""Record semantics guards: the commands.sh condensate and the script
operator's contract.

commands.sh carries the agent's full world-facing
action stream: Bash verbatim + Write/Edit as here-docs; reads excluded.
The guard here EXECUTES a synthesized condensate and asserts the files
materialize, so the here-doc encoding is proven by bash itself, not by
string inspection.
"""

from __future__ import annotations

import json
import subprocess
from pathlib import Path

import pytest

from openrua import agents
from openrua.runner.record import extract_commands


def _tool_use(oid, name, inp):
    return {"type": "assistant", "message": {"content": [
        {"type": "tool_use", "id": oid, "name": name, "input": inp}]}}


def _make_transcript(path: Path, target: Path) -> None:
    tricky = "line with 'quotes' and $VARS\nEOF\nOPENRUA_EOF almost\n"
    recs = [
        _tool_use("t1", "Bash", {"command": "echo probe-ran > probe.txt"}),
        _tool_use("t2", "Write", {"file_path": str(target),
                                  "content": tricky}),
        _tool_use("t3", "Edit", {"file_path": str(target),
                                 "old_string": "$VARS",
                                 "new_string": "$REPLACED"}),
        _tool_use("t4", "Read", {"file_path": str(target)}),  # not extracted
    ]
    path.write_text("\n".join(json.dumps(r) for r in recs))


def test_condensate_replays_bash_write_edit(tmp_path):
    transcript = tmp_path / "transcript.jsonl"
    target = tmp_path / "deep" / "made_by_write.txt"
    _make_transcript(transcript, target)

    out = tmp_path / "commands.sh"
    extract_commands(transcript, out, agents.get("claude-code"))
    body = out.read_text()
    assert "Read" not in body  # reads have zero world effect

    r = subprocess.run(["bash", "-n", str(out)], capture_output=True, text=True)
    assert r.returncode == 0, r.stderr

    r = subprocess.run(["bash", str(out)], cwd=tmp_path,
                       capture_output=True, text=True)
    assert r.returncode == 0, r.stderr
    assert (tmp_path / "probe.txt").read_text().strip() == "probe-ran"
    replayed = target.read_text()
    assert "$REPLACED" in replayed and "$VARS" not in replayed
    assert "OPENRUA_EOF almost" in replayed  # marker dodged the content


def test_script_operator_requires_a_script():
    from openrua.runner.operators import OPERATORS, script_operator

    assert set(OPERATORS) == {"none", "script", "agent"}
    with pytest.raises(ValueError, match="--script"):
        script_operator({"script": None})


# ----------------------------------------------- scrubbing and archiving

def test_secret_strings_collects_only_long_values(tmp_path):
    from openrua.runner.record import secret_strings
    (tmp_path / "creds.json").write_text(json.dumps({
        "accessToken": "tok-" + "a" * 30,
        "nested": {"refresh": ["tok-" + "b" * 30]},
        "short": "abc", "count": 5}))
    (tmp_path / "junk.json").write_text("not json at all")  # tolerated
    secrets = secret_strings(tmp_path)
    assert "tok-" + "a" * 30 in secrets and "tok-" + "b" * 30 in secrets
    assert "abc" not in secrets


def test_strict_credentials_allow_absent_profile_but_reject_broken_files(tmp_path):
    from openrua.runner.record import secret_strings, secret_strings_from_env_file

    assert secret_strings(tmp_path / 'not-created', strict=True) == []
    (tmp_path / 'auth.json').write_text('not json')
    with pytest.raises(json.JSONDecodeError):
        secret_strings(tmp_path, strict=True)
    assert secret_strings(tmp_path) == []
    missing = tmp_path / 'missing.env'
    with pytest.raises(FileNotFoundError):
        secret_strings_from_env_file(missing, strict=True)
    assert secret_strings_from_env_file(missing) == []


@pytest.mark.parametrize('operation', ['list', 'read'])
def test_strict_credentials_report_access_failure(monkeypatch, tmp_path, operation):
    from openrua.runner.record import secret_strings

    (tmp_path / 'auth.json').write_text('{}')

    def fail(*args, **kwargs):
        raise PermissionError('credential access unavailable')

    monkeypatch.setattr(Path, 'iterdir' if operation == 'list' else 'read_text', fail)
    with pytest.raises(PermissionError):
        secret_strings(tmp_path, strict=True)


def test_scrub_file_redacts_every_secret(tmp_path):
    from openrua.runner.record import scrub_file
    f = tmp_path / "transcript.jsonl"
    f.write_text("saw tok-SECRETSECRETSECRETS twice: tok-SECRETSECRETSECRETS")
    scrub_file(f, ["tok-SECRETSECRETSECRETS"])
    body = f.read_text()
    assert "SECRETSECRET" not in body and body.count("[REDACTED]") == 2


def test_finalize_scrubs_camera_diagnostics_without_rewriting_frames(tmp_path):
    from openrua.runner.record import finalize_trial

    frames = tmp_path / 'frames'
    frames.mkdir()
    secret = 'synthetic-camera-credential-value'
    (frames / 'camera.log').write_text(f'camera failure: {secret}\n')
    image = b'\xff\xd8\xff\x00camera-frame'
    (frames / 'view.jpg').write_bytes(image)
    finalize_trial(tmp_path, object(), [secret])
    assert (frames / 'camera.log').read_text() == 'camera failure: [REDACTED]\n'
    assert (frames / 'view.jpg').read_bytes() == image


def test_archive_prior_attempt_moves_evidence_down(tmp_path):
    from openrua.runner.record import archive_prior_attempt
    trial = tmp_path / "seed0"
    trial.mkdir()
    assert archive_prior_attempt(trial) is None  # nothing yet -> no-op
    (trial / "result.json").write_text("{}")
    (trial / "bridge.log").write_text("boot")
    dest = archive_prior_attempt(trial)
    assert dest == trial / "attempts" / "attempt-0001"
    assert (dest / "result.json").exists() and (dest / "bridge.log").exists()
    assert not (trial / "result.json").exists()  # top level = latest attempt
    (trial / "result.json").write_text("{}")
    assert archive_prior_attempt(trial) == trial / "attempts" / "attempt-0002"


def test_archive_prior_attempt_keeps_what_it_is_told(tmp_path):
    from openrua.runner.record import archive_prior_attempt
    trial = tmp_path / "seed0"
    trial.mkdir()
    (trial / "result.json").write_text("{}")
    (trial / ".claim").write_text("mine")
    dest = archive_prior_attempt(trial, keep=[trial / ".claim"])
    assert (trial / ".claim").read_text() == "mine"
    assert not (dest / ".claim").exists()
    assert (dest / "result.json").exists()


def test_claim_keeps_its_lock_while_archiving(tmp_path):
    from openrua.runner import lock
    from openrua.runner.trial import claim
    trial = tmp_path / "seed0"
    trial.mkdir()
    (trial / "result.json").write_text("{}")
    held = claim(trial, "rc-x")
    assert (trial / "attempts" / "attempt-0001" / "result.json").exists()
    assert lock.holder(trial)["stem"] == "rc-x"  # still claimed while running
    held.release()
    assert lock.holder(trial) is None


def test_provenance_pins_the_whole_chain(tmp_path):
    from openrua.runner.record import provenance

    class _Agent:
        name = "stub"
        version_argv = ("echo", "9.9.9")
        version = None

    class _Args:
        operator, task_suite = "none", "suite"
        task_ids, seeds = [0], [0]
        wall_clock_min, ros_domain = 30, 44

    code_root = Path(__file__).resolve().parents[2]
    cfg_path = tmp_path / "config.yaml"
    cfg_path.write_text("machine: {}\n")
    cfg = {"machine": {"backend": {
        "kind": "sim",
        "image": "openrua-definitely-missing",
        "simulator": {"engine": "x"},
        "gpus": False}}}
    prov = provenance(cfg_path, cfg, _Args(), _Agent(), "tplhash",
                      "prompt text", code_root, proxy_image="openrua-proxy")
    assert prov["openrua_version"]
    assert len(prov["openrua_commit"]) == 40
    # the simulator content is in the robot image: name recorded, digest probed
    assert prov["sim_image"] == "openrua-definitely-missing"
    assert prov["host"]["hostname"] and prov["host"]["cpu_count"] >= 1
    assert prov["ros_domain"] == 44 and prov["gpu_render"] is False
    assert prov["agent_cli"] == {"name": "stub", "version_pin": None}
    assert prov["prompt_sha256"] and prov["config_sha256"]
    # a missing image must record its probe result, never kill the trial
    assert isinstance(prov["sim_image_digest"], str)


def test_run_config_is_write_once(tmp_path):
    from openrua.runner.record import write_run_config
    write_run_config(tmp_path, {"gen": 1})
    write_run_config(tmp_path, {"gen": 2})  # concurrent runner: no rewrite
    assert json.loads((tmp_path / "config.json").read_text()) == {"gen": 1}


@pytest.mark.parametrize('failure', ['fsync', 'replace'])
@pytest.mark.parametrize('artifact', ['result', 'provenance'])
def test_failed_record_update_keeps_the_previous_complete_record(tmp_path, monkeypatch, failure, artifact):
    from openrua.runner import record

    before = {'success': None, 'verdict': 'pending', 'termination': 'self_finished'}
    write = record.write_result if artifact == 'result' else record.write_trial_provenance
    write(tmp_path, before)

    def failed(*args):
        raise OSError('disk unavailable')

    monkeypatch.setattr(record.os, failure, failed)
    with pytest.raises(OSError, match='disk unavailable'):
        write(tmp_path, {**before, 'success': True, 'verdict': 'person'})
    assert json.loads((tmp_path / f'{artifact}.json').read_text()) == before
    assert not list(tmp_path.glob(f'.{artifact}-*'))


def test_scrub_covers_the_archived_workspace_and_leaves_binaries_alone(tmp_path):
    from openrua.runner.record import scrub_file, scrub_tree
    ws = tmp_path / "workspace"
    (ws / "notes").mkdir(parents=True)
    (ws / "notes" / "env.txt").write_text("TOKEN=sk-secret-value\n")
    png = bytes([0x89, 0x50, 0x4E, 0x47, 0, 255, 13, 10])
    (ws / "shot.png").write_bytes(png)
    scrub_tree(ws, ["sk-secret-value"])
    assert (ws / "notes" / "env.txt").read_text() == "TOKEN=[REDACTED]\n"
    assert (ws / "shot.png").read_bytes() == png
    scrub_file(tmp_path / "absent.log", ["x"])          # nothing there: nothing to do
