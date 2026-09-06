"""Record semantics guards: the commands.sh condensate and the script
operator's contract.

The condensate (ruling 2026-08-16) carries the agent's full world-facing
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

from robocli import agents
from robocli.bench.record import extract_commands


def _tool_use(oid, name, inp):
    return {"type": "assistant", "message": {"content": [
        {"type": "tool_use", "id": oid, "name": name, "input": inp}]}}


def _make_transcript(path: Path, target: Path) -> None:
    tricky = "line with 'quotes' and $VARS\nEOF\nROBOCLI_EOF almost\n"
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
    assert "ROBOCLI_EOF almost" in replayed  # marker dodged the content


def test_script_operator_requires_a_script():
    from robocli.bench.run import OPERATORS, script_operator

    assert set(OPERATORS) == {"none", "script", "agent"}
    with pytest.raises(ValueError, match="--script"):
        script_operator({"script": None})


# ----------------------------------------------- scrubbing and archiving

def test_secret_strings_collects_only_long_values(tmp_path):
    from robocli.bench.record import secret_strings
    (tmp_path / "creds.json").write_text(json.dumps({
        "accessToken": "tok-" + "a" * 30,
        "nested": {"refresh": ["tok-" + "b" * 30]},
        "short": "abc", "count": 5}))
    (tmp_path / "junk.json").write_text("not json at all")  # tolerated
    secrets = secret_strings(tmp_path)
    assert "tok-" + "a" * 30 in secrets and "tok-" + "b" * 30 in secrets
    assert "abc" not in secrets


def test_scrub_file_redacts_every_secret(tmp_path):
    from robocli.bench.record import scrub_file
    f = tmp_path / "transcript.jsonl"
    f.write_text("saw tok-SECRETSECRETSECRETS twice: tok-SECRETSECRETSECRETS")
    scrub_file(f, ["tok-SECRETSECRETSECRETS"])
    body = f.read_text()
    assert "SECRETSECRET" not in body and body.count("[REDACTED]") == 2


def test_archive_prior_attempt_moves_evidence_down(tmp_path):
    from robocli.bench.record import archive_prior_attempt
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


def test_provenance_pins_the_whole_chain(tmp_path):
    from robocli.bench.record import provenance

    class _Agent:
        NAME = "stub"
        VERSION_ARGV = ["echo", "9.9.9"]

    class _Args:
        operator, task_suite = "none", "suite"
        task_ids, seeds = [0], [0]
        wall_clock_min, ros_domain = 30, 44

    code_root = Path(__file__).resolve().parents[1]
    cfg_path = tmp_path / "assembly.yaml"
    cfg_path.write_text("machine: {}\n")
    # a substrate is a git checkout holding the simulator venv; stand one up
    substrate = tmp_path / "substrates" / "cap-x"
    (substrate / ".venv-libero").mkdir(parents=True)
    import subprocess
    subprocess.run(["git", "init", "-q", str(substrate)], check=True)
    subprocess.run(["git", "-C", str(substrate), "-c", "user.name=t",
                    "-c", "user.email=t@t", "commit", "-q", "--allow-empty",
                    "-m", "x"], check=True)
    cfg = {"machine": {"body": {
        "image": "robocli-definitely-missing",
        "substrate": {"venv": str(substrate / ".venv-libero")},
        "gpus": False}}}
    prov = provenance(cfg_path, cfg, _Args(), _Agent(), "tplhash",
                      "prompt text", code_root,
                      substrate_venv=substrate / ".venv-libero")
    assert prov["robocli_version"]
    assert len(prov["robocli_commit"]) == 40
    assert len(prov["substrate_commit"]) == 40  # the cap-x checkout is git
    assert prov["host"]["hostname"] and prov["host"]["cpu_count"] >= 1
    assert prov["ros_domain"] == 44 and prov["gpu_render"] is False
    assert prov["agent_cli"] == {"name": "stub", "version": "9.9.9"}
    assert prov["prompt_sha256"] and prov["config_sha256"]
    # a missing image must record its probe result, never kill the trial
    assert isinstance(prov["sim_image_digest"], str)


def test_run_config_is_write_once(tmp_path):
    from robocli.bench.record import write_run_config
    write_run_config(tmp_path, {"gen": 1})
    write_run_config(tmp_path, {"gen": 2})  # concurrent runner: no rewrite
    assert json.loads((tmp_path / "config.json").read_text()) == {"gen": 1}
