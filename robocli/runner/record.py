"""What a trial writes: provenance, secret scrubbing, the commands.sh
extract, and every file under a trial directory.

This module is the only writer under ``runs/``. Where and when to write
is the runner's decision and arrives as path parameters; what the files
say is decided here. Success predicates belong to the benchmark's own
code in the bridge and never appear on the agent's surface.

Leaf: no robocli imports. The agent, the workspace template hash, the
prompt text and the code root are handed in by the runner.
"""

from __future__ import annotations

import hashlib
import json
import os
import platform
import shlex
import subprocess
from datetime import datetime, timezone
from pathlib import Path
from typing import Iterable

import yaml

from robocli import __version__ as robocli_version


def secret_strings(creds_dir: Path) -> list[str]:
    """Every long string value in the credential files = a secret."""
    secrets: list[str] = []
    for f in creds_dir.glob("*.json"):
        try:
            blob = json.loads(f.read_text())
        except Exception:  # noqa: BLE001
            continue

        def walk(v):
            if isinstance(v, dict):
                for x in v.values():
                    walk(x)
            elif isinstance(v, list):
                for x in v:
                    walk(x)
            elif isinstance(v, str) and len(v) >= 20:
                secrets.append(v)

        walk(blob)
    return secrets


def secret_strings_from_env_file(path: Path) -> list[str]:
    """The values in a KEY=value env file = secrets, same rule as above.

    Used for the sandbox's minted auth token, which lives in a file rather
    than in the credential JSONs. Short values are skipped for the same
    reason secret_strings() skips them: a two-character value scrubbed out
    of a transcript would mangle it.
    """
    try:
        text = path.read_text()
    except OSError:
        return []
    out = []
    for line in text.splitlines():
        line = line.strip()
        if not line or line.startswith("#") or "=" not in line:
            continue
        value = line.split("=", 1)[1].strip()
        if len(value) >= 20:
            out.append(value)
    return out


def scrub_file(path: Path, secrets: list[str]) -> None:
    """Redact known secret strings before anything enters the record (the
    agent can print its own configuration)."""
    if not path.exists() or not secrets:
        return
    text = path.read_text(errors="replace")
    for s in secrets:
        text = text.replace(s, "[REDACTED]")
    path.write_text(text)


def _heredoc_marker(content: str) -> str:
    """A terminator line guaranteed absent from the content."""
    m = "ROBOCLI_EOF"
    while m in content:
        m += "_X"
    return m


def _write_as_command(path: str, content: str) -> str:
    q = shlex.quote(path)
    m = _heredoc_marker(content)
    nl = "" if (not content or content.endswith("\n")) else "\n"
    return (f"mkdir -p \"$(dirname {q})\"\n"
            f"cat > {q} <<'{m}'\n{content}{nl}{m}")


def _edit_as_command(op: dict) -> str:
    payload = repr(json.dumps({
        "file_path": op.get("path", ""),
        "old_string": op.get("old", ""),
        "new_string": op.get("new", ""),
        "replace_all": bool(op.get("replace_all")),
    }))
    m = _heredoc_marker(payload)
    return (f"python3 - <<'{m}'\n"
            "import json\n"
            f"op = json.loads({payload})\n"
            "p = op['file_path']\n"
            "body = open(p).read()\n"
            "assert op['old_string'] in body, 'edit target not found: ' + p\n"
            "n = -1 if op['replace_all'] else 1\n"
            "open(p, 'w').write(body.replace(op['old_string'], op['new_string'], n))\n"
            f"{m}")


OP_MARKER = "# robocli op "
OUTPUT_HEAD = 4000  # characters of an operation's output kept in ops.jsonl


def as_command(op: dict) -> str | None:
    """One operation as the shell that performs it: a shell command
    verbatim, a write or an edit as a here-doc. None for anything
    without a world effect."""
    kind = op.get("kind")
    if kind == "write":
        return _write_as_command(op.get("path", ""), op.get("content", ""))
    if kind == "edit":
        return _edit_as_command(op)
    if kind == "shell" and op.get("command"):
        return op["command"]
    return None


def extract_commands(transcript: Path, out: Path, agent) -> None:
    """The agent's world-facing operations in order, every detour
    included: shell commands verbatim, writes and edits as shell
    here-docs (agents keep scripts in the sandbox's /tmp, which is not
    in the archived workspace); reads are not extracted (no world
    effect; they stay in the transcript). A ``# robocli op N`` line
    opens each operation, which is how ``--operator script`` replays
    the file one operation at a time.

    Evidence first; the same file feeds ``--operator script`` for
    open-loop replay. The agent ran closed-loop, so an identical outcome
    is likely under a paused clock and a seeded reset, never guaranteed.
    No ``set -e``: the agent's commands failed and moved on, and the
    replay does the same. Transcript parsing is the agent's hook."""
    if not transcript.exists():
        return
    lines = ["#!/usr/bin/env bash",
             f"# auto-extracted from {transcript.name} "
             "(shell + write/edit condensate)", ""]
    for i, op in enumerate(agent.replay_ops(transcript)):
        cmd = as_command(op)
        if cmd is None:
            continue
        lines += [f"{OP_MARKER}{i}", cmd, ""]
    out.write_text("\n".join(lines))


def write_ops(trial_dir: Path, ops: Iterable[dict]) -> Path:
    """``ops.jsonl``: the timed operation stream a demo is rendered from.
    One line per operation: ``i``, ``kind``, the shell text (``command``),
    the output's head and the wall times ``t0``/``t1`` at which it ran.
    From a transcript the times are the agent's; from a script replay
    the operator's own."""
    path = trial_dir / "ops.jsonl"
    with path.open("w") as f:
        for i, op in enumerate(ops):
            cmd = as_command(op)
            if cmd is None:
                continue
            f.write(json.dumps({
                "i": i, "kind": op.get("kind"), "command": cmd,
                "output": str(op.get("output", ""))[:OUTPUT_HEAD],
                "t0": op.get("t0"), "t1": op.get("t1"),
            }) + "\n")
    return path


def provenance(cfg_path: Path, cfg: dict, args, agent, template_hash: str,
               prompt: str, code_root: Path, proxy_image: str,
               simulator_venv: Path | None = None,
               resume_prompt: str | None = None) -> dict:
    """The reproduction record. ``prompt`` is the same string the launcher
    formats (one source: the runner passes agents.PROMPT); ``resume_prompt``
    is the other string an agent can be shown, the one a trial resumed
    after a quota wall receives, pinned for the same reason."""

    def sh(cmd: list[str]) -> str:
        # Provenance must never kill a trial: a probe that fails records
        # "unavailable".
        try:
            return subprocess.run(cmd, capture_output=True, text=True).stdout.strip()
        except OSError:
            return "unavailable"

    # The images this trial actually ran in: the agent's toolchain
    # (sandbox) and the proxy are experiment conditions as much as the
    # robot image.
    backend = cfg.get("machine", {}).get("backend", {})
    image = backend.get("image") or ""
    sandbox_image = backend.get("sandbox_image") or ""
    # The benchmark content itself (tasks, predicates, the vendored
    # forks) lives in the simulator checkout; its commit is as much a
    # link in the reproduction chain as our own. The checkout root is
    # the directory holding the simulator venv (resolved by the caller).
    simulator = str(simulator_venv.parent) if simulator_venv else ""
    return {
        "created_utc": datetime.now(timezone.utc).isoformat(),
        "robocli_version": robocli_version,
        # A git commit when the code runs from a checkout (editable
        # install); "unavailable" from a wheel, where the version above
        # is the pin.
        "robocli_commit": sh(["git", "-C", str(code_root), "rev-parse", "HEAD"]),
        "git_dirty": bool(sh(["git", "-C", str(code_root), "status", "--porcelain"])),
        "simulator_commit": (
            sh(["git", "-C", simulator, "rev-parse", "HEAD"])
            if simulator else "unavailable"),
        "simulator_dirty": bool(
            sh(["git", "-C", simulator, "status", "--porcelain"])
            if simulator else False),
        # Software rendering runs at host speed (llvmpipe scales with
        # cores); where a trial ran is a condition.
        "host": {"hostname": platform.node(), "cpu_count": os.cpu_count()},
        "ros_domain": getattr(args, "ros_domain", None),
        "sim_image_digest": sh(
            ["docker", "image", "inspect", "--format", "{{.Id}}", image]
        ),
        "sandbox_image_digest": sh(
            ["docker", "image", "inspect", "--format", "{{.Id}}", sandbox_image]
        ),
        "proxy_image_digest": sh(
            ["docker", "image", "inspect", "--format", "{{.Id}}", proxy_image]
        ),
        # Render device: physics is CPU either way, so results are
        # comparable across devices; recorded so nobody digs it out of
        # bridge.log.
        "gpu_render": bool(backend.get("gpus", False)),
        # Camera recording renders every step and costs wall clock, so a
        # recorded trial says so: None = off, [] = the profile's
        # cameras.record, else the names given.
        "record": (None if getattr(args, "record", None) is None
                   else [c for c in str(args.record).split(",") if c.strip()]),
        # Whatever answers to `docker` on this host (Docker Engine or a
        # podman with its docker-compatible command); one string as the
        # engine prints it, plus the flags the sandbox container took.
        "container_engine": sh(["docker", "--version"]) or "unavailable",
        "sandbox_run_args": list(cfg.get("sandbox", {}).get("run_args", [])),
        # Relative to where the run was started when it lives there
        # (records keep no absolute paths they can avoid); a bundled
        # config records its package path.
        "config_file": str(cfg_path.relative_to(Path.cwd())
                           if cfg_path.is_relative_to(Path.cwd())
                           else cfg_path),
        "config_sha256": hashlib.sha256(cfg_path.read_bytes()).hexdigest(),
        "prompt_sha256": hashlib.sha256(prompt.encode()).hexdigest(),
        # Hashed separately rather than blended into prompt_sha256: each
        # field then names exactly the text it pins, and the opening
        # prompt's hash keeps the meaning it has always had.
        "resume_prompt_sha256":
            hashlib.sha256(resume_prompt.encode()).hexdigest()
            if resume_prompt is not None else None,
        "workspace_template_sha256": template_hash,
        # The version that ran is read inside each trial's sandbox and
        # recorded there (agent_version); the pin, if any, is the config's.
        "agent_cli": {"name": agent.name, "version_pin": agent.version},
        "operator": args.operator,
        "task_suite": args.task_suite,
        "task_ids": args.task_ids,
        "seeds": args.seeds,
        "wall_clock_cap_min": args.wall_clock_min,
    }


def write_config(trial_dir: Path, cfg: dict) -> Path:
    """The trial's resolved config, computed once by the runner and
    written here as the file every party reads (workspace seeding, the
    robot's bring-up, the bridge)."""
    path = trial_dir / "config.yaml"
    path.write_text(yaml.safe_dump(cfg, sort_keys=False))
    return path


def write_result(trial_dir: Path, record: dict) -> None:
    (trial_dir / "result.json").write_text(json.dumps(record, indent=2))


def write_trial_provenance(trial_dir: Path, prov: dict) -> None:
    (trial_dir / "provenance.json").write_text(json.dumps(prov, indent=2))


def write_run_config(run_dir: Path, prov: dict) -> None:
    """Run-level config.json is write-once (concurrent single-trial
    runners must not rewrite each other's copy); the per-execution record
    is each trial's provenance.json."""
    if not (run_dir / "config.json").exists():
        (run_dir / "config.json").write_text(json.dumps(prov, indent=2))


def write_run_summary(run_dir: Path) -> Path | None:
    """``SUMMARY.md`` for a run, regenerated from the artifacts under it
    (the run's config.json and every trial's result.json), so it is
    always a faithful view and never a second record. Written atomically:
    concurrent single-trial runners share a run directory."""
    cfg_file = run_dir / "config.json"
    if not cfg_file.is_file():
        return None
    prov = json.loads(cfg_file.read_text())
    rows = []
    for result in sorted(run_dir.glob("trials/*/seed*/result.json")):
        try:
            r = json.loads(result.read_text())
        except (OSError, json.JSONDecodeError):
            continue
        meta = r.get("operator_meta") or {}
        rows.append({
            "suite": r.get("task_suite", result.parents[1].name.rsplit("-", 1)[0]),
            "task": r.get("task_id"), "seed": r.get("init_state_id"),
            "success": r.get("success"), "termination": r.get("termination"),
            "wall_s": r.get("wall_seconds"), "turns": meta.get("num_turns"),
            "model": meta.get("model"), "agent_version": r.get("agent_version"),
            "anomaly": r.get("anomaly"),
        })
    scored = [x for x in rows if x["success"] is not None]
    successes = sum(1 for x in scored if x["success"])
    agent = prov.get("agent_cli", {})
    cfg = prov.get("config", {})
    # The model actually run: the config's, else the agent's default as
    # the first trial recorded it.
    model = cfg.get("agent", {}).get("model") or next(
        (x["model"] for x in rows if x.get("model")), "?")
    version = agent.get("version_pin") or next(
        (x["agent_version"] for x in rows if x.get("agent_version")), "")
    lines = [f"# {run_dir.name}", "",
             f"- benchmark: {cfg.get('task', {}).get('benchmark', '?')}",
             f"- config: `{prov.get('config_file', '?')}` (sha256 {str(prov.get('config_sha256', ''))[:12]})",
             f"- agent: {agent.get('name', '?')} {version}, model "
             f"{model}, operator {prov.get('operator', '?')}",
             f"- code: robocli {prov.get('robocli_version', '?')} @ "
             f"{str(prov.get('robocli_commit', ''))[:12]}"
             f"{' (dirty)' if prov.get('git_dirty') else ''}; "
             f"simulator @ {str(prov.get('simulator_commit', ''))[:12]}",
             f"- images: robot {str(prov.get('sim_image_digest', ''))[:19]}, sandbox "
             f"{str(prov.get('sandbox_image_digest', ''))[:19]}, proxy "
             f"{str(prov.get('proxy_image_digest', ''))[:19]}",
             f"- trials: {len(rows)} recorded; "
             + (f"{successes}/{len(scored)} succeeded" if scored else "no automatic verdict")
             + f"; {sum(1 for x in rows if x['anomaly'])} anomalies", "",
             "| suite | task | seed | success | termination | wall s | turns |",
             "|---|---|---|---|---|---|---|"]
    for x in rows:
        success = {True: "yes", False: "no", None: "n/a"}[x["success"]]
        lines.append(f"| {x['suite']} | {x['task']} | {x['seed']} | {success} | "
                     f"{x['termination']} | {x['wall_s']} | {x['turns'] if x['turns'] is not None else ''} |")
    anomalies = [x for x in rows if x["anomaly"]]
    if anomalies:
        lines += ["", "## Anomalies", ""]
        lines += [f"- {x['suite']} task {x['task']} seed {x['seed']}: {x['anomaly']}"
                  for x in anomalies]
    out = run_dir / "SUMMARY.md"
    tmp = run_dir / ".SUMMARY.md.tmp"
    tmp.write_text("\n".join(lines) + "\n")
    tmp.replace(out)
    return out


def archive_prior_attempt(trial_dir: Path, keep: Iterable[Path] = ()) -> Path | None:
    """Move any existing trial artifacts into attempts/attempt-NNNN before
    a rerun writes new ones. Evidence is never overwritten: the top level
    is always the latest attempt, superseded ones move down. ``keep``
    names entries that belong to the attempt about to run rather than
    to a previous one (its claim on the directory); they stay in place."""
    if not trial_dir.exists():
        return None
    kept = {Path(k).name for k in keep} | {"attempts"}
    entries = [p for p in trial_dir.iterdir() if p.name not in kept]
    if not any(p.name in ("result.json", "transcript.jsonl") for p in entries):
        return None
    attempts = trial_dir / "attempts"
    attempts.mkdir(exist_ok=True)
    n = len(list(attempts.glob("attempt-*"))) + 1
    dest = attempts / f"attempt-{n:04d}"
    dest.mkdir()
    for p in entries:
        p.rename(dest / p.name)
    return dest


def finalize_trial(trial_dir: Path, agent, secrets: list[str]) -> None:
    """After a trial: scrub secrets from everything the agent or the sim
    could have echoed, then extract the replay script and the timed
    operation stream from the transcript (an operator that wrote
    ``ops.jsonl`` itself has no transcript)."""
    transcript = trial_dir / "transcript.jsonl"
    scrub_file(transcript, secrets)
    scrub_file(trial_dir / "bridge.log", secrets)
    if transcript.exists():
        extract_commands(transcript, trial_dir / "commands.sh", agent)
        write_ops(trial_dir, agent.replay_ops(transcript))
