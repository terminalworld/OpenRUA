"""Trial record semantics: what counts, what gets written, what never leaks.

The examiner's bookkeeping: provenance content, secret scrubbing, the
commands.sh evidence extract, and the writing of every file under a
trial directory. This module is the only writer under ``runs/``; WHERE
and WHEN to write is the conductor's decision and arrives as plain path
parameters; WHAT the files say is decided here. The blind protocol
holds throughout: success predicates belong to the benchmark's own code
in the sim process and never appear on the agent's surface.

Leaf discipline: no robocli imports. The agent adapter, the workspace
template hash, the prompt text, and the repo root are handed in by the
conductor (parameter passing, never lookup).
"""

from __future__ import annotations

import hashlib
import json
import subprocess
from datetime import datetime, timezone
from pathlib import Path

import yaml


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
    """Redact known secret strings before anything enters the git-tracked
    record (defense against the agent cat-ing its own config)."""
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
    import shlex
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


def extract_commands(transcript: Path, out: Path, agent) -> None:
    """Full action condensate: the agent's world-facing ops in order,
    every detour included. Bash commands verbatim; Write/Edit
    materialized as shell here-docs (agents keep scripts in the
    sandbox's /tmp, which is NOT in the archived workspace); reads are
    not extracted (zero world effect; they stay in the transcript).

    Faithful evidence first; the same file feeds ``--operator script``
    for OPEN-LOOP best-effort replay. The agent ran closed-loop, so an
    identical outcome is likely under paused clock + seeded reset, never
    guaranteed. No ``set -e``: the agent's commands failed and moved on;
    the condensate does the same. Transcript parsing is the adapter's
    job (adapter passed in)."""
    if not transcript.exists():
        return
    lines = ["#!/usr/bin/env bash",
             f"# auto-extracted from {transcript.name} "
             "(shell + write/edit condensate)", ""]
    for op in agent.replay_ops(transcript):
        kind = op.get("kind")
        if kind == "write":
            lines.append(_write_as_command(op.get("path", ""), op.get("content", "")))
        elif kind == "edit":
            lines.append(_edit_as_command(op))
        elif kind == "shell" and op.get("command"):
            lines.append(op["command"])
        else:
            continue
        lines.append("")
    out.write_text("\n".join(lines))


def provenance(cfg_path: Path, cfg: dict, args, agent, template_hash: str,
               prompt: str, code_root: Path, substrate_venv: Path | None = None,
               resume_prompt: str | None = None) -> dict:
    """The frozen reproduction record. ``prompt`` is the SAME string the
    launcher formats (F18: one source; the conductor passes agents.PROMPT;
    inline by ruling 2026-08-15, no prompt file exists). ``resume_prompt``
    is the other string an agent can be shown -- the one a trial resumed
    after a quota wall receives -- pinned for the same reason."""

    def sh(cmd: list[str]) -> str:
        # Provenance must never kill a trial: the 2026-08-09 incident was
        # a FileNotFoundError from the agent CLI's version probe during
        # its auto-update window taking down the whole batch.
        try:
            return subprocess.run(cmd, capture_output=True, text=True).stdout.strip()
        except OSError:
            return "unavailable"

    # The leg's ACTUAL image (audit 2026-08-14 F9: a hardcoded jazzy name
    # recorded a digest the humble legs never ran).
    body = cfg.get("machine", {}).get("body", {})
    image = body.get("image", "robocli-sim-jazzy")
    # All three containers a trial runs in: the agent's toolchain
    # (sandbox) and the wall (proxy) are experiment conditions as much
    # as the sim body (completeness ruling 2026-08-18: record
    # generously; the RUNBOOK reconciles builds against these).
    sandbox_image = body.get("sandbox_image", "robocli-sandbox")
    # The benchmark content itself (tasks, predicates, the vendored
    # forks) lives in the substrate checkout; its commit is as much a
    # link in the reproduction chain as our own. The checkout root is
    # the directory holding the substrate venv (resolved by the caller).
    substrate = str(substrate_venv.parent) if substrate_venv else ""
    import os
    import platform
    from robocli import __version__ as robocli_version
    return {
        "created_utc": datetime.now(timezone.utc).isoformat(),
        "robocli_version": robocli_version,
        # A git commit when the code runs from a checkout (editable
        # install); "unavailable" from a wheel, where the version above
        # is the pin.
        "robocli_commit": sh(["git", "-C", str(code_root), "rev-parse", "HEAD"]),
        "git_dirty": bool(sh(["git", "-C", str(code_root), "status", "--porcelain"])),
        "substrate_commit": (
            sh(["git", "-C", substrate, "rev-parse", "HEAD"])
            if substrate else "unavailable"),
        "substrate_dirty": bool(
            sh(["git", "-C", substrate, "status", "--porcelain"])
            if substrate else False),
        # Wall-clock trials render at host speed (llvmpipe scales with
        # cores); where a trial ran is a condition, not trivia.
        "host": {"hostname": platform.node(), "cpu_count": os.cpu_count()},
        "ros_domain": getattr(args, "ros_domain", None),
        "sim_image_digest": sh(
            ["docker", "image", "inspect", "--format", "{{.Id}}", image]
        ),
        "sandbox_image_digest": sh(
            ["docker", "image", "inspect", "--format", "{{.Id}}", sandbox_image]
        ),
        "proxy_image_digest": sh(
            ["docker", "image", "inspect", "--format", "{{.Id}}", "robocli-proxy"]
        ),
        # Render device disclosure: physics is CPU either way (results
        # comparable across devices); recorded so nobody has to dig it
        # out of bridge.log.
        "gpu_render": bool(body.get("gpus", False)),
        # Relative to where the run was started when it lives there
        # (ruling 2026-08-17: records keep no absolute paths they can
        # avoid); a bundled config records its package path.
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
        "agent_cli": {"name": agent.name,
                      "version": sh(list(agent.version_argv)) if agent.version_argv
                      else "unavailable"},
        "operator": args.operator,
        "task_suite": args.task_suite,
        "task_ids": args.task_ids,
        "seeds": args.seeds,
        "wall_clock_cap_min": args.wall_clock_min,
    }


def write_assembly(trial_dir: Path, cfg: dict) -> Path:
    """The trial's resolved suite view, computed ONCE by the conductor
    and written here as the artifact every consumer reads (manual
    seeding, body bringup, the machine itself). One computation, one
    file, zero chance of same-code-different-arguments drift (ruling
    2026-08-16)."""
    path = trial_dir / "assembly.yaml"
    path.write_text(yaml.safe_dump(cfg, sort_keys=False))
    return path


def write_result(trial_dir: Path, record: dict) -> None:
    (trial_dir / "result.json").write_text(json.dumps(record, indent=2))


def write_trial_provenance(trial_dir: Path, prov: dict) -> None:
    (trial_dir / "provenance.json").write_text(json.dumps(prov, indent=2))


def write_run_config(run_dir: Path, prov: dict) -> None:
    """Run-level config.json is write-once (concurrent single-trial runners
    under the batch master must not rewrite each other's copy); the
    authoritative per-execution record is each trial's provenance.json."""
    if not (run_dir / "config.json").exists():
        (run_dir / "config.json").write_text(json.dumps(prov, indent=2))


def archive_prior_attempt(trial_dir: Path) -> Path | None:
    """Move any existing trial artifacts into attempts/attempt-NNNN before a
    rerun writes new ones. Evidence is never overwritten (notes/13 §3.2;
    minimal-form archiving: layout stays flat, superseded attempts move
    down, the top level is always the latest attempt)."""
    if not trial_dir.exists():
        return None
    entries = [p for p in trial_dir.iterdir() if p.name != "attempts"]
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
    """Post-trial evidence hygiene: scrub secrets from everything the
    agent or the sim could have echoed, then extract the replay script."""
    transcript = trial_dir / "transcript.jsonl"
    scrub_file(transcript, secrets)
    scrub_file(trial_dir / "bridge.log", secrets)
    extract_commands(transcript, trial_dir / "commands.sh", agent)
