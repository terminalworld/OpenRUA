"""Agents: the occupant package (adapters + launcher + opening prompt).

The harness claim is "agent-agnostic by construction": any CLI-driven
coding agent plugs in with no per-model adaptation. Structurally that
means every fact about a *particular* agent (launch command, auth
layout, transcript format, quota wording) lives in exactly one adapter
module here, and every consumer (launcher, operators, runner,
precheck, demo replay, orchestration audit/trim/batch) touches
only this interface. The boundary test
(orchestration/test_agent_boundary.py) enforces the seam: agent tokens
outside this package fail CI.

Adding an agent = adding one module here implementing every name in
ADAPTER_INTERFACE (the plug shape; test-enforced) and registering it in
``get()`` + ADAPTERS. Nothing else changes.

Selection: configs carry ``agent.cli`` (default ``claude-code``);
recorded per trial in ``operator_meta.cli`` so post-hoc tools
(audit/trim/demo) resolve the same adapter the trial actually ran.
"""

from __future__ import annotations

DEFAULT_CLI = "claude-code"
ADAPTERS = (DEFAULT_CLI,)

# The plug shape: every registered adapter must implement ALL of these
# (machine-enforced by tests/test_agents.py). This list IS the package's
# outward contract; consumers touch adapters only through get() and only
# these names. Optional extras (e.g. PINNED_CLI_VERSION) are read via
# getattr by their consumers.
ADAPTER_INTERFACE = (
    # identity and defaults
    "NAME", "DEFAULT_MODEL", "DEFAULT_EFFORT", "DEFAULT_CREDENTIALS_DIR",
    "CREDENTIALS_FILENAME", "CONFIG_ENV", "VERSION_ARGV",
    # launch
    "launch_argv",
    # build-time emitters (seat install, wall whitelist)
    "sandbox_install", "proxy_filter_lines",
    # auth and precheck checks
    "sandbox_mounts", "credentials_check", "login_hint",
    "sandbox_cli_check", "token_hint",
    # quota
    "quota_probe_argv", "quota_window_open", "matches_quota_anomaly",
    "read_rate_limits",
    # transcript accounting
    "read_final", "scan_transcript", "quota_since",
    "assistant_turns_before",
    # action extraction
    "bash_commands", "replay_ops",
)


def get(cli: str | None = None):
    """Resolve an adapter module by its ``agent.cli`` name."""
    cli = cli or DEFAULT_CLI
    if cli == DEFAULT_CLI:
        from . import claude_code
        return claude_code
    raise KeyError(f"unknown agent cli {cli!r} (available: {DEFAULT_CLI})")


import shutil as _shutil
import subprocess as _subprocess
import tempfile as _tempfile
from pathlib import Path as _Path

# THE opening prompt, inline by ruling 2026-08-15: it is stable skin, a
# separate document earns nothing. Single source shared by the launcher
# (what the agent sees) and the evaluator's provenance hash (audit
# 2026-08-14 F18: divergent sources would pin the hash of a prompt the
# agent never saw). {task} is the only placeholder (guarded by test).
PROMPT = """\
You are working on a robot's onboard computer.

Your task: {task}

Survey the machine yourself to find out what robot this is and what it
can do; work until the task is physically done, verify it your own way,
then finish.

The workspace contains starter docs and tools you can use.
"""

# The only thing said to an agent whose trial was suspended at a quota wall
# and is being resumed (ruling 2026-08-20). Deliberately content-free: the
# session already holds the task, the workspace, and everything the agent
# has done, so restating any of it would hand a resumed trial context an
# uninterrupted one never got. It cannot be made to disappear entirely -- a
# resumed trial receives one more user turn than an uninterrupted one, and
# that structural difference is disclosed rather than hidden.
RESUME_PROMPT = "Continue where you left off."


def prepare_profile(creds_home: _Path, adapter,
                    require_credentials: bool = True
                    ) -> tuple[_Path, _Path | None]:
    """Stage the occupant's luggage: auth profile for one sandbox entry.

    The profile dir (settings and other non-rotating state) is always a
    fresh per-entry copy. What happens to the credentials file depends on
    how the sandbox authenticates:

    - ``require_credentials=True`` is the shared-file arrangement
      (post-incident 2026-08-08): OAuth refresh tokens ROTATE, so every
      consumer must read/write the SAME file or the forks kill each other.
      The file is returned as a path for bind-mounting, never copied.
    - ``require_credentials=False`` is the minted-token arrangement
      (2026-09-02): the sandbox carries its own token, so no credentials
      file is needed or wanted, and the second element comes back None.
      Passing a login profile that happens to hold credentials does not
      change that; nothing gets mounted either way.

    Returns (profile_copy_dir, shared_file_or_None); the caller owns
    deleting the copy dir.
    """
    creds_file = creds_home / adapter.CREDENTIALS_FILENAME
    if not require_credentials:
        creds_file = None
    elif not creds_file.exists():
        raise RuntimeError(
            f"credentials missing: {adapter.login_hint(creds_home)}")
    cfg_dir = _Path(_tempfile.mkdtemp(prefix="robocli-agentcfg-"))
    for pattern in ("*.json", ".*.json"):
        for f in creds_home.glob(pattern):
            if f.name != adapter.CREDENTIALS_FILENAME:
                _shutil.copy2(f, cfg_dir / f.name)
    _subprocess.run(["chmod", "-R", "777", str(cfg_dir)])
    return cfg_dir, creds_file
