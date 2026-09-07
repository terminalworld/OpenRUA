"""Codex hooks: the behaviour behind the codex manifest.

Headless launch is ``codex exec --json``: one JSONL event per line
(``thread.started``, ``turn.started``, ``item.started`` /
``item.completed`` with an item of type ``command_execution``,
``file_change`` or ``agent_message``, ``turn.completed`` with token
usage). Approvals and the CLI's own sandbox are switched off because the
container is the sandbox; web search is disabled because it cannot be
routed through the proxy. A resumed launch continues the most recent
session in the sandbox's own CODEX_HOME (``codex exec resume --last``).

The CLI has no turn budget flag, so ``max_turns`` is not enforced by it;
``read_final`` counts completed items as the agent's turns so the runner
can carry its budget across segments. A ``file_change`` event names the
file but not its content, so replay covers shell commands only.
"""

from __future__ import annotations

import json
from pathlib import Path
from typing import Any

from robocli.agents.base import Agent

QUOTA_PHRASES = ("rate limit", "usage limit", "quota")
_ACTION_ITEMS = ("command_execution", "file_change", "agent_message")


class Codex(Agent):
    """Hooks only; every fact comes from the manifest."""

    def _env(self, proxy: str) -> list[str]:
        return [
            "-e", f"{self.credentials.config_env}={self.credentials.mount_point}",
            "-e", f"HTTPS_PROXY={proxy}",
            "-e", f"HTTP_PROXY={proxy}",
        ]

    def _opts(self, options: dict[str, Any] | None) -> dict[str, Any]:
        return {**self.default_options, **(options or {})}

    def _config_flags(self, opts: dict[str, Any]) -> list[str]:
        return ["-c", f'model_reasoning_effort="{opts["effort"]}"',
                "-c", f'web_search="{opts["web_search"]}"']

    def launch_argv(self, sandbox: str, prompt: str, model: str, max_turns: int,
                    proxy: str, options: dict[str, Any] | None = None,
                    session_id: str | None = None, resume: bool = False,
                    token_file: str | None = None, **_: Any) -> list[str]:
        opts = self._opts(options)
        verb = ["exec", "resume", "--last"] if resume else ["exec"]
        return [
            *self.exec_argv(sandbox, self._env(proxy), token_file),
            "codex", *verb,
            "--json",
            "--skip-git-repo-check",
            "--dangerously-bypass-approvals-and-sandbox",
            "-C", "/workspace",
            "--model", model,
            *self._config_flags(opts),
            prompt,
        ]

    def interactive_argv(self, sandbox: str, model: str, proxy: str,
                         options: dict[str, Any] | None = None,
                         prompt: str | None = None, **_: Any) -> list[str]:
        opts = self._opts(options)
        return [
            *self.exec_argv(sandbox, self._env(proxy), interactive=True),
            "codex", "--skip-git-repo-check", "--dangerously-bypass-approvals-and-sandbox",
            "-C", "/workspace", "--model", model, *self._config_flags(opts),
            *([prompt] if prompt else []),
        ]

    def sandbox_cli_check(self) -> tuple[str, str] | None:
        """The CLI inside the sandbox is the pinned version; no check
        without a pin."""
        if not self.version:
            return None
        return ("sandbox_cli_matches_pin",
                f"bash -c 'codex --version | grep -qF {self.version}'")

    def login_hint(self, creds_home: Path) -> str:
        return f"run {self.credentials.config_env}={creds_home} codex login"

    def token_hint(self, token_file: Path | str) -> str:
        return (f"write an API key as umask 077 && printf '{self.token_env}=%s\\n' "
                f"'<key>' > {token_file}")

    def quota_probe_argv(self, model: str) -> list[str]:
        return ["codex", "exec", "--json", "--skip-git-repo-check", "--ephemeral",
                "--model", model, "Reply with exactly: OK"]

    def quota_window_open(self, returncode: int, text: str) -> bool:
        return returncode == 0 and "limit" not in text.lower()

    def matches_quota_anomaly(self, text: str) -> bool:
        low = text.lower()
        return any(k in low for k in QUOTA_PHRASES)

    def read_final(self, transcript: Path) -> dict:
        """Totals across every segment in the transcript: completed items
        as turns, token usage summed over ``turn.completed`` events."""
        turns = 0
        usage: dict[str, int] = {}
        segments = 0
        for rec in _iter_records(transcript):
            t = rec.get("type")
            if t == "item.completed" and rec.get("item", {}).get("type") in _ACTION_ITEMS:
                turns += 1
            elif t == "turn.completed":
                segments += 1
                for k, v in (rec.get("usage") or {}).items():
                    if isinstance(v, (int, float)):
                        usage[k] = usage.get(k, 0) + v
        if not segments and not turns:
            return {}
        return {"num_turns": turns, "hit_max_turns": False, "usage": usage or None,
                "segments": segments}

    def scan_transcript(self, transcript: Path) -> dict:
        ev: dict = {"quota": None, "malformed_lines": 0, "lines": 0,
                    "has_final_result": False, "final_is_error": False,
                    "api_transport_error": False, "logged_out_launch": False}
        try:
            lines = transcript.read_text(errors="replace").splitlines()
        except OSError as e:
            ev["read_error"] = str(e)
            return ev
        for i, line in enumerate(lines, 1):
            if not line.strip():
                continue
            ev["lines"] += 1
            try:
                rec = json.loads(line)
            except json.JSONDecodeError:
                ev["malformed_lines"] += 1
                continue
            t = rec.get("type", "")
            if t == "turn.completed":
                ev["has_final_result"] = True
                ev["final_is_error"] = False
            elif t in ("turn.failed", "error") or rec.get("item", {}).get("type") == "error":
                text = json.dumps(rec)[:200]
                ev["has_final_result"] = True
                ev["final_is_error"] = True
                ev["final_text"] = text
                low = text.lower()
                if ev["quota"] is None and any(p in low for p in QUOTA_PHRASES):
                    ev["quota"] = {"evidence": f"{transcript.name}:{i}:error", "message": text[:160]}
                ev["logged_out_launch"] = "not logged in" in low or "login" in low and "auth" in low
        return ev

    def quota_since(self, transcript: Path, line: int = 0) -> dict | None:
        try:
            lines = transcript.read_text(errors="replace").splitlines()
        except OSError:
            return None
        for i in range(max(0, line), len(lines)):
            low = lines[i].lower()
            if ('"type": "turn.failed"' in low or '"type":"turn.failed"' in low
                    or '"type": "error"' in low or '"type":"error"' in low) \
                    and any(p in low for p in QUOTA_PHRASES):
                return {"evidence": f"{transcript.name}:{i + 1}:error",
                        "message": lines[i][:160], "resets_at": None}
        return None

    def replay_ops(self, transcript: Path) -> list[dict]:
        """Shell commands in order (``command_execution`` items); file
        changes carry no content in the transcript and are not replayed."""
        ops: list[dict] = []
        for rec in _iter_records(transcript):
            item = rec.get("item", {})
            if rec.get("type") == "item.completed" and item.get("type") == "command_execution":
                ops.append({"kind": "shell", "command": item.get("command", ""),
                            "output": item.get("aggregated_output", ""), "duration_s": 0.0})
        return ops


def _iter_records(transcript: Path):
    try:
        text = transcript.read_text(errors="replace")
    except OSError:
        return
    for line in text.splitlines():
        line = line.strip()
        if not line:
            continue
        try:
            yield json.loads(line)
        except json.JSONDecodeError:
            continue


HOOKS = Codex
