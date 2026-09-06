"""Claude Code hooks: the behaviour behind the claude-code manifest.

Reached through ``robocli.agents.get("claude-code")``, which composes
this class with ``configs/agents/claude-code.yaml``:

1. launch: the in-sandbox headless command (``claude -p``; permissions
   skipped inside the container; WebSearch/WebFetch disabled at the CLI
   layer because server-side search cannot be proxied; stream-json
   transcript on stdout).
2. auth: a profile directory the CLI reads from ``CLAUDE_CONFIG_DIR``
   (the shared credentials file is bind-mounted, the rest copied), or a
   token passed by file. No API key anywhere.
3. transcript accounting: stream-json records; the final ``result``
   record (num_turns, is_error, subtype), ``assistant`` records with
   timestamps.
4. action extraction: world-facing tool_use blocks (Bash/Write/Edit),
   streamed as growing snapshots per tool_use id, normalized to
   shell/write/edit operations.
5. quota: probe request, quota phrases, the rejected rate_limit_event shape.
"""

from __future__ import annotations

import json
from datetime import datetime
from pathlib import Path
from typing import Any

from robocli.agents.base import Agent

QUOTA_PHRASES = ("session limit", "rate limit", "weekly limit", "usage limit")
_REPLAY_TOOLS = ("Bash", "Write", "Edit")


class ClaudeCode(Agent):
    """Hooks only; every fact comes from the manifest."""

    # ---------------------------------------------------------- launch

    def _env(self, proxy: str, opts: dict[str, Any]) -> list[str]:
        return [
            "-e", f"{self.credentials.config_env}={self.credentials.mount_point}",
            "-e", f"HTTPS_PROXY={proxy}",
            "-e", f"HTTP_PROXY={proxy}",
            "-e", f"BASH_DEFAULT_TIMEOUT_MS={opts['bash_timeout_ms']}",
            "-e", f"BASH_MAX_TIMEOUT_MS={opts['bash_max_timeout_ms']}",
        ]

    def _opts(self, options: dict[str, Any] | None) -> dict[str, Any]:
        return {**self.default_options, **(options or {})}

    def launch_argv(self, sandbox: str, prompt: str, model: str, max_turns: int,
                    proxy: str, options: dict[str, Any] | None = None,
                    session_id: str | None = None, resume: bool = False,
                    token_file: str | None = None, **_: Any) -> list[str]:
        """``--resume`` names an existing session; ``--session-id`` creates
        the named one. They are not interchangeable, so pick by phase. A
        resumed launch carries no task restatement (the session holds it)."""
        opts = self._opts(options)
        session: list[str] = []
        if session_id:
            session = ["--resume", session_id] if resume else ["--session-id", session_id]
        return [
            "docker", "exec",
            "-u", "robot", "-w", "/workspace",
            *(["--env-file", token_file] if token_file else []),
            *self._env(proxy, opts),
            sandbox,
            "claude",
            *session,
            "-p", prompt,
            "--model", model,
            "--effort", str(opts["effort"]),
            "--autocompact", str(opts["autocompact"]),
            "--dangerously-skip-permissions",
            "--disallowedTools", "WebSearch", "WebFetch",
            "--max-turns", str(max_turns),
            "--output-format", "stream-json",
            "--verbose",
        ]

    def interactive_argv(self, sandbox: str, model: str, proxy: str,
                         options: dict[str, Any] | None = None,
                         prompt: str | None = None, **_: Any) -> list[str]:
        """Same user, mounts and proxy as the headless launch; the optional
        prompt becomes the opening message."""
        opts = self._opts(options)
        return [
            "docker", "exec", "-it",
            "-u", "robot", "-w", "/workspace",
            *self._env(proxy, opts),
            sandbox,
            "claude", "--model", model, "--effort", str(opts["effort"]),
            *([prompt] if prompt else []),
        ]

    def sandbox_cli_check(self) -> tuple[str, str]:
        """The CLI inside the sandbox is the version the manifest pins."""
        return ("sandbox_cli_matches_pin",
                f"bash -c 'claude --version | grep -qF {self.version}'")

    # ------------------------------------------------------------ auth

    def login_hint(self, creds_home: Path) -> str:
        return f"run {self.credentials.config_env}={creds_home} claude login"

    def token_hint(self, token_file: Path | str) -> str:
        """Minting authorises whichever account is logged in to the browser,
        not the one the profile environment variable points at."""
        return (f"mint one with `claude setup-token` (it authorises the account "
                f"logged in to your browser), then: umask 077 && printf "
                f"'{self.token_env}=%s\\n' '<token>' > {token_file}")

    # ----------------------------------------------------------- quota

    def quota_probe_argv(self, model: str) -> list[str]:
        """One minimal request; the window is judged by quota_window_open()."""
        return ["claude", "-p", "Reply with exactly: OK", "--model", model,
                "--max-turns", "1"]

    def quota_window_open(self, returncode: int, text: str) -> bool:
        return returncode == 0 and "limit" not in text.lower()

    def matches_quota_anomaly(self, text: str) -> bool:
        """Phrases only, not bare substrings: "joint limit exceeded" is a
        robot error, not a quota wall."""
        low = text.lower()
        return any(k in low for k in (*QUOTA_PHRASES, "quota"))

    def read_rate_limits(self, transcript: Path) -> list[dict]:
        """Every quota reading the CLI wrote into a transcript, in file order.

        The CLI streams its own rate-limit state as ``rate_limit_event``
        records: one lands at session start and more as the state changes,
        so every trial carries at least one reading of each live window.
        Reading them costs nothing extra and needs no credentials, and
        each reading belongs to a known trial rather than to an instant.

        Normalized to the shape consumers speak, so no caller has to know the
        CLI's own spelling: {window, utilization, resets_at, status, at}.
        ``window`` is "five_hour" / "seven_day"; ``utilization`` is a
        fraction of the allowance, not a percentage.

        ``at`` is when the reading was taken (unix seconds), or None. The
        event record carries no clock of its own, so it borrows the nearest
        neighbouring record that has one. Without it a reading would be dated
        by when someone read the file: the last reading of a trial sits at
        71% of the way through by median, but a tenth of trials carry only
        the opening one, which is then as old as the whole trial.

        Faithful and unreduced: picking the latest, or the latest per window,
        is the consumer's policy (orchestration/quota_ledger.py). Rejections
        are classification evidence and belong to scan_transcript(); this
        function reports them too, so history stays complete.
        """
        out: list[dict] = []
        pending: list[dict] = []   # events seen before any clock appeared
        last_at: float | None = None
        for rec in _iter_records(transcript):
            at = _ts(rec.get("timestamp"))
            if at is not None:
                last_at = at
                for e in pending:      # the first clock also dates what preceded it
                    e["at"] = at
                pending.clear()
            if rec.get("type") != "rate_limit_event":
                continue
            info = rec.get("rate_limit_info") or {}
            entry = {"window": info.get("rateLimitType"),
                     "utilization": info.get("utilization"),
                     "resets_at": info.get("resetsAt"),
                     "status": info.get("status"),
                     "at": last_at}
            if last_at is None:
                pending.append(entry)
            out.append(entry)
        return out


    # --------------------------------------------------- transcript accounting





    def read_final(self, transcript: Path) -> dict:
        """The trial's totals from the CLI's final result record(s): {num_turns,
        hit_max_turns, usage, cost_usd, duration_ms, segments}. {} when
        unreadable or absent. These land in operator_meta so cost and
        token statistics read straight from result.json.

        A trial suspended at a quota wall and resumed writes one result
        record per segment into the same transcript, so the totals sum
        across them. ``segments`` says how many there were; it is 1 for an
        uninterrupted trial, whose numbers are unchanged by this. Whether
        the trial ended on its turn budget is the last segment's verdict,
        not a sum.

        The result record is usually a segment's last line but not always:
        a session that spawned background tasks gets trailing ``system``
        task notifications appended after it, so every ``type: "result"``
        is collected rather than trusting position.
        """
        results = [rec for rec in _iter_records(transcript)
                   if rec.get("type") == "result"]
        if not results:
            return {}
        usage: dict = {}
        for rec in results:
            for k, v in (rec.get("usage") or {}).items():
                if k in _USAGE_SUMS:
                    # Numbers only: this function promises {} on an unreadable
                    # transcript, so a damaged usage field must not raise.
                    usage[k] = usage.get(k, 0) + (v if isinstance(v, (int, float))
                                                  else 0)
                else:
                    usage.setdefault(k, v)   # non-additive (service_tier, ...)
        return {"num_turns": _sum_or_none(results, "num_turns"),
                "hit_max_turns": results[-1].get("subtype") == "error_max_turns",
                "usage": usage or None,
                "cost_usd": _sum_or_none(results, "total_cost_usd"),
                "duration_ms": _sum_or_none(results, "duration_ms"),
                "segments": len(results)}




    def quota_since(self, transcript: Path, line: int = 0) -> dict | None:
        """Quota evidence written after ``line``, or None. ``line`` is how many
        lines the transcript already had; 0 scans the whole file.

        scan_transcript()'s quota verdict is sticky on purpose: a trial
        that was ever rejected is voided. That is right for post-hoc
        classification and wrong for the runner deciding whether to wait:
        after a wall is waited out and the trial resumes and finishes, the
        sticky verdict still reports the old rejection, and a runner
        reading it would suspend a finished trial again and again.

        The caller supplies the boundary rather than this function inferring
        one. A runner knows exactly how long the file was before it launched a
        segment; inferring the boundary from record shapes (the CLI's own
        ``system``/``init``, say) would make a correctness-critical decision
        depend on the CLI continuing to emit that record on every resume.
        """
        try:
            lines = transcript.read_text(errors="replace").splitlines()
        except OSError:
            return None
        for i in range(max(0, line), len(lines)):
            try:
                rec = json.loads(lines[i])
            except json.JSONDecodeError:
                continue
            hit = _rejection(rec, i + 1, transcript.name)
            if hit:
                return hit
            # A weekly limit carries no rejection event, only a quota
            # phrase on an error final result. Phrases are scanned only
            # on error results.
            if rec.get("type") == "result" and rec.get("is_error"):
                text = str(rec.get("result", ""))
                if any(p in text.lower() for p in QUOTA_PHRASES):
                    return {"evidence": f"{transcript.name}:{i + 1}:result-text",
                            "message": text[:160], "resets_at": None}
        return None


    def scan_transcript(self, transcript: Path) -> dict:
        """Quota and infrastructure evidence for post-hoc classification:

        - quota: rejected rate-limit evidence ({evidence, resets_at, ...})
          from the CLI's rate_limit_event, or a quota phrase on an error
          final result. Phrases are scanned only on error results: a
          normal final record's text is the agent's own closing message,
          and robotics prose plausibly contains quota wording.
        - has_final_result / final_is_error / final_text
        - api_transport_error: the API transport killed the session
          ("API Error: ...").
        - logged_out_launch: the CLI launched logged-out.
        - lines / malformed_lines / read_error
        """
        ev: dict = {"quota": None, "malformed_lines": 0, "lines": 0,
                    "has_final_result": False, "api_transport_error": False,
                    "logged_out_launch": False}
        try:
            raw = transcript.read_text(errors="replace")
        except OSError as e:
            ev["read_error"] = str(e)
            return ev
        for i, line in enumerate(raw.splitlines()):
            ev["lines"] += 1
            if not line.strip():
                continue
            try:
                rec = json.loads(line)
            except json.JSONDecodeError:
                ev["malformed_lines"] += 1
                continue
            rtype = rec.get("type")
            if rtype == "rate_limit_event":
                hit = _rejection(rec, i + 1, transcript.name)
                if hit:
                    ev["quota"] = hit
            elif rtype == "result":
                ev["has_final_result"] = True
                text = str(rec.get("result", ""))
                ev["final_is_error"] = bool(rec.get("is_error"))
                ev["final_text"] = text[:200]
                # The transport and login verdicts follow the last result
                # record; a later clean result resets them (a concatenated
                # transcript may carry several). Matched on the 200-char
                # window. The quota latch below is sticky instead: ever
                # rejected voids the trial, matched on the full text.
                low200 = ev["final_text"].lower()
                ev["api_transport_error"] = (
                    ev["final_is_error"] and "api error" in low200)
                ev["logged_out_launch"] = (
                    ev["final_is_error"] and "not logged in" in low200)
                if (ev["quota"] is None and ev["final_is_error"]
                        and any(p in text.lower() for p in QUOTA_PHRASES)):
                    ev["quota"] = {
                        "evidence": f"{transcript.name}:{i + 1}:result-text",
                        "message": text[:160],
                    }
        return ev



    def assistant_turns_before(self, transcript: Path, wall_unix: float) -> int | None:
        """Agent turns completed at or before an instant (post-hoc turn
        budgets)."""
        if not transcript.exists() or wall_unix is None:
            return None
        n = 0
        for rec in _iter_records(transcript):
            if rec.get("type") != "assistant":
                continue
            t = _ts(rec.get("timestamp"))
            if t is not None and t <= wall_unix:
                n += 1
        return n

    # ------------------------------------------------ action extraction

    def replay_ops(self, transcript: Path) -> list[dict]:
        """Bash (executed) and Write/Edit (file materialization; agents keep
        scripts in the sandbox's /tmp, which is not in the archived
        workspace) in order, normalized to shell/write/edit operations.

        The transcript is written streaming: one long command appears as
        several snapshots of the same tool_use id, each a bit longer, the
        last one complete. Each id is replayed exactly once, with its
        final input, at its first position."""
        results: dict[str, dict] = {}
        order: list[dict] = []
        for rec in _iter_records(transcript):
            if rec.get("type") == "user":
                content = rec.get("message", {}).get("content")
                if isinstance(content, list):
                    for b in content:
                        if isinstance(b, dict) and b.get("type") == "tool_result":
                            txt = b.get("content")
                            if isinstance(txt, list):
                                txt = "\n".join(x.get("text", "") for x in txt
                                                if isinstance(x, dict))
                            results[b.get("tool_use_id")] = {
                                "text": str(txt or ""), "ts": _ts(rec.get("timestamp"))}
            elif rec.get("type") == "assistant":
                for b in rec.get("message", {}).get("content", []):
                    if isinstance(b, dict) and b.get("type") == "tool_use" \
                            and b.get("name") in _REPLAY_TOOLS:
                        order.append({"id": b.get("id"), "tool": b["name"],
                                      "input": b.get("input", {})})
        seen: dict = {}
        deduped: list[dict] = []
        for op in order:
            if op["id"] in seen:
                seen[op["id"]].update(tool=op["tool"], input=op["input"])
            else:
                seen[op["id"]] = op
                deduped.append(op)
        ops: list[dict] = []
        prev_ts: float | None = None
        for op in deduped:
            res = results.get(op["id"], {})
            ts = res.get("ts")
            dur = (ts - prev_ts) if (ts and prev_ts) else None
            prev_ts = ts or prev_ts
            norm = _normalize(op["tool"], op["input"])
            if norm is None:
                continue
            ops.append({**norm, "output": res.get("text", ""),
                        "duration_s": max(1.0, min(dur, 600.0)) if dur else 30.0})
        return ops


def _normalize(tool: str, inp: dict) -> dict | None:
    """Claude's tool names -> the adapter-neutral operation shapes."""
    if tool == "Bash":
        return {"kind": "shell", "command": inp.get("command", "")} if inp.get("command") else None
    if tool == "Write":
        return {"kind": "write", "path": inp.get("file_path", ""),
                "content": inp.get("content", "")}
    if tool == "Edit":
        return {"kind": "edit", "path": inp.get("file_path", ""),
                "old": inp.get("old_string", ""), "new": inp.get("new_string", ""),
                "replace_all": bool(inp.get("replace_all"))}
    return None


# ----------------------------------------------------- record helpers

_USAGE_SUMS = ("input_tokens", "output_tokens",
               "cache_creation_input_tokens", "cache_read_input_tokens")


def _iter_records(transcript: Path):
    """Parsed stream-json records, tolerating malformed/truncated lines
    (they are evidence of interruption, not reasons to crash)."""
    for line in transcript.read_text(errors="replace").splitlines():
        if not line.strip():
            continue
        try:
            yield json.loads(line)
        except json.JSONDecodeError:
            continue

def _sum_or_none(records: list[dict], key: str):
    """Sum of a key across segments, or None when no segment reported it
    (so a missing field stays missing rather than becoming a fake 0)."""
    vals = [r.get(key) for r in records
            if isinstance(r.get(key), (int, float))]
    return sum(vals) if vals else None

def _rejection(rec: dict, lineno: int, fname: str) -> dict | None:
    """The quota evidence in one record, or None. Shared by the whole-file
    scan and the last-segment scan so the two cannot drift apart."""
    if rec.get("type") != "rate_limit_event":
        return None
    info = rec.get("rate_limit_info") or {}
    if info.get("status") != "rejected":
        return None
    return {"evidence": f"{fname}:{lineno}:rate_limit_event",
            "rate_limit_type": info.get("rateLimitType"),
            "resets_at": info.get("resetsAt")}

def _ts(s: str | None) -> float | None:
    if not s:
        return None
    try:
        return datetime.fromisoformat(s.replace("Z", "+00:00")).timestamp()
    except ValueError:
        return None


HOOKS = ClaudeCode
