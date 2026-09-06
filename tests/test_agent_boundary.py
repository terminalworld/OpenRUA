"""Agent-knowledge boundary.

Every fact about a specific coding agent lives in its manifest
(``robocli/configs/agents/``) and its hooks module
(``robocli/plugins/agents/``): launch command, auth layout, transcript
format, quota wording. This test walks every other file in the package
and fails on any agent-specific token, so swapping the agent never means
editing the harness: consumers touch only the ``Agent`` contract.

Test fixtures (test_*.py) are exempt; they fabricate agent-shaped
transcripts on purpose.
"""

from __future__ import annotations

from pathlib import Path

PKG = Path(__file__).resolve().parents[1] / "robocli"
HOOKS_DIR = PKG / "plugins" / "agents"

# Case-insensitive substrings that mark agent-specific knowledge. The
# quoted 'assistant' forms catch transcript-record-type literals without
# banning the English word in comments.
TOKENS = (
    "claude",
    "anthropic",
    ".credentials.json",
    "rate_limit_event",
    "error_max_turns",
    "stream-json",
    "session limit",
    "rate limit",
    "weekly limit",
    "usage limit",
    "tool_use",
    "tool_result",
    "'assistant'",
    '"assistant"',
)


def _scanned_files():
    for p in sorted(PKG.rglob("*.py")):
        if HOOKS_DIR in p.parents or "__pycache__" in p.parts:
            continue
        yield p
    # The sandbox and proxy images are agent-parameterized via --build-arg
    # (install line and whitelist come from the manifests): the recipes
    # themselves stay agent-free too.
    for f in ("sandbox/sandbox.Dockerfile", "proxy/proxy.Dockerfile",
              "proxy/tinyproxy.conf"):
        yield PKG / f


def test_agent_knowledge_stays_in_manifests_and_hooks():
    leaks = []
    for p in _scanned_files():
        text = p.read_text(errors="replace").lower()
        for i, line in enumerate(text.splitlines(), 1):
            for tok in TOKENS:
                if tok in line:
                    leaks.append(f"{p.relative_to(PKG.parent)}:{i}: {tok!r}")
    assert not leaks, (
        f"agent-specific knowledge outside plugins/agents ({len(leaks)} leaks):\n"
        + "\n".join(leaks))
