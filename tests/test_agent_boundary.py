"""Agent-knowledge boundary (decoupling ruling 2026-08-15).

EVERY fact about a specific CLI agent (Claude Code today) lives in
``robocli/agents/``: launch command, auth layout, transcript format,
quota wording. This test walks every other file in the package and
fails on any leaked agent-specific token, so "swap the agent" can never
again mean "edit ten files": consumers touch only the adapter interface.

Test fixtures (test_*.py) are exempt; they fabricate agent-shaped
transcripts on purpose, mimicking real artifacts.
"""

from __future__ import annotations

from pathlib import Path

PKG = Path(__file__).resolve().parents[1] / "robocli"
ADAPTER_DIR = PKG / "agents"

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
        if ADAPTER_DIR in p.parents or "__pycache__" in p.parts:
            continue
        # the agent-facing workspace tools talk to ROS, never to an agent
        yield p
    # The sandbox and proxy images are agent-parameterized via --build-arg
    # (install snippet / whitelist come from the adapter): the recipes
    # themselves must stay agent-free too.
    for f in ("sandbox/sandbox.Dockerfile", "proxy/proxy.Dockerfile",
              "proxy/tinyproxy.conf"):
        yield PKG / f
    # Inside the adapter package only the adapters themselves may know an
    # agent: the base class, the launcher and the front door are generic.
    # The registry (__init__.py) is exempt: naming the default adapter is
    # its job.
    yield ADAPTER_DIR / "base.py"
    yield ADAPTER_DIR / "launcher.py"
    yield ADAPTER_DIR / "__main__.py"


def test_agent_knowledge_stays_in_adapters():
    leaks = []
    for p in _scanned_files():
        text = p.read_text(errors="replace").lower()
        for i, line in enumerate(text.splitlines(), 1):
            for tok in TOKENS:
                if tok in line:
                    leaks.append(f"{p.relative_to(PKG.parent)}:{i}: {tok!r}")
    assert not leaks, (
        f"agent-specific knowledge outside the adapters ({len(leaks)} leaks):\n"
        + "\n".join(leaks))
