"""What every agent adapter must satisfy, as a test anyone can run.

A third-party adapter proves itself with one call from its own tests::

    from robocli.testing import check_agent
    from my_agent import AGENT

    def test_conforms():
        check_agent(AGENT)

``check_agent`` raises AssertionError naming the first violation; the
bundled adapters run through the same function in RoboCLI's own suite.
Leaf module: imports only the adapter base.
"""

from __future__ import annotations

from pathlib import Path

from robocli.agents.base import HOOKS, Agent, Credentials


def check_agent(agent: Agent) -> None:
    """Assert the adapter contract (see robocli/agents/base.py)."""
    assert isinstance(agent, Agent), "AGENT must be an robocli.agents.Agent instance"
    cls = type(agent)
    # identity
    assert agent.name and agent.name.replace("-", "_").isidentifier(), \
        f"name {agent.name!r} must be a bare identifier (dashes allowed)"
    assert agent.name == agent.name.lower(), "name is lowercase"
    assert agent.default_model, "default_model is required"
    # declared facts, right types
    assert isinstance(agent.install, str), "install is a shell string ('' = nothing)"
    assert '"' not in agent.install and "\n" not in agent.install.strip(), \
        "install is one line without double quotes (docker build-arg contract)"
    assert isinstance(agent.whitelist, tuple) and all(isinstance(x, str) for x in agent.whitelist), \
        "whitelist is a tuple of regex strings"
    assert agent.credentials is None or isinstance(agent.credentials, Credentials)
    assert agent.token_env is None or isinstance(agent.token_env, str)
    assert agent.version_argv is None or (isinstance(agent.version_argv, tuple) and agent.version_argv)
    assert isinstance(agent.default_options, dict)
    # the required method
    assert cls.launch_argv is not Agent.launch_argv, "launch_argv must be implemented"
    argv = agent.launch_argv("box", "task text", agent.default_model, 3,
                             "http://wall:8888", options=dict(agent.default_options),
                             session_id="sid", resume=False, token_file=None)
    assert isinstance(argv, list) and argv and all(isinstance(x, str) for x in argv), \
        "launch_argv returns a non-empty list of strings"
    assert argv[:2] == ["docker", "exec"], "launch_argv is a docker exec into the sandbox"
    assert "box" in argv, "launch_argv names the sandbox container"
    if agent.token_env:
        with_token = agent.launch_argv("box", "t", agent.default_model, 1,
                                       "http://w", options={}, token_file="/f")
        assert "--env-file" in with_token and "/f" in with_token, \
            "a token_env adapter hands the token file to docker --env-file"
    # generic behaviour derived from declarations
    mounts = agent.sandbox_mounts(Path("/cfg"), Path("/creds"))
    assert isinstance(mounts, tuple)
    if agent.credentials:
        assert any(m.endswith(f":{agent.credentials.mount_point}") for m in mounts)
    # every hook keeps its documented default unless overridden
    p = Path("/nonexistent/transcript.jsonl")
    if "interactive_argv" not in agent.capabilities:
        assert agent.interactive_argv("box", "m", "http://w") is None
    if "read_final" not in agent.capabilities:
        assert agent.read_final(p) == {}
    if "read_rate_limits" not in agent.capabilities:
        assert agent.read_rate_limits(p) == []
    if "replay_ops" not in agent.capabilities:
        assert agent.replay_ops(p) == []
    if "quota_since" not in agent.capabilities:
        assert agent.quota_since(p) is None
    if "quota_window_open" not in agent.capabilities:
        assert agent.quota_window_open(1, "anything") is True
    # capabilities are derived, never declared
    assert agent.capabilities == frozenset(
        h for h in HOOKS if getattr(cls, h) is not getattr(Agent, h))
    # hooks that exist return the right shape
    hint = agent.login_hint(Path("/home/x"))
    assert isinstance(hint, str) and hint
    for check in (agent.credentials_check(), agent.sandbox_cli_check()):
        assert check is None or (isinstance(check, tuple) and len(check) == 2
                                 and all(isinstance(x, str) for x in check)), \
            "prechecks are (name, bash) pairs or None"
