# Agents

An agent adapter is one module under `robocli/agents/` implementing the
names in `ADAPTER_INTERFACE` (`robocli/agents/__init__.py`): identity
and defaults, the sandbox install snippet, the proxy whitelist lines,
the launch command (headless, for `robocli run`) and the interactive
seat (for `robocli agent`), the login check, and how to read its
transcript for turns, tokens and the commands it ran.

Shipped: `claude_code.py`. `codex.py` is next; a new adapter is
registered by adding its name to `ADAPTERS` and `get()`. The tests in
`tests/test_agents.py` check that every registered adapter implements
the whole interface, and that nothing outside the package names a
concrete adapter (consumers go through `agents.get()`).

Login: each adapter keeps a dedicated profile directory
(`~/.robocli-claude` for Claude Code) whose credentials file is
bind-mounted into every sandbox as one shared file. Log in once on the
host with that directory as the agent's config dir; `robocli doctor`
prints the exact command when it is missing.
