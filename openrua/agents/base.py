"""The agent contract: what OpenRUA needs to know about one coding agent.

An ``Agent`` is composed by the registry from two halves:

- the manifest (``configs/agents/<name>.yaml``): the facts, set here as
  plain attributes: name, default model, the shell that installs the
  CLI into the sandbox image, the hosts it talks to, how it logs in;
- the hooks module (``plugins/agents/<hooks>.py``, exposing ``HOOKS``,
  a subclass of this class): the behaviour. One method is required,
  ``launch_argv``, the docker-exec command that runs the agent headless
  on a task. Every other hook has a documented default; a consumer that
  finds the default does without (no interactive mode, no quota
  bookkeeping, no replay). ``capabilities`` says which hooks a class
  implements, so ``openrua agents`` and ``doctor`` can list them.

Hooks take ``**_`` so the harness can pass new keyword arguments without
breaking older hooks modules. ``openrua.testing.check_agent`` is the
conformance test.

Leaf module: imports nothing from openrua.
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Any


@dataclass(frozen=True)
class Credentials:
    """How a CLI keeps its login, when it keeps one in a profile directory.

    ``dirname``: the profile's directory name under ``~/.openrua/credentials/``.
    ``filename``: the file inside it that holds the rotating secret. It is
    bind-mounted as ONE shared file into every sandbox (OAuth refresh
    tokens rotate; copies kill each other), the rest of the profile is a
    fresh per-sandbox copy.
    ``config_env``: the environment variable the CLI reads its profile
    directory from.
    ``mount_point``: where the profile lands inside the sandbox.
    """
    dirname: str
    filename: str
    config_env: str
    mount_point: str


# The optional hooks, in the order they are documented below. An agent
# "has" a capability when its hooks class overrides the hook.
HOOK_NAMES = (
    "interactive_argv", "sandbox_cli_check", "login_hint", "token_hint",
    "quota_probe_argv", "quota_window_open", "matches_quota_anomaly",
    "read_rate_limits", "quota_since", "read_final", "scan_transcript",
    "assistant_turns_before", "replay_ops",
)


class Agent:
    """The contract. The manifest's fields arrive as keyword arguments;
    a hooks module subclasses this and implements ``launch_argv``."""

    # ---- required ---------------------------------------------------
    name: str = ""                      # agent name (``agent.name`` in configs)
    default_model: str = ""             # model when the config names none

    # ---- manifest facts, all with defaults --------------------------
    binary: str = ""                    # the CLI executable inside the sandbox
    version: str | None = None          # the CLI version the install line pins
    install: str = ""                   # one-line root shell chain that installs the
                                        # CLI into the sandbox image ("" = nothing)
    whitelist: tuple[str, ...] = ()     # hostname regexes the CLI must reach through
                                        # the proxy (tinyproxy filter syntax)
    credentials: Credentials | None = None   # profile-directory login, or None
    token_env: str | None = None        # env var carrying a long-lived token, when
                                        # the CLI accepts one (passed by file)
    version_argv: tuple[str, ...] | None = None   # prints the CLI version (provenance)
    instruction_file: str | None = None  # the instructions file this CLI reads on
                                         # its own at start, if any; declared only,
                                         # nothing is seeded from it yet
    default_options: dict[str, Any] = {}  # knobs a config may override under
                                          # ``agent.options``

    def __init__(self, **fields: Any) -> None:
        for k, v in fields.items():
            if not hasattr(type(self), k):
                raise TypeError(f"{type(self).__name__} has no attribute {k!r}")
            setattr(self, k, v)
        if not self.name or not self.default_model:
            raise TypeError(f"{type(self).__name__}: name and default_model are required")

    # ---- the one required method ------------------------------------
    def launch_argv(self, sandbox: str, prompt: str, model: str, max_turns: int,
                    proxy: str, options: dict[str, Any] | None = None,
                    session_id: str | None = None, resume: bool = False,
                    token_file: str | None = None, **_: Any) -> list[str]:
        """The docker-exec command that runs the agent headless in
        ``sandbox`` on ``prompt``, as the ``robot`` user in /workspace,
        with ``proxy`` as its only route out, writing its transcript to
        stdout. ``options`` are the merged adapter knobs. ``session_id``
        names the session so ``resume=True`` can continue it later.
        ``token_file`` is a host file holding ``<token_env>=<token>`` for
        docker's ``--env-file``."""
        raise NotImplementedError(f"{self.name}: launch_argv is required")

    # ---- generic behaviour derived from the declarations ------------
    @staticmethod
    def exec_argv(sandbox: str, env: list[str] = (), token_file: str | None = None,
                  interactive: bool = False) -> list[str]:
        """The ``docker exec`` prefix every hook shares: the ``robot`` user
        in /workspace, ``env`` as ``-e`` pairs already rendered, the
        token file handed to the process by ``--env-file``, a terminal
        when ``interactive``. Append the agent's own command."""
        return [
            "docker", "exec", *(["-it"] if interactive else []),
            "-u", "robot", "-w", "/workspace",
            *(["--env-file", token_file] if token_file else []),
            *env,
            sandbox,
        ]

    def sandbox_mounts(self, config_dir: Path,
                       credentials_file: Path | None = None) -> tuple[str, ...]:
        """SRC:DST mounts for the sandbox: the per-sandbox profile copy
        at ``credentials.mount_point`` and, when one is given, the shared
        credentials file inside it. Empty for an adapter without a
        profile-directory login."""
        if self.credentials is None:
            return ()
        c = self.credentials
        mounts = (f"{config_dir}:{c.mount_point}",)
        if credentials_file is None:
            return mounts
        return mounts + (f"{credentials_file}:{c.mount_point}/{c.filename}",)

    def credentials_check(self) -> tuple[str, str] | None:
        """(name, bash) preflight check that the mounted credentials file is
        readable by the sandbox user. None without a profile login."""
        if self.credentials is None:
            return None
        f = f"{self.credentials.mount_point}/{self.credentials.filename}"
        return ("credentials_readable", f"bash -c '[ ! -e {f} ] || test -r {f}'")

    @property
    def capabilities(self) -> frozenset[str]:
        """The hooks this agent's class implements (overrides)."""
        return frozenset(h for h in HOOK_NAMES
                         if getattr(type(self), h) is not getattr(Agent, h))

    # ---- optional hooks; each docstring states the default ----------
    def interactive_argv(self, sandbox: str, model: str, proxy: str,
                         options: dict[str, Any] | None = None,
                         prompt: str | None = None, **_: Any) -> list[str] | None:
        """docker-exec command that opens the agent interactively in the
        sandbox (a person at the keyboard). Default: None, meaning
        ``openrua agent`` refuses with "no interactive mode"."""
        return None

    def sandbox_cli_check(self) -> tuple[str, str] | None:
        """(name, bash) preflight check run inside the sandbox before the
        agent starts (a version pin, say). Default: None, no check."""
        return None

    def login_hint(self, creds_home: Path) -> str:
        """One line telling the operator how to log this CLI in with
        ``creds_home`` as its profile directory. Default: a generic line."""
        return (f"log the {self.name} CLI in with its profile directory "
                f"set to {creds_home}")

    def token_hint(self, token_file: Path | str) -> str | None:
        """How to mint the token file ``launch_argv`` expects. Default:
        None (this CLI takes no token file)."""
        return None

    def quota_probe_argv(self, model: str) -> list[str] | None:
        """A minimal in-sandbox request whose outcome says whether the
        account's quota window is open (see ``quota_window_open``).
        Default: None, no probe; callers treat the window as open."""
        return None

    def quota_window_open(self, returncode: int, text: str) -> bool:
        """Judge a probe's result. Default: True."""
        return True

    def matches_quota_anomaly(self, text: str) -> bool:
        """Does a runner anomaly string look like a quota wall? Default: False."""
        return False

    def read_rate_limits(self, transcript: Path) -> list[dict]:
        """Every quota reading the CLI wrote into a transcript, normalized
        to ``{window, utilization, resets_at, status, at}``. Default: []."""
        return []

    def quota_since(self, transcript: Path, line: int = 0) -> dict | None:
        """Quota-wall evidence written after ``line`` (``{evidence,
        resets_at, ...}``), or None. Default: None, never a wall; the
        runner then never suspends a trial."""
        return None

    def read_final(self, transcript: Path) -> dict:
        """The trial's totals from the transcript: ``{num_turns,
        hit_max_turns, usage, cost_usd, duration_ms, segments}``. Default:
        {}, nothing known; the runner then counts turns as unknown."""
        return {}

    def scan_transcript(self, transcript: Path) -> dict | None:
        """Classification evidence (``quota``, ``has_final_result``,
        ``final_is_error``, ``api_transport_error``, ...). Default: None."""
        return None

    def assistant_turns_before(self, transcript: Path, wall_unix: float) -> int | None:
        """Agent turns completed at or before an instant (post-hoc turn
        budgets). Default: None, unknown."""
        return None

    def replay_ops(self, transcript: Path) -> list[dict]:
        """The agent's world-facing operations in order, each one of
        ``{kind: "shell", command}``, ``{kind: "write", path, content}`` or
        ``{kind: "edit", path, old, new, replace_all}``, plus ``output``,
        ``duration_s`` and the wall times ``t0``/``t1`` (unix seconds,
        None when the transcript has none). Default: [] (no replay)."""
        return []
