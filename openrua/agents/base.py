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

Leaf contract: only type annotations refer to the optional conversation contract.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path
import shutil
import shlex
import re
from typing import Any, Callable, TYPE_CHECKING

if TYPE_CHECKING:
    from openrua.agents.conversation import Conversation


def version_from_output(text: str | None) -> str | None:
    """Read a release number from a native CLI version response."""
    found = re.search(r'[0-9]+(?:[.][0-9]+)+', text or '')
    return found.group() if found else None


def saved_permissions(agent, facts: dict, state: dict | None = None) -> str | None:
    """Restore a recorded mode using its own catalog, including older identifiers."""
    if getattr(agent, 'permission_modes', None) == ():
        return None
    state = state or {}
    value = (state.get('settings') or {}).get('permissions')
    native = facts.get('permissions_format') == 'native'
    if value:
        choices = (state.get('setting_choices') or {}).get('permissions') or []
        if choices:
            native = all(row.get('value') in agent.permission_modes for row in choices)
    else:
        value = facts.get('permissions')
    if not value:
        return getattr(agent, 'default_permissions', Agent.default_permissions)
    return value if native else getattr(agent, 'legacy_permission_modes', {}).get(value, value)


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
    native_dir: str | None = None
    login: tuple[str, ...] | None = None


class NotSignedIn(RuntimeError):
    """The agent has no login where a session reads it; ``hint`` says how
    to sign in (the command line shows it as the fix)."""

    def __init__(self, message: str, hint: str = "") -> None:
        super().__init__(message)
        self.hint = hint


@dataclass(frozen=True)
class ProfileLogin:
    """Local login material availability, not online authentication or quota.

    Detail must describe the check without including credential contents.
    """
    available: bool
    detail: str


@dataclass(frozen=True)
class PreparedProfile:
    """A session-local profile and the plugin's explicit sandbox mounts.

    The directory is also passed to transcript collection. Shared credential
    paths stay outside it; the plugin decides their native layout and mounts.
    """
    directory: Path
    mounts: tuple[str, ...]
    # Custom credential formats supply current values for trial redaction.
    # Called before and after execution so rotations are included.
    read_secrets: Callable[[], list[str]] | None = field(default=None, repr=False)


@dataclass(frozen=True)
class SkillLocations:
    """Native project directory and sandbox mount for personal skill files."""
    project: str
    personal: str
    instructions: str | None = None


def _copy_profile(source: Path, dest: Path, credentials: Credentials | None,
                  require_credentials: bool, login_hint: str) -> tuple[Path, Path | None]:
    """The legacy JSON layout, also used by the package compatibility helper."""
    shared = source / credentials.filename if credentials and require_credentials else None
    resolved_dest, resolved_source = dest.resolve(), source.resolve()
    if dest.is_symlink() or resolved_dest == resolved_source or resolved_dest in resolved_source.parents:
        raise ValueError("profile destination must be separate from the native login directory")
    if shared is not None and not shared.is_file():
        raise NotSignedIn("the agent is not signed in on this computer", hint=login_hint)
    if dest.exists():
        shutil.rmtree(dest)
    dest.mkdir(parents=True, mode=0o700)
    if credentials:
        for pattern in ("*.json", ".*.json"):
            for path in source.glob(pattern):
                if path.name != credentials.filename:
                    shutil.copy2(path, dest / path.name)
    return dest, shared


def read_api_key(path: Path) -> str:
    """Read one explicitly selected raw key without including it in errors."""
    try:
        if not path.is_file():
            raise ValueError("API key path must be a regular file")
        key = path.read_text(encoding="utf-8").strip()
    except (OSError, UnicodeError):
        raise ValueError(f"cannot read API key file: {path}") from None
    if not key or any(char.isspace() for char in key) or "\x00" in key:
        raise ValueError("API key file must contain one nonempty key, without whitespace")
    return key


# The optional hooks, in the order they are documented below. An agent
# "has" a capability when its hooks class overrides the hook.
HOOK_NAMES = (
    "inspect_login", "prepare_profile", "prepare_api_profile", "interactive_argv", "conversation", "sandbox_cli_check", "login_hint", "token_hint",
    "quota_probe_argv", "quota_window_open", "matches_quota_anomaly", "model_catalog",
    "read_rate_limits", "quota_since", "read_final", "scan_transcript",
    "assistant_turns_before", "replay_ops", "collect",
)


class Agent:
    """The contract. The manifest's fields arrive as keyword arguments;
    a hooks module subclasses this and implements ``launch_argv``."""

    # ---- required ---------------------------------------------------
    name: str = ""                      # agent name (``agent.name`` in configs)
    default_model: str = ""             # model when the config names none

    # ---- manifest facts, all with defaults --------------------------
    title: str = ""                     # the agent's name as people write it ("" = name)
    binary: str = ""                    # the CLI executable inside the sandbox
    version: str | None = None          # the CLI version the install line pins
    prepare: str = ""
    prepare_check: str = ""
    install_template: str = ""          # the declared recipe before selecting a release
    install: str = ""                   # one-line root shell chain that installs the
                                        # CLI into the sandbox image ("" = nothing)
    whitelist: tuple[str, ...] = ()     # hostname regexes the CLI must reach through
                                        # the proxy (tinyproxy filter syntax)
    credentials: Credentials | None = None   # profile-directory login, or None
    skills: SkillLocations | None = None
    permission_modes: tuple[str, ...] = ()
    default_permissions: str | None = None
    legacy_permission_modes: dict[str, str] = {}
    model_selection: bool = True
    token_env: str | None = None        # env var carrying a long-lived token, when
                                        # the CLI accepts one (passed by file)
    version_argv: tuple[str, ...] | None = None   # prints the CLI version (provenance)
    min_version: str | None = None      # oldest CLI that takes this adapter's flags
    latest_release: str | None = None   # URL naming the newest release (JSON `version`)
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
        with ``proxy`` as its only route out (empty: no proxy), writing
        its transcript to stdout. ``options`` are the merged adapter
        knobs; ``limits`` the run's (see prepare_profile), passed as a
        keyword by every launch hook. ``session_id``
        names the session so ``resume=True`` can continue it later.
        ``token_file`` is a host file holding ``<token_env>=<token>`` for
        docker's ``--env-file``."""
        raise NotImplementedError(f"{self.name}: launch_argv is required")

    # ---- generic behaviour derived from the declarations ------------
    @staticmethod
    def exec_argv(sandbox: str, env: list[str] = (), token_file: str | None = None,
                  interactive: bool = False, stdin: bool = False) -> list[str]:
        """The ``docker exec`` prefix every hook shares: the ``robot`` user
        in /workspace, ``env`` as ``-e`` pairs already rendered, the
        token file handed to the process by ``--env-file``, a terminal
        when ``interactive``. Append the agent's own command."""
        return [
            "docker", "exec", *(["-it"] if interactive else ["-i"] if stdin else []),
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
    def prepare_api_profile(self, key_file: Path, dest: Path, *,
                            limits: dict | None = None) -> PreparedProfile:
        """Prepare explicitly selected API auth using this CLI's native format.

        No native login source is supplied: API mode must not load subscription
        credentials as a fallback. Do not authenticate online or start a model
        task here. Include API secrets in read_secrets for trial redaction.
        Default: unsupported, with no profile mutation.
        """
        raise ValueError(f"{self.name} does not support explicit API profiles; use native login")

    def inspect_login(self, source: Path) -> ProfileLogin | None:
        """Read-only local inspection used by discovery and doctor.

        Default: check that the manifest's credential file is readable and
        nonempty, or return None when no profile login applies. Override for
        native credential directories or other formats. Do not access the
        network, log in, refresh tokens, or select alternative credentials.
        Availability never establishes a valid subscription or API balance.
        """
        if self.credentials is None:
            return None
        credential = source / self.credentials.filename
        try:
            if not credential.is_file():
                return ProfileLogin(False, "credential file is missing or is not a regular file")
            with credential.open("rb") as stream:
                if not stream.read(1):
                    return ProfileLogin(False, "credential file is empty")
        except OSError as exc:
            return ProfileLogin(False, f"credential file cannot be read: {exc}")
        return ProfileLogin(True, "credential file is readable and nonempty")

    def prepare_profile(self, source: Path, dest: Path, *,
                        require_credentials: bool = True,
                        limits: dict | None = None) -> PreparedProfile:
        """Prepare one session's native configuration without selecting auth.

        ``limits`` (config.schema.Limits as a dict; an absent key is not
        limited) is the run's: a CLI that reads one of them from its
        profile gets it written here; the rest arrive at launch.

        Default: copy JSON settings and bind the declared credential file,
        preserving the existing profile contract. Override for other native
        layouts. The source is an explicitly resolved profile; a plugin must
        not choose another account, log in, or fall back to paid credentials.
        With require_credentials=False, authentication is supplied separately
        by the caller, and no native login credentials should be mounted.
        """
        directory, credentials = _copy_profile(source, dest, self.credentials,
                                               require_credentials, self.login_hint(source))
        return PreparedProfile(directory, self.sandbox_mounts(directory, credentials))

    def interactive_argv(self, sandbox: str, model: str, proxy: str,
                         options: dict[str, Any] | None = None,
                         prompt: str | None = None, **_: Any) -> list[str] | None:
        """docker-exec command that opens the agent interactively in the
        sandbox (a person at the keyboard). Default: None, meaning
        ``openrua agent`` refuses with "no interactive mode"."""
        return None

    def conversation(self, sandbox: str, model: str, proxy: str,
                     options: dict[str, Any] | None = None,
                     session_id: str | None = None, **_: Any) -> Conversation | None:
        """Optional structured native connection; None means unsupported.

        session_id is a native identity previously emitted by this capability,
        not a new caller-generated identifier. No process starts in this hook.
        Existing batch and terminal-only plugins need not implement it.
        """
        return None

    def model_catalog(self, sandbox: str, model: str, proxy: str, **kwargs):
        """A metadata-only native connection, or None when unsupported."""
        return None

    def version_probe(self) -> str | None:
        """Shell command printing the sandbox CLI's version, None without
        ``version_argv``. HOME points away from the workspace: a CLI may
        create its config directory under HOME when asked anything, and
        the workspace is the session's record."""
        if not self.version_argv:
            return None
        return "HOME=/tmp " + shlex.join(self.version_argv)

    def sandbox_cli_check(self) -> tuple[str, str] | None:
        """(name, bash) check run inside the sandbox before the agent
        starts: the CLI works, is the pinned version when requested, and
        is no older than ``min_version``. None without ``version_argv``."""
        if not self.version_argv:
            return None
        if not (self.version or self.min_version):
            return ('sandbox_cli_version', self.version_probe() + ' >/dev/null')
        # No single quotes inside: the whole check is one bash -c '...'.
        parts = [f'v="$({self.version_probe()} | grep -oE "[0-9]+([.][0-9]+)+" | head -1)"',
                 '[ -n "$v" ]']
        if self.version:
            parts.append(f'[ "$v" = "{self.version}" ]')
        if self.min_version:
            parts.append(f'printf "%s\\n%s\\n" {self.min_version} "$v" | sort -V -C')
        return ("sandbox_cli_version", "bash -c '" + " && ".join(parts) + "'")

    def installation_check(self) -> tuple[str, str] | None:
        """Verify the declared release as well as any plugin-specific checks."""
        checks = list(dict.fromkeys(check for check in
            (Agent.sandbox_cli_check(self), self.sandbox_cli_check()) if check))
        return ('agent_installation', ' && '.join(f'({command})' for _, command in checks)) if checks else None

    def login_hint(self, creds_home: Path) -> str:
        """One line telling the operator how to log this CLI in with
        ``creds_home`` as its profile directory. Default: ``openrua login``
        when the manifest names the CLI's sign-in (with ``--dir`` when the
        directory is not the CLI's own), else a generic line."""
        c = self.credentials
        if c is not None and c.login:
            own = c.native_dir and Path(creds_home).resolve() == (Path.home() / c.native_dir).resolve()
            return f"run openrua login {self.name}" + ("" if own else f" --dir {creds_home}")
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

    def steps(self, record: dict) -> list[dict]:
        """What one transcript record says the agent is doing, for a person
        watching it (``openrua exec``): ``{kind: "text", text}`` for what it
        says; ``{kind: "start", id, label, shell}`` when it starts a command
        or a tool (``shell``: a shell command, ``label`` the command, or the
        tool and its target; a start may repeat with a fuller label);
        ``{kind: "end", id, ok}`` when that one is done. Default: [],
        nothing shown but the run's header and its result."""
        return []

    def replay_ops(self, transcript: Path) -> list[dict]:
        """The agent's world-facing operations in order, each one of
        ``{kind: "shell", command}``, ``{kind: "write", path, content}`` or
        ``{kind: "edit", path, old, new, replace_all}``, plus ``output``,
        ``duration_s`` and the wall times ``t0``/``t1`` (unix seconds,
        None when the transcript has none). Default: [] (no replay)."""
        return []

    def collect(self, profile_dir: Path, trial_dir: Path) -> list[Path]:
        """After the run, before the sandbox's profile directory is
        discarded: keep what this agent wants from it in the trial
        directory (a CLI that writes its own session log there, say).
        Returns the files written; they get the same secret scrub as the
        transcript. The file names are this agent's alone: its reading
        hooks find them beside the transcript. Default: nothing kept."""
        return []
