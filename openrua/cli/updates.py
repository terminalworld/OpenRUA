"""Offer newer releases at start, as coding agents' own terminals do.

Two things can be out of date: OpenRUA itself, and the agent CLI in the
sandbox image. At most once a day each one's newest release is read
(OpenRUA's from PyPI, the agent's from its manifest's
``latest_release``); when what is installed is older, ``openrua`` asks:
update now, skip, or skip until the next version. Updating OpenRUA runs
the upgrade its installer uses (pip, uv or pipx), and the new version
takes effect when ``openrua`` is run again; updating the agent CLI
rebuilds the sandbox image (the layers below the CLI are reused). An
agent CLI older than the adapter's ``min_version`` cannot run a session,
so then the choice is update or quit. Commands meant for scripts (up,
exec) print one line and never ask. OpenRUA installed from a source
checkout is updated with git and is never offered one; a pinned agent
version (``agent.version``) is never offered one either.
"""

from __future__ import annotations

import json
import subprocess
import sys
import urllib.request
from dataclasses import dataclass
from datetime import datetime, timedelta, timezone
from importlib import metadata
from pathlib import Path

from openrua import agents
from openrua.config import paths

CHECK_EVERY = timedelta(days=1)
PACKAGE = "openrua"
PACKAGE_RELEASES = "https://pypi.org/pypi/openrua/json"   # newest as info.version


@dataclass(frozen=True)
class Offer:
    name: str               # what is offered: openrua, or an agent's name
    installed: str
    latest: str | None      # None when only the minimum is known
    required: bool          # older than min_version: no session without it
    update: list[str]       # the command that updates it
    restart: bool           # the update takes effect in the next run of openrua
    title: str = ''
    target: str | None = None


def _numbers(text: str | None) -> tuple[int, ...] | None:
    found = agents.version_from_output(text)
    return tuple(int(x) for x in found.split('.')) if found else None


def installed_version(agent, image: str) -> str | None:
    """The CLI version the image carries, as the CLI prints it."""
    probe = agent.version_probe()
    if not probe:
        return None
    try:
        r = subprocess.run(["docker", "run", "--rm", "--entrypoint", "", image,
                            "bash", "-c", probe], capture_output=True, text=True, timeout=60)
    except (OSError, subprocess.TimeoutExpired):
        return None
    return r.stdout.strip() or None


def _cache(home: Path | None) -> Path:
    return paths.home(home) / "update-check.json"


def _read(home: Path | None) -> dict:
    try:
        return json.loads(_cache(home).read_text())
    except (OSError, ValueError):
        return {}


def _write(home: Path | None, data: dict) -> None:
    path = _cache(home)
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(data, indent=2) + "\n")


def _latest(name: str, url: str | None, home: Path | None, now: datetime | None,
            field=lambda body: body.get("version")) -> str | None:
    """``name``'s newest release from ``url``, read at most once a day;
    None when unknown (no URL, no network): then nothing is offered."""
    if not url:
        return None
    now = now or datetime.now(timezone.utc)
    data = _read(home)
    seen = data.get(name, {})
    try:
        fresh = now - datetime.fromisoformat(seen["checked"]) < CHECK_EVERY
    except (KeyError, ValueError):
        fresh = False
    if fresh:
        return seen.get("latest")
    try:
        with urllib.request.urlopen(url, timeout=5) as answer:
            latest = field(json.load(answer))
    except (OSError, ValueError, KeyError, TypeError):
        return seen.get("latest")
    data[name] = {**seen, "checked": now.isoformat(), "latest": latest}
    _write(home, data)
    return latest


def latest_version(agent, home: Path | None, now: datetime | None = None) -> str | None:
    return _latest(agent.name, agent.latest_release, home, now)


def skip_version(home: Path | None, name: str, version: str) -> None:
    """Do not offer ``version`` of ``name`` again; a later one is offered."""
    data = _read(home)
    data[name] = {**data.get(name, {}), "skipped": version}
    _write(home, data)


def _offer(name: str, have: str | None, latest: str | None, minimum: str | None,
           home: Path | None, update: list[str], restart: bool) -> Offer | None:
    have_n = _numbers(have)
    if have_n is None:
        return None
    required = _numbers(minimum) is not None and have_n < _numbers(minimum)
    newer = _numbers(latest) is not None and _numbers(latest) > have_n
    if not required and (not newer or _read(home).get(name, {}).get("skipped") == latest):
        return None
    return Offer(name, have, latest if newer else None, required, update, restart,
                 'OpenRUA' if restart else name, latest if newer else minimum)


def upgrade_command() -> list[str] | None:
    """How this OpenRUA was installed, as the command that upgrades it;
    None for a source checkout (git updates it) or an unknown install."""
    try:
        dist = metadata.distribution(PACKAGE)
    except metadata.PackageNotFoundError:
        return None
    try:
        origin = json.loads(dist.read_text("direct_url.json") or "{}")
    except ValueError:
        origin = {}
    if origin.get("dir_info") is not None:
        return None
    prefix = Path(sys.prefix).as_posix()
    if "/uv/tools/" in prefix:
        return ["uv", "tool", "upgrade", PACKAGE]
    if "/pipx/venvs/" in prefix:
        return ["pipx", "upgrade", PACKAGE]
    return [sys.executable, "-m", "pip", "install", "-U", PACKAGE]


def openrua_offer(home: Path | None, now: datetime | None = None) -> Offer | None:
    """A newer OpenRUA on PyPI, if this install is one that upgrades from there."""
    command = upgrade_command()
    if command is None:
        return None
    latest = _latest(PACKAGE, PACKAGE_RELEASES, home, now,
                     field=lambda body: body["info"]["version"])
    return _offer(PACKAGE, metadata.version(PACKAGE), latest, None, home, command, True)


def offer_for(composed, home: Path | None, now: datetime | None = None) -> Offer | None:
    """Offer an explicit CLI release while retaining its existing tool environment."""
    selected = composed.cfg["agent"]
    agent = agents.get(selected["name"], home, version=selected.get("version"))
    if agent.version:
        return None
    image = composed.cfg["machine"]["backend"]["sandbox_image"]
    latest = latest_version(agent, home, now)
    target = latest if _numbers(latest) and (not _numbers(agent.min_version) or _numbers(latest) >= _numbers(agent.min_version)) else agent.min_version
    if not target:
        return None
    command = [sys.executable, "-m", "openrua", *(["--home", str(home)] if home else []),
               'update', '--agent', selected['name'] + '@' + target, '--image', image]
    offer = _offer(agent.name, installed_version(agent, image), latest,
                   agent.min_version, home, command, False)
    if offer is not None:
        from dataclasses import replace
        return replace(offer, title=agent.title or agent.name, target=target)
    return None


def describe(offer: Offer) -> str:
    if offer.restart:
        return f"OpenRUA {offer.latest} is available (you have {offer.installed})."
    if offer.required:
        return (f"{offer.name} in the sandbox is {offer.installed}, older than this "
                f"OpenRUA can run; update this agent before starting.")
    return (f"{offer.name} {offer.latest} is available (the sandbox has "
            f"{offer.installed}); compatible runtime dependencies are retained.")


def _shown(command: list[str]) -> str:
    """The command as a person would type it."""
    if command[:3] == [sys.executable, "-m", "openrua"]:
        return " ".join(["openrua", *command[3:]])
    if command[:3] == [sys.executable, "-m", "pip"]:
        return " ".join(["pip", *command[3:]])
    return " ".join(command)


def ask(offer: Offer, home: Path | None, read=input, show=print) -> bool:
    """Ask in the terminal; True to go on with the session. Update now
    runs the update (and, for OpenRUA itself, stops so the new version
    runs next time); skip goes on; skip until the next version also
    remembers this one."""
    show(describe(offer))
    if offer.required:
        show("  1. Update now\n  2. Quit")
        if (read("Choose [1/2]: ").strip() or "1") != "1":
            return False
        return update(offer, show)
    show("  1. Update now\n  2. Skip\n  3. Skip until the next version")
    choice = read("Choose [1/2/3, Enter = 2]: ").strip() or "2"
    if choice == "1":
        return update(offer, show)
    if choice == "3" and offer.latest:
        skip_version(home, offer.name, offer.latest)
    return True


def update(offer: Offer, show=print) -> bool:
    """Run the update; True to go on with the session."""
    show(f"[update] {_shown(offer.update)}")
    code = subprocess.call(offer.update)
    if offer.restart:
        show(f"[update] OpenRUA {offer.latest} is installed: run openrua again"
             if code == 0 else f"[update] the upgrade failed (exit {code}); this "
             f"OpenRUA ({offer.installed}) still works")
        return code != 0
    if code:
        show(f"[update] the agent update failed (exit {code}); the previous image is unchanged")
    return code == 0 or not offer.required


def offers(composed, home: Path | None) -> list[Offer]:
    """Everything there is to offer before a session, OpenRUA first (a new
    OpenRUA may bring what a new agent CLI needs)."""
    found = []
    for check in (lambda: openrua_offer(home),
                  lambda: offer_for(composed, home) if composed is not None else None):
        try:
            offer = check()
        except Exception:  # noqa: BLE001; an update check never stops a start
            offer = None
        if offer is not None:
            found.append(offer)
    return found


def notice(composed, home: Path | None, show=print) -> None:
    """One line per offer, for commands that never ask."""
    for offer in offers(composed, home):
        show(f"[update] {describe(offer)} {_shown(offer.update)}")


def run_logged(offer: Offer, home: Path | None, progress) -> tuple[int, Path]:
    """Retain installer output privately and report progress without a terminal handoff."""
    import os
    import signal
    import tempfile
    directory = paths.home(home) / 'updates'
    directory.mkdir(parents=True, exist_ok=True)
    folder = Path(tempfile.mkdtemp(prefix='update-', dir=directory))
    log = folder / 'output.log'
    with open(log, 'x', encoding='utf-8', opener=lambda p, f: os.open(p, f, 0o600)) as output:
        progress('', title=f'Updating {offer.title or offer.name}', compact=True,
                 detail=f'{offer.installed} → {offer.target or offer.latest or "supported version"}',
                 summary='Starting update')
        try:
            child = subprocess.Popen(offer.update, stdin=subprocess.DEVNULL, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                                     text=True, errors='replace', start_new_session=True)
        except OSError as exc:
            output.write(str(exc) + '\n')
            return 127, log
        try:
            for line in child.stdout:
                output.write(line); output.flush()
                summary = line.removeprefix('[update] ').strip() if line.startswith('[update] ') else None
                if line.startswith('[build] '):
                    summary = 'Verifying the installed CLI'
                progress(line.rstrip(), **({'summary': summary} if summary else {}))
            return child.wait(), log
        except BaseException:
            try: os.killpg(child.pid, signal.SIGINT)
            except ProcessLookupError: pass
            try: child.wait(timeout=5)
            except subprocess.TimeoutExpired:
                os.killpg(child.pid, signal.SIGKILL); child.wait()
            raise
        finally:
            child.stdout.close()


def present(offer: Offer, home: Path | None, application) -> bool:
    """Offer, update, inspect logs and explicitly retry inside one fullscreen host."""
    spec = {'mode': 'update', 'name': offer.title or offer.name, 'installed': offer.installed,
            'target': offer.target or offer.latest or 'supported version', 'required': offer.required,
            'restart': offer.restart}
    phase, code, log = 'offer', None, None
    while True:
        response = application.screen({**spec, 'phase': phase,
            'success': code == 0 if code is not None else None,
            'log_path': str(log) if log else ''})
        action = response.get('action')
        if action in {'quit', 'close'}:
            return False
        if phase == 'offer' and action in {'continue', 'skip'}:
            if offer.required:
                return False
            if action == 'skip' and offer.latest:
                skip_version(home, offer.name, offer.latest)
            return True
        if phase == 'result' and action == 'continue':
            return not (code == 0 and offer.restart) and (code == 0 or not offer.required)
        if action == 'logs' and log is not None:
            with log.open('rb') as source:
                source.seek(max(0, log.stat().st_size - 64000))
                text = source.read().decode(errors='replace')
            viewed = application.screen({**spec, 'phase': 'logs', 'text': text, 'log_path': str(log)})
            if viewed.get('action') == 'quit':
                return False
            continue
        if (phase == 'offer' and action == 'update') or (phase == 'result' and action == 'retry' and code != 0):
            code, log = run_logged(offer, home, application.progress)
            phase = 'result'
            continue
        raise RuntimeError('Invalid software update selection; no additional update was run')
