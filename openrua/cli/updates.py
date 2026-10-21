"""Offer a newer agent CLI at start, as coding agents' own terminals do.

The agent CLI lives in the sandbox image, so updating it means
rebuilding that image (the layers below the CLI are reused). At most
once a day the newest release is read from the agent's
``latest_release``; when the image's CLI is older, ``openrua`` asks:
update now, skip, or skip until the next version. A CLI older than the
adapter's ``min_version`` cannot run a session, so then the choice is
update or quit. Commands meant for scripts (run, up) print one line and
never ask, and a pinned version (``agent.version``) is never offered an
update. Trials do not look: a run's CLI does not change under it.
"""

from __future__ import annotations

import json
import re
import subprocess
import sys
import urllib.request
from dataclasses import dataclass
from datetime import datetime, timedelta, timezone
from pathlib import Path

from openrua import agents
from openrua.cli.preparation import image_commands
from openrua.config import paths

CHECK_EVERY = timedelta(days=1)


@dataclass(frozen=True)
class Offer:
    agent: str
    installed: str
    latest: str | None      # None when only the minimum is known
    required: bool          # older than min_version: no session without it
    rebuild: list[str]      # openrua arguments that rebuild the sandbox


def _numbers(text: str | None) -> tuple[int, ...] | None:
    found = re.search(r"\d+(?:\.\d+)+", text or "")
    return tuple(int(x) for x in found.group().split(".")) if found else None


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


def latest_version(agent, home: Path | None, now: datetime | None = None) -> str | None:
    """The newest release, read at most once a day; None when unknown (no
    URL, no network): then nothing is offered."""
    if not agent.latest_release:
        return None
    now = now or datetime.now(timezone.utc)
    data = _read(home)
    seen = data.get(agent.name, {})
    try:
        fresh = now - datetime.fromisoformat(seen["checked"]) < CHECK_EVERY
    except (KeyError, ValueError):
        fresh = False
    if fresh:
        return seen.get("latest")
    try:
        with urllib.request.urlopen(agent.latest_release, timeout=5) as answer:
            latest = json.load(answer).get("version")
    except (OSError, ValueError):
        return seen.get("latest")
    data[agent.name] = {**seen, "checked": now.isoformat(), "latest": latest}
    _write(home, data)
    return latest


def skip_version(home: Path | None, agent: str, version: str) -> None:
    """Do not offer ``version`` again; a later one is offered."""
    data = _read(home)
    data[agent] = {**data.get(agent, {}), "skipped": version}
    _write(home, data)


def offer_for(composed, home: Path | None, now: datetime | None = None) -> Offer | None:
    """What to offer for this selection's agent, if anything."""
    selected = composed.cfg["agent"]
    agent = agents.get(selected["name"], home, version=selected.get("version"))
    if agent.version:
        return None
    image = composed.cfg["machine"]["backend"]["sandbox_image"]
    rebuild = next((command for name, command in image_commands(composed)
                    if name == image), None)
    have = installed_version(agent, image) if rebuild else None
    have_n = _numbers(have)
    if have_n is None:
        return None
    minimum = _numbers(agent.min_version)
    required = minimum is not None and have_n < minimum
    latest = latest_version(agent, home, now)
    newer = _numbers(latest) is not None and _numbers(latest) > have_n
    if not required and (not newer or _read(home).get(agent.name, {}).get("skipped") == latest):
        return None
    return Offer(agent.name, have, latest if newer else None, required, rebuild)


def describe(offer: Offer) -> str:
    if offer.required:
        return (f"{offer.agent} in the sandbox is {offer.installed}, older than this "
                f"OpenRUA can run; updating rebuilds the sandbox image.")
    return (f"{offer.agent} {offer.latest} is available (the sandbox has "
            f"{offer.installed}); updating rebuilds the sandbox image.")


def ask(offer: Offer, home: Path | None, read=input, show=print) -> bool:
    """Ask in the terminal; True to go on with the session. Update now
    rebuilds the image; skip goes on; skip until the next version also
    remembers this one."""
    show(describe(offer))
    if offer.required:
        show("  1. Update now\n  2. Quit")
        choice = (read("Choose [1/2]: ").strip() or "1")
        if choice != "1":
            return False
        return update(offer, home, show)
    show("  1. Update now\n  2. Skip\n  3. Skip until the next version")
    choice = read("Choose [1/2/3, Enter = 2]: ").strip() or "2"
    if choice == "1":
        return update(offer, home, show)
    if choice == "3" and offer.latest:
        skip_version(home, offer.agent, offer.latest)
    return True


def update(offer: Offer, home: Path | None, show=print) -> bool:
    """Rebuild the sandbox image with the current release; True on success."""
    argv = [sys.executable, "-m", "openrua", *(["--home", str(home)] if home else []),
            *offer.rebuild]
    show(f"[update] openrua {' '.join(offer.rebuild)}")
    code = subprocess.call(argv)
    if code:
        show(f"[update] the rebuild failed (exit {code}); the previous image is unchanged")
    return code == 0 or not offer.required


def notice(composed, home: Path | None, show=print) -> None:
    """One line for commands that never ask."""
    try:
        offer = offer_for(composed, home)
    except Exception:  # noqa: BLE001; an update check never stops a start
        return
    if offer is not None:
        show(f"[update] {describe(offer)} openrua {' '.join(offer.rebuild)}")
