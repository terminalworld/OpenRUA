"""Docker on this computer, set up with the person when it is not ready.

OpenRUA runs the robot, the simulator and the agent's terminal in
containers. When a start needs it, at a terminal: when Docker is not installed,
this user may not use it yet, or its service is stopped, say so and
offer the fix, which runs only on a yes and asks for the sudo password
itself. Joining the docker group counts from the next login; this start
goes on at once under ``sg docker``. Docker's own install script
(get.docker.com) covers the common Linux distributions; its guide covers
the rest.
"""

from __future__ import annotations

import getpass
import os
import shlex
import shutil
import subprocess
import sys
import tempfile
from typing import Callable

from openrua.errors import UnavailableError

INSTALL_SCRIPT = "https://get.docker.com"
GUIDE = "https://docs.docker.com/engine/install/"
WHY = "OpenRUA runs the robot, the simulator and your agent's terminal in Docker containers."


def state(engine_error: Callable[[], str] | None = None) -> tuple[str, str]:
    """(what, detail): what is "ready", "missing", "no access", "stopped"
    or "other", with Docker's own words for the last three."""
    if not shutil.which("docker"):
        return "missing", ""
    if engine_error is None:
        from openrua.doctor.checks import engine_error
    problem = engine_error()
    if not problem:
        return "ready", ""
    low = problem.lower()
    if "permission denied" in low:
        return "no access", problem
    if "cannot connect to the docker daemon" in low or "is the docker daemon running" in low:
        return "stopped", problem
    return "other", problem


def _yes(answer: str) -> bool:
    return answer.strip().lower() in ("", "y", "yes")


def _steps(what: str, user: str) -> list[list[str]]:
    """The commands that fix ``what``, as they will be shown and run."""
    if what == "missing":
        script = os.path.join(tempfile.gettempdir(), "get-docker.sh")
        return [["curl", "-fsSL", INSTALL_SCRIPT, "-o", script], ["sudo", "sh", script],
                ["sudo", "usermod", "-aG", "docker", user]]
    if what == "no access":
        return [["sudo", "usermod", "-aG", "docker", user]]
    return [["sudo", "systemctl", "enable", "--now", "docker"]]


def ensure(ask: Callable[[str], str] = input, run=subprocess.run,
           engine_error: Callable[[], str] | None = None, restart=None) -> None:
    """Return when Docker is ready; otherwise offer the fix. Raises
    UnavailableError when it is declined or does not take."""
    what, detail = state(engine_error)
    if what == "ready":
        return
    if what == "other":
        raise UnavailableError(f"Docker is not answering: {detail}",
                               hint="openrua doctor says more; then run openrua again")
    user = getpass.getuser()
    question = {"missing": "Docker is not installed on this computer.",
                "no access": f"Docker is installed, but {user} may not use it yet.",
                "stopped": "Docker is installed, but its service is not running."}[what]
    steps = _steps(what, user)
    print(f"\n{question} {WHY}\nOpenRUA can set it up now with:", flush=True)
    for step in steps:
        print(f"  {shlex.join(step)}", flush=True)
    print("(sudo asks for your password.)", flush=True)
    if not _yes(ask("Run these now? [Y/n] ")):
        raise UnavailableError(question.rstrip("."), hint=f"set it up as {GUIDE} says, "
                               "then run openrua again")
    if what == "missing" and not shutil.which("curl"):
        raise UnavailableError("downloading Docker's install script needs curl",
                               hint=f"sudo apt-get install curl, or install Docker as {GUIDE} says")
    for step in steps:
        if run(step).returncode != 0:
            raise UnavailableError(f"{shlex.join(step)} failed",
                                   hint=f"install Docker as {GUIDE} says, then run openrua again")
    if what in ("missing", "no access"):
        # The new group counts from the next login: go on under it now.
        (restart or _restart_in_docker_group)()
    if state(engine_error)[0] != "ready":
        raise UnavailableError("Docker is still not ready", hint="openrua doctor says what is wrong")


def _restart_in_docker_group() -> None:
    argv = [sys.executable, "-m", "openrua", *sys.argv[1:]]
    if not shutil.which("sg"):
        raise UnavailableError("you were added to the docker group, which counts from your "
                               "next login", hint="log out and back in, then run openrua again")
    print("Docker is ready; going on.", flush=True)
    sys.stdout.flush()
    os.execvp("sg", ["sg", "docker", "-c", shlex.join(argv)])
