"""Errors that reach the user, with a fix and an exit code.

Library code raises these (ordinary exceptions a caller can catch);
only the CLI entry point turns one into a message and an exit status.
Exit codes follow sysexits.h so scripts can tell a bad config (78) from
a missing file (66), a service that is not up (69) or a login problem
(77) without parsing text.

Stdlib-only leaf: any unit may import it.
"""

from __future__ import annotations

EXIT_CODES = {
    "ok": 0,
    "error": 1,
    "usage": 2,         # bad arguments
    "noinput": 66,      # a named thing does not exist
    "unavailable": 69,  # docker, an image, a container, the simulator
    "noperm": 77,       # login or credentials
    "config": 78,       # a config file that does not fit the schema
}


class RoboCLIError(Exception):
    """Base: ``message`` says what is wrong, ``hint`` how to fix it
    (a command when there is one), ``exit_code`` what the process
    returns when this reaches the entry point."""
    code = "error"

    def __init__(self, message: str, hint: str = "") -> None:
        super().__init__(message)
        self.message = message
        self.hint = hint

    @property
    def exit_code(self) -> int:
        return EXIT_CODES[self.code]

    def __str__(self) -> str:
        return self.message


class UsageError(RoboCLIError):
    code = "usage"


class NotFound(RoboCLIError, FileNotFoundError):
    """A named robot, benchmark, agent or file does not exist."""
    code = "noinput"


class UnavailableError(RoboCLIError):
    """Something that should be running or built is not."""
    code = "unavailable"


class AuthError(RoboCLIError):
    code = "noperm"


class ConfigError(RoboCLIError, ValueError):
    """A config file that does not fit the schema; the message names the
    file and every offending key path."""
    code = "config"
