"""``python -m robocli.sandbox <verb> ...``: the package's command line.

From outside, the package is ONE self-contained unit; verbs are its
interface and the files behind them are implementation detail. Each
verb module stays independently runnable on its own (``python -m
robocli.sandbox.up ...`` works identically); this dispatcher only
forwards.
"""

from __future__ import annotations

import sys

_VERBS = {
    "build": ("robocli.sandbox.build", "Dockerfile + PREINSTALL -> sandbox image"),
    "up": ("robocli.sandbox.up", "machine + config -> live sandbox container"),
    "down": ("robocli.sandbox.down", "container name -> removed"),
    "workspace": ("robocli.sandbox.workspace",
                  "config -> seeded workspace / template hash"),
}


def _usage() -> str:
    lines = ["usage: python -m robocli.sandbox <verb> [options]",
             "", "verbs:"]
    lines += [f"  {v:<10} {desc}" for v, (_, desc) in _VERBS.items()]
    lines += ["", "verb options: python -m robocli.sandbox <verb> --help",
              "contract: see robocli/sandbox/__init__.py"]
    return "\n".join(lines)


def main() -> int:
    args = sys.argv[1:]
    if not args or args[0] in ("-h", "--help"):
        print(_usage())
        return 0
    verb = args[0]
    if verb not in _VERBS:
        print(f"unknown verb {verb!r}\n\n{_usage()}", file=sys.stderr)
        return 2
    module = _VERBS[verb][0]
    sys.argv = [f"python -m {module}", *args[1:]]
    import importlib
    return importlib.import_module(module).main()


if __name__ == "__main__":
    sys.exit(main())
