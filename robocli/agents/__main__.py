"""The package's front door: ``python -m robocli.agents <verb> ...``.

From outside, the occupant package is ONE self-contained unit; the
verbs below are its whole command surface (adapters are never invoked
by module path). ``preinstall`` and ``whitelist`` emit build-time facts
for the sandbox/proxy images' generic slots.
"""

from __future__ import annotations

import argparse
import sys

_VERBS = {
    "launch": "sandbox + task + credentials -> transcript",
    "preinstall": "adapter -> seat install command (sandbox.build slot)",
    "whitelist": "adapter -> wall domain regexes (proxy.build slot)",
}


def _usage() -> str:
    from robocli.agents import DEFAULT_CLI
    lines = ["usage: python -m robocli.agents <verb> [options]",
             "", "verbs:"]
    lines += [f"  {v:<12} {desc}" for v, desc in _VERBS.items()]
    lines += ["", f"adapters: {DEFAULT_CLI} (default)",
              "verb options: python -m robocli.agents <verb> --help",
              "contract: see robocli/agents/__init__.py"]
    return "\n".join(lines)


def _emit(verb: str, argv: list[str]) -> int:
    from robocli import agents
    ap = argparse.ArgumentParser(
        prog=f"python -m robocli.agents {verb}",
        description=_VERBS[verb])
    ap.add_argument("--cli", default=None,
                    help=f"adapter name (default: {agents.DEFAULT_CLI})")
    adapter = agents.get(ap.parse_args(argv).cli)
    if verb == "preinstall":
        print(adapter.sandbox_install())
    else:
        print("\n".join(adapter.proxy_filter_lines()))
    return 0


def main() -> int:
    args = sys.argv[1:]
    if not args or args[0] in ("-h", "--help"):
        print(_usage())
        return 0
    verb = args[0]
    if verb not in _VERBS:
        print(f"unknown verb {verb!r}\n\n{_usage()}", file=sys.stderr)
        return 2
    if verb in ("preinstall", "whitelist"):
        return _emit(verb, args[1:])
    sys.argv = ["python -m robocli.agents.launcher", *args[1:]]
    from robocli.agents import launcher
    return launcher.main()


if __name__ == "__main__":
    sys.exit(main())
