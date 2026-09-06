"""The package's front door: ``python -m robocli.agents <verb> ...``.

From outside, the occupant package is ONE self-contained unit; the
verbs below are its whole command surface (adapters are never invoked
by module path). ``preinstall`` and ``whitelist`` emit build-time facts
for the sandbox/proxy images' generic slots, for one or several agents
(``--cli`` repeats; the union is emitted).
"""

from __future__ import annotations

import argparse
import sys

_VERBS = {
    "launch": "sandbox + task + credentials -> transcript",
    "preinstall": "agents -> seat install command (sandbox.build slot)",
    "whitelist": "agents -> wall domain regexes (proxy.build slot)",
    "list": "the adapters available (bundled, then ~/.robocli/agents/)",
}


def _usage() -> str:
    from robocli.agents import DEFAULT_CLI
    lines = ["usage: python -m robocli.agents <verb> [options]",
             "", "verbs:"]
    lines += [f"  {v:<12} {desc}" for v, desc in _VERBS.items()]
    lines += ["", f"default adapter: {DEFAULT_CLI}",
              "verb options: python -m robocli.agents <verb> --help",
              "contract: robocli/agents/base.py; conformance: robocli.testing.check_agent"]
    return "\n".join(lines)


def _emit(verb: str, argv: list[str]) -> int:
    from robocli import agents, paths
    ap = argparse.ArgumentParser(
        prog=f"python -m robocli.agents {verb}", description=_VERBS[verb])
    ap.add_argument("--cli", action="append", default=None,
                    help=f"adapter name; repeat for several (default: {agents.DEFAULT_CLI})")
    ap.add_argument("--home", default=None, type=paths.home,
                    help="user directory holding agents/ (default: ~/.robocli)")
    args = ap.parse_args(argv)
    if verb == "list":
        for a in agents.available(args.home):
            if a.agent is None:
                print(f"{a.name:<16} {a.source:<8} FAILED: {a.error}")
            else:
                print(f"{a.name:<16} {a.source:<8} {' '.join(sorted(a.agent.capabilities))}")
        return 0
    chosen = [agents.get(c, args.home) for c in (args.cli or [agents.DEFAULT_CLI])]
    if verb == "preinstall":
        print(agents.preinstall(chosen))
    else:
        print("\n".join(agents.whitelist(chosen)))
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
    if verb in ("preinstall", "whitelist", "list"):
        return _emit(verb, args[1:])
    sys.argv = ["python -m robocli.agents.launcher", *args[1:]]
    from robocli.agents import launcher
    return launcher.main()


if __name__ == "__main__":
    sys.exit(main())
