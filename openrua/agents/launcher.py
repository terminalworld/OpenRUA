"""Headless agent launcher: run an agent in a live sandbox.

The agent lives inside the sandbox: this module docker-execs the agent
CLI in the sandbox container as the ``robot`` user in /workspace; all
native tools operate in-sandbox and containment is the container
itself. Invoked by the runner as a subprocess (never imported).
Everything agent-specific (binary, flags, auth env, transcript format)
comes from the agent selected by ``--agent``.

``python -m openrua.agents.launcher --sandbox <container> --task
"<sentence>" --transcript <path> [--prompt-file <path>] [--agent <name>]
[--model ...] [--option k=v ...] [--max-turns N] [--proxy http://host:port]
[--session-id <uuid>] [--resume] [--token-file <path>]``

A trial suspended at a quota wall resumes by re-running this launcher with
``--resume`` and the same ``--session-id``: the agent is shown the
content-free resume prompt instead of the task (the session already holds
it), and the new segment is APPENDED to the same transcript so one file
still holds the whole trial.

Network: the sandbox's only way out is the model-API proxy; the agent's
hooks disable any server-side search the CLI offers (it cannot be
proxied), verifiable in the transcript.

The launcher is pure process construction: no prompt tricks, no aids.
"""

from __future__ import annotations

import argparse
import subprocess
import sys
from pathlib import Path

from openrua import agents

# Hand-run convenience only: the evaluator always passes the real URL it
# got from proxy ensure. Must stay consistent with the proxy package's
# defaults (leaves cannot import each other; tests/test_agents.py guards
# the pair against drift).


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--sandbox", required=True)
    ap.add_argument("--task", required=True)
    ap.add_argument("--prompt-file", default=None,
                    help="override prompt template with a {task} "
                    "placeholder (default: agents.PROMPT)")
    ap.add_argument("--transcript", required=True)
    ap.add_argument("--agent", default=None,
                    help="agent adapter name (default: openrua.agents default)")
    ap.add_argument("--model", default=None,
                    help="model id (default: the adapter's default_model)")
    ap.add_argument("--option", action="append", default=[], metavar="KEY=VALUE",
                    help="adapter knob (repeatable; the adapter's "
                    "default_options apply underneath)")
    ap.add_argument("--home", default=None,
                    help="user directory holding agents/ (default: ~/.openrua)")
    ap.add_argument("--max-turns", type=int, default=100)
    ap.add_argument("--proxy", required=True, help="the proxy URL the sandbox reaches its model API through")
    ap.add_argument("--session-id", default=None,
                    help="name the session up front so it can be resumed")
    ap.add_argument("--token-file", default=None,
                    help="file holding the sandbox CLI's auth token as "
                    "KEY=value; docker hands it to the CLI process only")
    ap.add_argument("--resume", action="store_true",
                    help="continue --session-id instead of starting it; the "
                    "agent is shown agents.RESUME_PROMPT, not the task, and "
                    "the transcript is appended to")
    args = ap.parse_args()

    if args.resume and not args.session_id:
        ap.error("--resume needs the --session-id of the session to continue")

    agent = agents.get(args.agent, args.home)
    options = {}
    for item in args.option:
        if "=" not in item:
            ap.error(f"--option expects KEY=VALUE, got {item!r}")
        k, v = item.split("=", 1)
        options[k] = v
    if args.resume:
        prompt = agents.RESUME_PROMPT
    else:
        template = Path(args.prompt_file).read_text() if args.prompt_file \
            else agents.PROMPT
        prompt = template.format(task=args.task)
    cmd = agent.launch_argv(
        sandbox=args.sandbox,
        prompt=prompt,
        model=args.model or agent.default_model,
        max_turns=args.max_turns,
        proxy=args.proxy,
        options=options,
        session_id=args.session_id,
        resume=args.resume,
        token_file=args.token_file,
    )
    # stderr goes to a sidecar, not DEVNULL: a launch-dead CLI (docker
    # exec miss, bad flag) exits loud but used to leave zero evidence.
    # Append on resume so the segments of one trial stay in one transcript;
    # truncate otherwise, so a retried attempt never inherits a dead one's
    # records.
    mode = "a" if args.resume else "w"
    with open(args.transcript, mode) as out, \
            open(f"{args.transcript}.stderr", mode) as err:
        proc = subprocess.run(cmd, stdout=out, stderr=err)
    return proc.returncode


if __name__ == "__main__":
    sys.exit(main())
