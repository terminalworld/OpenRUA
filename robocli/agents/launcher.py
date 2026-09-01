"""Headless agent launcher: send an occupant into a live sandbox.

The agent lives INSIDE the sandbox: this module docker-execs the agent
CLI in the sandbox container as the ``robot`` user in /workspace; all
native tools operate in-sandbox and containment is the container wall
itself. Invoked by the evaluator as a subprocess (never imported;
architecture contract). Everything agent-specific (binary, flags, auth
env, transcript format) comes from the adapter selected by ``--cli``.

``python -m robocli.agents.launcher --sandbox <container> --task
"<sentence>" --transcript <path> [--prompt-file <path>] [--cli <name>]
[--model ...] [--max-turns N] [--proxy http://host:port]
[--session-id <uuid>] [--resume] [--autocompact <auto|tokens>]``

A trial suspended at a quota wall resumes by re-running this launcher with
``--resume`` and the same ``--session-id``: the agent is shown the
content-free resume prompt instead of the task (the session already holds
it), and the new segment is APPENDED to the same transcript so one file
still holds the whole trial.

Network posture (locked): the sandbox's only way out is the model-API
wall; the adapter disables any server-side search the CLI offers (it
cannot be firewalled), verifiable in the transcript.

The launcher is skeleton: pure process construction, zero tricks. Any
model-compensating aid belongs in ``addons/`` (does not exist unless
debugging forces it), never here.
"""

from __future__ import annotations

import argparse
import subprocess
import sys
from pathlib import Path

from robocli import agents

# Hand-run convenience only: the evaluator always passes the real URL it
# got from proxy ensure. Must stay consistent with the proxy package's
# defaults (leaves cannot import each other; tests/test_agents.py guards
# the pair against drift).
DEFAULT_PROXY = "http://robocli-proxy:8888"


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--sandbox", required=True)
    ap.add_argument("--task", required=True)
    ap.add_argument("--prompt-file", default=None,
                    help="override prompt template with a {task} "
                    "placeholder (default: agents.PROMPT)")
    ap.add_argument("--transcript", required=True)
    ap.add_argument("--cli", default=None,
                    help="agent adapter name (default: robocli.agents default)")
    ap.add_argument("--model", default=None)
    ap.add_argument("--effort", default=None,
                    help="reasoning effort level (default: adapter's "
                    "DEFAULT_EFFORT)")
    ap.add_argument("--max-turns", type=int, default=100)
    ap.add_argument("--proxy", default=DEFAULT_PROXY)
    ap.add_argument("--autocompact", default=None,
                    help="context-compaction threshold (default: the "
                    "adapter's DEFAULT_AUTOCOMPACT)")
    ap.add_argument("--session-id", default=None,
                    help="name the session up front so it can be resumed")
    ap.add_argument("--resume", action="store_true",
                    help="continue --session-id instead of starting it; the "
                    "agent is shown agents.RESUME_PROMPT, not the task, and "
                    "the transcript is appended to")
    args = ap.parse_args()

    if args.resume and not args.session_id:
        ap.error("--resume needs the --session-id of the session to continue")

    agent = agents.get(args.cli)
    if args.resume:
        prompt = agents.RESUME_PROMPT
    else:
        template = Path(args.prompt_file).read_text() if args.prompt_file \
            else agents.PROMPT
        prompt = template.format(task=args.task)
    cmd = agent.launch_argv(
        sandbox=args.sandbox,
        prompt=prompt,
        model=args.model or agent.DEFAULT_MODEL,
        max_turns=args.max_turns,
        proxy=args.proxy,
        effort=args.effort or agent.DEFAULT_EFFORT,
        autocompact=args.autocompact or agent.DEFAULT_AUTOCOMPACT,
        session_id=args.session_id,
        resume=args.resume,
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
