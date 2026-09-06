"""The two strings an agent is ever shown by the harness.

``PROMPT`` opens a trial; ``{task}`` is its only placeholder. The
launcher formats it and the trial record hashes the same string, so the
hash pins the text the agent saw. ``RESUME_PROMPT`` is what a trial
suspended at a quota wall receives when it continues: content-free on
purpose, since the session already holds the task and the workspace,
and restating either would hand a resumed trial context an
uninterrupted one never had. A resumed trial still receives one more
user turn than an uninterrupted one; that difference is recorded, not
hidden.
"""

PROMPT = """\
You are working on a robot's onboard computer.

Your task: {task}

Survey the machine yourself to find out what robot this is and what it
can do; work until the task is physically done, verify it your own way,
then finish.

The workspace contains starter docs and tools you can use.
"""

RESUME_PROMPT = "Continue where you left off."
