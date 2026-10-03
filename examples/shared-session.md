---
summary: Start browser and CLI chat with one simulated robot, check the queue, reconnect, and keep your work
read_when:
  - You want to try shared robot chat for the first time
  - You want a manual check of the browser, queue, and saved observations
---

# Your first shared robot session

This walkthrough uses one simulated Panda, one native coding agent, and two
clients: a browser and a terminal. Both clients talk to the same agent
conversation and queue. OpenRUA manages delivery and resources; the agent
still decides which ROS commands and programs to execute.

Shared chat is experimental and available from OpenRUA 0.1.0.
Earlier releases do not include these commands. The walkthrough starts
with Codex, for which live conversation and simulation checks have completed.
Claude Code also has a conversation adapter; its successful model turns have
not yet been validated. See [validation scope](../docs/sessions.md#validation-scope).

## 1. Prepare the package, login, and images

Use a Linux execution host with Docker available, as described in
[Install](../docs/install.md). Install the release in your Python environment,
or use `pip install -e .` inside an OpenRUA source checkout:

```sh
pip install -U 'openrua>=0.1.0'
openrua serve --help
openrua session --help
```

Configure the selected agent's native login using the instructions in
[Logging an agent in](../docs/install.md#logging-an-agent-in).
This uses the agent's own CLI and login profile. You need an available model
and account quota when you send a task.

Build the three images explicitly. Skip unchanged builds if they are already
present and `doctor` reports them ready:

```sh
openrua build --bench capbench
openrua build sandbox --distro humble --agent codex
openrua build proxy --agent codex
openrua doctor panda --sim robosuite --bench capbench --agent codex
```

`build --bench` builds the simulator image. It does not also install the agent
in a sandbox or build the proxy. The first build downloads the simulator and
its dependencies; allow it to finish before starting the session.

## 2. Start the execution host

In terminal A:

```sh
openrua serve panda --sim robosuite --bench capbench \
  --task-suite capbench_lift --task-id 0 --agent codex \
  --name chat-demo --port 8765
```

This explicitly selects a cube-lifting scene, independent of a saved benchmark
default. It starts one interactive scene, not a benchmark campaign. The agent
model comes from its configuration; add `--model MODEL` to select another model
available to your account.

Wait for the `[serve]` address and connection instructions. Keep this terminal
running: closing a client is harmless to the session, but Ctrl-C in **serve**
requests shutdown. If port 8765 is occupied, choose another port. If
`chat-demo` already exists, choose a new name; previous work is never overwritten.

## 3. Open the browser and request an observation

In terminal B on the same execution host:

```sh
openrua session --name chat-demo web
```

Paste the displayed access token into the page and select **Connect**. Wait
for **Ready**. Send this message:

> Inspect the workspace documentation and robot interfaces. Save one camera image as snaps/first.png and describe what you see. Do not move the robot yet.

The page should show your message, agent output, and tool activity. In
**Workspace files**, open `snaps` and select `first.png` after the agent saves
it. Use **Refresh** if needed. This is a saved observation with a modification
time, not a continuous camera feed. If the agent reports an error, inspect it;
a completed turn alone does not prove the requested image was saved.

### If the execution host is a remote server

On that server, use `openrua session --name chat-demo web --no-open` to print
the address and token. On your laptop, forward the same port through SSH:

```sh
ssh -N -L 8765:127.0.0.1:8765 USER@ROBOT_HOST
```

Replace `USER@ROBOT_HOST` with your SSH login, then open
`http://127.0.0.1:8765` on your laptop and paste the token. Keep the SSH tunnel
running. This forwards the local service; OpenRUA does not expose a public
web server or provide a hosted mobile app.

## 4. Submit from both clients and reconnect

While a browser-requested turn is running, submit another message from terminal B:

```sh
openrua session --name chat-demo send "Summarize what you observed. Do not move the robot."
```

The browser should show the new message queued behind the active turn. If the
first turn already finished, the second can start immediately. To observe the
same conversation from the terminal without submitting another task:

```sh
openrua chat --name chat-demo --follow
```

Ctrl-C here only detaches this client. Close or disconnect the browser while a
turn runs, then reconnect with the same address and token. Progress continues
in terminal A, and the browser reloads retained messages. Reloading the page
requires pasting the token again. Do not resend a task just to reconnect.

## 5. Try manipulation, interruption, and queue editing

In the browser, ask the agent to pick up the red cube and verify the outcome.
During an active turn, submit a follow-up such as “Save another camera image
and describe the final scene.” Use the queued message controls to edit or
withdraw it before it starts.

Select **Interrupt current turn** while the first turn is active. The queue should become
paused and retain the follow-up. Inspect the current state, revise the queued
instruction if needed, and select **Continue queued messages** to continue. Agent failure
or an unknown execution outcome also pauses the queue; the page explains when
an explicit reconciliation note is required.

Interrupting an agent is not a physical emergency stop. This walkthrough uses
simulation. If an operation completes before you interrupt it, run another
turn to test interruption rather than assuming cancellation was exercised.

## 6. End the session and find your files

Select **End session** in the browser, or run:

```sh
openrua session --name chat-demo end
openrua session --name chat-demo status
openrua session --name chat-demo events
```

Terminal A exits after successful resource shutdown. Status and events remain
readable from disk. With the default user directory, the workspace is at
`~/.openrua/sandboxes/chat-demo/workspace/workspace/`; the event journal and
native stderr log are stored in `~/.openrua/sandboxes/chat-demo/`.
If you used `--home`, use that same directory for every client command.

Starting another `serve` with the old name does not resume the ended robot.
Use a new name to keep earlier work. When you intentionally want to delete a
stopped session and its retained materials, run `openrua clean --name chat-demo`.

## What to check

| Action | Expected result |
|---|---|
| Browser and CLI submit | Messages appear in one conversation and share queue order |
| View an image or program | Saved file opens without sending the agent another task |
| Close a browser or chat client | Execution continues while `serve` remains running |
| Reconnect or reload | Retained progress returns; old tasks are not resubmitted |
| Interrupt or encounter a failure | Follow-up messages remain paused for review |
| End the session | Owned resources stop; messages, images, and code remain |

## If something does not work

- **Unknown `serve` command:** check which Python environment supplies `openrua`
  and install the current repository version there.
- **Missing or stale image/login:** run the exact `doctor` command above and
  follow the repair command in its report. Agent images must contain the agent
  selected for this session.
- **Quota or native agent error:** inspect the browser's error and the retained
  `agent.stderr.log`; do not assume the pending queue continued. Native account
  quota is not bypassed by the shared interface.
- **No image:** check the agent's actual output path and tool result, then
  refresh the workspace browser. The interface does not capture images by itself.
- **Connection refused:** check that terminal A and, if used, the SSH tunnel
  remain running. Restarting the browser cannot restart a stopped owner.

For CLI editing, input replies, recovery limits and API details, see the
[shared session manual](../docs/sessions.md). To use the agent's original
interactive terminal, follow [First task](first-task.md) with a separate
session. Attaching that original terminal to this same shared conversation is
not yet supported.
