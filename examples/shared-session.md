---
summary: Start TUI, browser, and CLI chat with one simulated robot, reconnect, and keep your work
read_when:
  - You want to try shared robot chat for the first time
  - You want a manual check of the browser, queue, and saved observations
---

# Your first shared robot session

This walkthrough uses one simulated Panda-Omron in RoboCasa365 and one native coding agent, with
OpenRUA's TUI, browser, or plain CLI as clients. They share the same agent
conversation and queue. OpenRUA manages delivery and resources; the agent
still decides which ROS commands and programs to execute.

This walkthrough uses OpenRUA 0.7.0. Shared chat remains experimental.
Bounded three-turn observation and gripper-control checks have passed with
both Claude Code and Codex; they do not establish success on the blender-lid
task below. See [validation scope](../docs/sessions.md#validation-scope).

## Guided setup

Run `openrua` for a new session with an automatic ID. The TUI links robot,
simulator and benchmark choices using recorded startup checks. **Prepare and
start** saves a valid selection, builds missing images with visible progress,
checks the agent login, then starts the background service and opens chat.
Existing images are reused; failed builds keep logs for retry. **Save & check**
reports preparation without downloading or starting resources.

The guided list offers Panda in the native robosuite Lift scene, Panda in
CaP-Bench, and Panda-Omron in RoboCasa365. Their default scenes have passed
startup, reset, sensor-stream and no-op control checks. This does not validate
every benchmark task or model task success. Other registered profiles remain
available through explicit CLI options. See
[setup validation](../docs/setup-validation.md) for evidence and adding combinations.

To change defaults later, use `openrua --setup`, `openrua config set`, or edit
the same `config.yaml`. Changes apply to new sessions, not the running one.
The steps below use explicit settings and a name so commands can refer to the
same example session.

## 1. Prepare the package, login, and images

Use a Linux execution host with Docker available, as described in
[Install](../docs/install.md). Install the release in your Python environment,
or use `pip install -e .` inside an OpenRUA source checkout:

```sh
pip install -U 'openrua>=0.7.0'
openrua serve --help
openrua session --help
```

Configure the selected agent's native login using the instructions in
[Logging an agent in](../docs/install.md#logging-an-agent-in).
This uses the agent's own CLI and login profile. You need an available model
and account quota when you send a task.

The setup action prepares missing images automatically. To prepare them manually
in advance, these commands are equivalent:

```sh
openrua build --bench robocasa365
openrua build sandbox --distro humble --agent codex
openrua build proxy --agent codex
openrua doctor panda-omron --sim robosuite --bench robocasa365 --agent codex
```

`build --bench` builds the simulator image. It does not also install the agent
in a sandbox or build the proxy. The first build downloads the simulator and
its dependencies; allow it to finish before starting the session.

## 2. Start the terminal session

```sh
openrua --robot panda-omron --sim robosuite --bench robocasa365 \
  --task-suite CloseBlenderLid --task-id 0 --agent codex \
  --name chat-demo --port 8765
```

The TUI shows these choices. Select **Prepare and start** to save the defaults,
check the preparation, and start the background service. This selects a
blender-lid closing scene, independent of a saved benchmark default; it starts one
interactive scene, not a benchmark campaign. Leave the model field blank for
the configured default, or select one available to your account.

Wait for chat to open. Type an instruction and press **Enter** to send;
**Ctrl+J** inserts a newline. `/queue` opens queued instructions; `/tools`
opens tool output. **Ctrl+D** on empty input detaches without stopping the service. If port 8765 is
occupied or `chat-demo` has retained work from an ended session, choose a new
port or name. Existing records are never overwritten.

## 3. Reconnect or open the browser

This walkthrough uses `--name chat-demo` so its commands can refer to the same
session. In ordinary use, omit the name for an automatic ID and use `/resume`
in the startup form or chat to find it later.

After leaving the TUI, reconnect without repeating startup options:

```sh
openrua --name chat-demo
```

Use the browser instead to inspect saved images:

### Browser and saved images

The browser connects to the same conversation, including messages sent from the
TUI. Use it to inspect saved images. In the TUI, `/files` opens the same workspace for text previews and
images on supported terminals; see
[workspace controls](../ui/terminal/README.md#saved-workspace-files).

In a free terminal on the same execution host (detach with Ctrl+D first if
the TUI is occupying it):

```sh
openrua --gui --name chat-demo
```

Paste the displayed access token into the page and select **Connect**. Wait
for **Ready**. Send this message from either client:

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
turn runs, then reconnect with the same address and token. Execution continues
in the background, and the browser reloads retained messages. Reloading the page
requires pasting the token again. Do not resend a task just to reconnect.

## 5. Try manipulation, interruption, and queue editing

In the browser, ask the agent to close the blender lid and verify the outcome.
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

The service exits after successful resource shutdown. Status and events remain
readable from disk. With the default user directory, the workspace is at
`~/.openrua/sandboxes/chat-demo/workspace/workspace/`; the event journal and
native stderr log are stored in `~/.openrua/sandboxes/chat-demo/`.
If you used `--home`, use that same directory for every client command.

Use `/resume` or `openrua --resume` to open an ended conversation read-only.
Its `/files` browser reads the recorded workspace without restarting the robot
or agent. Missing files do not prevent reading the retained transcript.

Starting with the old name does not resume the ended robot.
Use a new name to keep earlier work. When you intentionally want to delete a
stopped session and its retained materials, run `openrua clean --name chat-demo`.

## What to check

| Action | Expected result |
|---|---|
| Browser and CLI submit | Messages appear in one conversation and share queue order |
| View an image or program | Saved file opens without sending the agent another task |
| Close a browser or chat client | Execution continues in the background service |
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
- **Connection refused:** inspect `~/.openrua/launches/chat-demo/service.log`
  and check the SSH tunnel if used. Restarting the browser cannot recover a
  stopped owner or prove that a startup still in progress has failed.

For CLI editing, input replies, recovery limits and API details, see the
[shared session manual](../docs/sessions.md). To use the agent's original
interactive terminal, follow [First task](first-task.md) with a separate
session. Attaching that original terminal to this same shared conversation is
not yet supported.
