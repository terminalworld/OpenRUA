---
summary: Start terminal and browser chat with one simulated robot, reconnect, and keep your work
read_when:
  - You want to try shared robot chat for the first time
  - You want a manual check of the browser, queue, and saved observations
---

# Your first shared robot session

This walkthrough uses one simulated Panda-Omron in RoboCasa365 and one native coding agent, with
OpenRUA's terminal and browser as clients. They share the same agent
conversation and queue. OpenRUA manages delivery and resources; the agent
still decides which ROS commands and programs to execute.

Bounded three-turn observation and gripper-control checks have passed with
both Claude Code and Codex; they do not establish success on the blender-lid
task below. See [validation scope](../docs/sessions.md#validation-scope).

## Guided setup

Run `openrua` for a new session with an automatic ID. First it asks
whether you trust the current directory as your project; you can choose or
create another one. Your files remain there across conversations; see
[robot projects](../docs/projects.md). The TUI links robot,
simulator and benchmark choices using recorded startup checks. **Save project and start** saves a valid selection, sets up Docker and the agent's sign-in when
they are missing, builds missing images with visible progress, then starts the
background service and opens chat.
Existing images are reused; failed builds keep logs for retry. **Save project settings and check setup** reports preparation without downloading or starting resources.

The guided list offers Panda in the native robosuite Lift scene, Panda in
CaP-Bench, Panda in LIBERO-PRO, and Panda-Omron in RoboCasa365. Their default scenes have passed
startup, reset, sensor-stream and no-op control checks. This does not validate
every benchmark task or model task success. Other registered profiles remain
available through explicit CLI options. See
[setup validation](../docs/setup-validation.md) for evidence and adding combinations.

To change defaults later, use `openrua --setup`, `openrua config set`, or edit
`.openrua/settings.json` in the project. Changes apply to new sessions, not the running one.
The steps below use explicit settings and a name so commands can refer to the
same example session.

## 1. Prepare the package, login, and images

Use a Linux execution host with Docker available, as described in
[Install](../docs/install.md). Install the release in your Python environment,
or use `pip install -e .` inside an OpenRUA source checkout:

```sh
pip install -U openrua
openrua serve --help
openrua session --help
```

If the selected agent is not signed in on this computer, **Save project and start**
offers its sign-in, or run `openrua login codex` first
([your agent's login](../docs/install.md#your-agents-login)). You need an
available model and account quota when you send a task.

The setup action prepares missing images automatically. To prepare them manually
in advance, these commands are equivalent:

```sh
openrua build --bench robocasa365
openrua build sandbox --distro humble --agent codex
openrua doctor panda-omron --sim robosuite --bench robocasa365 --agent codex
```

`build --bench` builds the simulator image. It does not also install the agent
in a sandbox. A session you start yourself needs no proxy: the agent reaches
the internet as it does on your own machine. The first build downloads the simulator and
its dependencies; allow it to finish before starting the session.

## 2. Start the terminal session

```sh
openrua --robot panda-omron --sim robosuite --bench robocasa365 \
  --task-suite CloseBlenderLid --task-id 0 --agent codex \
  --name chat-demo --port 8765
```

This command supplies the environment and agent explicitly, so OpenRUA checks
preparation and opens chat directly. It selects a blender-lid closing scene,
independent of a saved benchmark default; it starts one interactive scene,
not a benchmark campaign. Without `--model`, the selected agent's configured
model remains in use. Add `--setup` to review these choices in the menu before
starting, or use `/model` in chat to choose a model available to your account.

Wait for chat to open. Type an instruction and press **Enter** to send;
**Ctrl+J** inserts a newline. `/queue` opens queued instructions; `/tools`
opens tool output. **Ctrl+D** twice on empty input opens the resource choice. Select **Yes**
to keep it running for the next steps. **No** is selected by default and
stops the agent and the simulator or drivers OpenRUA started. If port 8765 is
occupied or `chat-demo` has retained work from an ended session, choose a new
port or name. Existing records are never overwritten.

## 3. Reconnect or open the browser

This walkthrough uses `--name chat-demo` so its commands can refer to the same
session. In ordinary use, omit the name for an automatic ID and come back with
`openrua --resume`, which lists running sessions first.

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

In a free terminal on the same execution host (leave the TUI with `/quit`
and select **Yes** to keep the session running if it is occupying the terminal):

```sh
openrua --gui --name chat-demo
```

Paste the displayed access token into the page and select **Connect**. Wait
for **Ready**. Send this message from either client:

> Inspect the workspace documentation and robot interfaces. Save one camera image as snaps/first.png and describe what you see. Do not move the robot yet.

The page should show your message, agent output, and tool activity. In
**Workspace files**, open `snaps` and select `first.png` after the agent saves
it. Use **Refresh** if needed. This is a saved observation with a modification
time; for the cameras live, press **Watch** under **Cameras**. If the agent reports an error, inspect it;
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

While a browser-requested turn is running, submit another message from another terminal:

```sh
openrua session --name chat-demo send "Summarize what you observed. Do not move the robot."
```

The browser should show the new message joining the running turn, then
joined: Claude Code and Codex take what you send mid-turn, as in their own
terminals. If the first turn already finished, the second starts as a turn of
its own. To inspect the conversation's saved events without submitting another task:

```sh
openrua session --name chat-demo events
```

The command prints the events and exits. Close or disconnect the browser while a
turn runs, then reconnect with the same address and token. Execution continues
in the background, and the browser reloads retained messages. Reloading the page
requires pasting the token again. Do not resend a task just to reconnect.

## 5. Try manipulation, interruption, and queue editing

In the browser, ask the agent to close the blender lid and verify the outcome.
During the turn, submit a follow-up such as “Save another camera image and
describe the final scene.” It joins the running turn.

Select **Interrupt current turn** while a turn is active. The turn stops, the
robot's motion actions are asked to cancel their goals, and the next queued
message starts; the queue does not pause. Agent failure or an unknown execution
outcome pauses the queue. Messages sent while it is paused stay queued, and the
queued message controls edit or withdraw them before they start. Inspect the
current state, revise a queued instruction if needed, and select **Continue
queued messages** to continue; the page explains when an explicit
reconciliation note is required.

Interrupting an agent is not a physical emergency stop. This walkthrough uses
simulation. If an operation completes before you interrupt it, run another
turn to test interruption rather than assuming cancellation was exercised.

## 6. End the session and find your files

Select **Stop session** in the browser, or run:

```sh
openrua session --name chat-demo end
openrua session --name chat-demo status
openrua session --name chat-demo events
```

The service exits after successful resource shutdown. Status and events remain
readable from disk. The workspace is your project directory; the event journal and
native stderr log are stored in `~/.openrua/sandboxes/chat-demo/`.
If you used `--home`, use that same directory for every client command.

Use `/resume` or `openrua --resume` to find this project's conversations.
A confirmed ended project conversation reconnects to fresh robot resources
with its native agent conversation; old actions and queued messages are not
replayed. `--resume --all` also shows other projects and older unassigned
records, which remain available read-only.

Starting with the old name does not resume the ended robot.
Running `openrua` again creates a new conversation with the same project files.
When you intentionally want to delete a
stopped session and its retained materials, run `openrua clean --name chat-demo`.

## What to check

| Action | Expected result |
|---|---|
| Browser and CLI submit | Messages appear in one conversation and share queue order |
| View an image or program | Saved file opens without sending the agent another task |
| Close the browser | Execution continues in the background service |
| Leave the terminal | Choose whether to stop execution resources or keep them running |
| Reconnect or reload | Retained progress returns; old tasks are not resubmitted |
| Send during a turn | The message joins the running turn |
| Interrupt | The turn stops and the next queued message starts |
| Encounter a failure | Follow-up messages remain paused for review |
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
