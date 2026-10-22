# Changelog

Versions are 0.b.c until the config format, the commands and the agent
plugin contract are declared stable, at 1.0.0. A release that asks you
to change something (a config key removed or renamed, a command flag
removed, a default that changes what a run does, an incompatible plugin
contract) raises b; any other release raises c. Changes collect under
Unreleased until a release is cut, and the release decides its number.

## Unreleased

- **See your robot live**: the browser's Cameras panel shows the robot's
  cameras a few frames a second, one camera at a time, read only while
  someone watches; `/watch` in the terminal says how to open it.
- **A real robot's run is recorded**: `openrua exec` on a real robot keeps
  its cameras, two frames a second, in the run (`--no-record` to skip;
  `--record CAMERAS` picks them), and `openrua demo` turns it into a video
  of the agent's commands beside the cameras, as for a simulated run.
- A real robot's preflight waits for its topics to arrive: the first
  `ros2 topic echo` used to give up on a topic not yet discovered, which
  stopped a real robot's `exec` or `bench` before the agent started.
- Agent manifests no longer take `instruction_file`, which nothing read:
  every workspace carries AGENTS.md, and both Claude Code and Codex read
  it. A manifest that still names it is refused; delete the line.

## 0.16.0

Two-arm robots, checked before the first real one. Changes you may notice:
`fjt_send.py` leads with `canceled` or `aborted` for a goal ended
elsewhere, and on a multi-arm machine `machine.yaml` lists each arm's
own frames, so a two-arm benchmark trial's agent sees a little more.

- **Two-arm robots**, checked on a simulated two-arm graph and with a live
  agent driving both arms: `openrua probe` drafts one `arms:` entry per
  trajectory controller (it took the first and left the rest in a
  comment); `machine.yaml` gives each arm its own base and hand frame;
  an interrupt cancels both arms' motions.
- `fjt_send.py` takes a position list that starts with a negative number
  (`-1.0,0.3,...`; argparse read it as an option), and a goal canceled or
  aborted elsewhere prints `canceled` or `aborted`, not `done`.
- In `machine.yaml`, the robot's `model` no longer carries the camera's
  comment.

## 0.15.0

The first minutes with a robot, `openrua exec`, and the browser. Changes
you may notice: a benchmark trial's agent sees a little differently
(Claude Code now reads the workspace's AGENTS.md, which gains one
sentence, and on Humble the `ros2` commands lose an error line);
`exec` and `bench` stopped by Ctrl-C record `stopped` and exit 130;
leaving the terminal asks whether to keep the session running.

- **The browser catches up with the terminal**: an Agent panel switches
  permissions and the model and lists the agent's own commands; the
  message box takes the same `/` commands; the page shows the same status
  notes (stopped robot, connecting checks, questions, a quiet minute) and
  the same notices (cancelled motions, a setting the agent refused). Both
  clients read commands, status and notices from one file, and tests fail
  when a command is added without a handler in either. In the page, a
  typed line answers the question the agent waits on, a tool shows what it
  ran (`Bash(ros2 topic list)`), and the agent's closing words show once.
- Typing a question's choice answers it in any case (`Allow` for `allow`).
- **Claude Code reads AGENTS.md in every session**: each session used to
  start from a copy of Claude's own state file, which on some machines was
  long stale and made the CLI skip AGENTS.md, so the agent could answer
  "I'm in a container, no camera" on a robot. Each session now builds its
  own state; settings still come along. AGENTS.md also says that
  /workspace is the one place kept after the session.
- **A welcome when a conversation opens**: which robot you are on, simulated
  or real, which agent is at the controls, and things to try; "Welcome
  back" when you return.
- **Files the agent saves are a click away**: a `/workspace/...` file named
  in a reply, and every file in `/files`, shows its path on this computer,
  which Cursor and VS Code open on Ctrl+click; iTerm2 and kitty draw
  pictures over SSH too.
- **Leaving asks**: Ctrl+D or `/quit` asks whether to keep the session
  (and its robot) running in the background or end it now. `openrua
  --resume` lists the sessions still running first.
- Agent manifests take a `title`, the agent's name as people write it.
- **Watch `openrua exec` work**: what the agent says and each command or
  tool it runs show as they happen, with whether each worked and how long
  it took, and a line that keeps time on what is still running; the
  outputs stay in the run's transcript.
- **You judge a real robot's run**: at the end of `openrua exec` on a real
  robot you are asked whether it did the task, and your answer becomes the
  result (`verdict: person`). Every run also keeps the agent's last words
  (`agent_final_message`), its own account of how it went.
- **Ctrl-C stops a run in order**: during `openrua exec` or `bench`, the
  agent and everything it started end, a real robot's motions are
  cancelled, a simulated scene is judged as it stands, the robot powers
  off, and the record says `stopped` (the command exits 130). A closed
  terminal or a dropped SSH connection stops it the same way, and a second
  Ctrl-C cannot cut the power-off short.
- On Humble (robosuite's own scenes, CaP-Bench, RoboCasa365), the agent's `ros2`
  commands no longer start with a Fast DDS `IP_LOCATOR Error` line: a
  simulated robot and its sandbox find each other by address there.
- `openrua doctor` with no arguments also checks your saved robot,
  simulator and benchmark, the ones `openrua` starts.
- In a source checkout, the terminal refuses built files older than
  `ui/terminal/src` and says how to rebuild them.

## 0.14.1

- **OpenRUA keeps itself up to date**: `openrua` checks PyPI at most once a
  day and, when a newer OpenRUA is out, offers to update now, skip, or skip
  until the next version, as Codex does; updating runs the installer's own
  upgrade (pip, uv or pipx), and the next `openrua` is the new version.
  `up` and `exec` print one line. A source checkout is never offered one.
- `openrua run` is gone, its pointer too.
- The README's News reads as news.

## 0.14.0

One task done by the agent on its own and recorded, on any robot, with
`openrua exec`; `openrua run` is gone. Changes you may notice:
`openrua run` now says what replaced it, `run` and `up` no longer take
`--permissions`, and `openrua bench` on a real robot runs its startup
checks first.

- **`openrua exec`**: the agent does one task on its own and the run is
  recorded under `runs/`, on a simulated or a real robot, as `codex exec`
  or `claude -p` run a coding agent without its terminal. A benchmark
  scene states its own task and judges success; a simulator's own scene
  judges success and takes the sentence given when it states none; a real
  robot's task is the sentence given (`openrua exec ./my-robot.yaml "put
  the cup on the plate"`). Like `bench`, it applies the protocol's limits
  unless the flags say otherwise. `openrua bench` runs a benchmark's task
  set this way.
- **`openrua run` is removed**: talking to the agent is `openrua`, its own
  terminal is `openrua up` and then `openrua agent`, and a recorded run is
  `openrua exec`. `openrua run` says so for one release, then goes.
- The README has a News section: releases and the paper, newest first.
- `openrua bench` on a real robot runs the robot's startup checks before
  any trial, as a session does.
- `--permissions` is an option of `openrua` and `openrua serve` only,
  where it acts; `run` and `up` no longer accept it (the native terminal
  keeps the agent CLI's own prompting).

## 0.13.0

The shared terminal behaves the way a coding agent's own does, a real
robot is connected with the coding agent doing the work, and an
interrupt stops the robot's motions too. Changes you may notice:
interrupting no longer pauses the queue, and it now cancels the goals on
the robot's motion actions.

### The terminal

- **What you send while the agent works joins the running turn**, after
  its current tool calls, instead of waiting for the turn to end: Claude
  Code takes it as a user message mid-turn (checked against the CLI: one
  turn, one result), Codex through `turn/steer`. An agent that cannot
  leaves it queued. Its message shows as joining, then joined.
- **Escape stops the turn at once** (no dialog) and the queue goes on;
  interrupting no longer pauses it. A failed turn, a lost agent, an
  unanswerable request or a stopped robot still pauses it, saying why.
- **A question takes a typed answer**: while the agent waits on one,
  what you type is the answer; Claude Code's questions accept an answer of
  your own. Codex's async questions show as questions.
- **A turn the agent starts by itself** (a background task finished, a
  Codex goal) is shown as its own, instead of being dropped and then
  counted under your next message.
- **Quiet agents**: the status line says how long the agent has produced
  nothing, past a minute; a lost agent's reason carries the last line its
  CLI wrote (an unknown flag, say).
- **Codex output reads as a person reads it**: your own message echoed
  back and empty reasoning are not shown, and a command appears once.
- **A newer agent CLI is offered at start**, at most once a day: update
  now (rebuilds the sandbox image), skip, or skip until the next version;
  a CLI older than the minimum must be updated to start. `run` and `up`
  print one line. The CLIs' own updaters are off inside the sandbox
  (`DISABLE_AUTOUPDATER`, `check_for_update_on_startup=false`); a
  manifest names where its newest release is read (`latest_release`).
- **An interrupted motion is canceled**: `fjt_send.py` and
  `gripper_cmd.py` cancel their goal on SIGINT, SIGTERM or SIGHUP before
  exiting, as `ros2 action send_goal` does, instead of leaving the
  controller to finish the motion.
- **AGENTS.md in every workspace**: two sentences pointing any agent at
  README.md and machine.yaml, read on its own by Codex and by Claude Code
  in a project without a CLAUDE.md.
- **`/permissions`** switches between asking before each command that
  changes something and running without asking; `--permissions ask|auto`
  sets how a session starts, and `openrua session configure` does the same
  from a script. Claude Code switches its permission mode; Codex sends its
  approval policy with each turn. An approval names the command it would
  run.
- **`/model`** shows the model and changes it from the next request.
- **The agent's own commands**: what the agent lists (Claude Code's
  `/compact`, `/clear`, skills and custom commands) completes after `/` and
  goes to the agent as a message; Codex's `/compact` and `/clear` run as
  its own terminal runs them. `/help` lists them. After `/clear` the session
  follows the new native conversation.

### Real robots

- **`openrua connect`** (or "Connect a real robot" in the first-run
  menu) opens a session whose terminal reaches this computer's docker,
  network and devices, working in the robot's folder
  (`~/.openrua/robots/<name>/`: `robot.yaml`, `driver/`, `cameras/`,
  `CONNECT.md`). The agent asks before each command that changes
  something. Its `check.py <step>` asks OpenRUA to check a step on the
  host: `host`, `drivers`, `probe` (the graph into `robot.draft.yaml`),
  `config`, `robot` (started twice from `robot.yaml`, checked against
  it), `done`. The status line shows each step's result. When `done`
  passes, the session moves to the robot's own terminal and the same
  conversation goes on there; `robot.yaml` becomes the default robot.
- **Drivers on the same computer are found**: on the host network a
  sandbox speaks UDP only (Jazzy through `FASTDDS_BUILTIN_TRANSPORTS`,
  Humble through a Fast DDS profile), since containers there share the
  host's addresses but not its shared memory, and found nothing of a
  driver in another container. The same holds for `openrua probe --host`.
- A session's owner may add operations of its own
  (`POST /api/actions/<name>`), and may move the conversation to new
  resources between turns without pausing it.
- **A real robot's startup checks**: `backend.startup_checks` lists what
  has to be true before the robot starts (its controller answers, its
  devices are plugged in), each a command on this computer with the fix a
  person applies when it fails. They run before every session and in
  `openrua doctor`; one that fails stops the start and lists every failed
  check with its fix, and where to go on connecting the robot if it
  changed. `openrua connect` has the agent write them.
- **An interrupt stops the robot too**: Escape or `/interrupt` stops the
  agent's turn through its own CLI, as before, and now also asks the
  robot's motion actions (each arm's trajectory and gripper ports, the
  planner's move action) to cancel every goal, so a motion stops whatever
  started it: a tool, a script of the agent's own, or a command Codex
  keeps running after its turn is interrupted. The terminal says which
  actions canceled and names any that did not answer. It is not an
  emergency stop.

## 0.12.0

The sandbox and the session around a real robot, from the same first
real-robot day.

- **Agent CLI versions**: Claude Code 2.1.198 and later require Node 22;
  the manifests installed Node 20, so npm quietly installed 2.1.197,
  which exits on `--autocompact`. Both manifests install Node 22. An
  unpinned build installs the release current at build time (it no
  longer reuses an older build's layer), prints each CLI's version, and
  is checked under a candidate tag before it replaces the image. A
  manifest may name `min_version` (Claude Code: 2.1.251); a build, a
  session or a trial with an older CLI is refused with the rebuild
  command. `doctor`'s rebuild hint carries a configured pin.
- **A robot's own interfaces in the sandbox**: `backend.sandbox_install`
  installs what the agent needs to speak the robot (vendor messages, the
  ros2 control CLI) when `openrua build sandbox --robot <file>` builds
  that robot's own sandbox image; a ROS workspace under
  `/opt/openrua/overlays/<name>` is sourced in every shell.
- **A robot's own driver images**: `backend.build` names the images a
  launch command uses and the directories they are built from, as docker
  compose's `build:`; `openrua build driver --robot <file>` builds them.
  Starting a session builds what is missing: the driver images, and the
  robot's own sandbox when it declares `sandbox_install`. `doctor` checks
  the driver images.
- **What the agent can reach**: three limits, `internet` (`open` or
  `model-only`), `web_search` and `account_connectors` (the claude.ai
  connectors and synced skills and plugins, the ChatGPT apps). A session
  you start (`openrua`, `run`, `up`, `serve`) sets none of them: no proxy
  starts and the agent CLI behaves as you have it set up; a simulated
  robot's sandbox reaches the internet through `openrua-internet`, a
  network only OpenRUA's sandboxes join. A benchmark trial applies its
  config's `protocol.limits`, by default all three. `--internet`,
  `--[no-]web-search` and `--[no-]account-connectors` set them on any of
  these commands. Codex's `web_search` option is replaced by the limit.
  Codex's app-server runs with full access and no longer warns about
  bubblewrap.
- **Proxy**: started only for `internet: model-only`, one standing proxy
  per whitelist (`openrua-proxy-<hash of its hosts>`). The image holds no
  whitelist; each proxy is given its hosts when it starts, so switching
  agents needs no rebuild and never replaces a proxy in use. `openrua
  build proxy` takes no `--agent`; `doctor` only warns when the image is
  missing.
- **The session record**: `$HOME` is `/workspace` again, so whatever the
  agent writes under `~` stays in the record; ROS logs and the pip and
  npm caches go to `/home/robot` (`ROS_HOME`, `XDG_CACHE_HOME`,
  `npm_config_cache`). A session writes `provenance.json` (OpenRUA
  version and commit, the robot config's hash and repository commit,
  images, the agent CLI's version, the limits).
- **Real robots**: no saved simulator or benchmark default applies to a
  real robot. A launch command runs in its file's directory
  (`backend.cwd`); shutdown sends SIGINT, SIGTERM and SIGKILL with
  `backend.stop_timeout_s` between them. When the robot's resources end
  on their own, the session says so, records it and holds the queue.
- **Pauses say why** in the terminal and the browser; leaving the
  terminal says the session (and robot) keeps running and how to end it.

## 0.11.0

What a machine is now comes from its config, not from OpenRUA's
simulated Panda. Seen on the first real robot (a Franka Panda with a
Robotiq 2F-85, a wrist ZED Mini and a RealSense D405): the workspace told
the agent things true only of the simulated Panda, and the agent acted on
them.

- **Grippers** are declared by their command interface and what their
  driver does: the robot type writes `unit` (m or rad), `open`, `closed`,
  `max_effort` and, when known, the `joint` reporting the position; the
  simulator's embodiment or the real robot's instance writes
  `holds_partway` and `effort_honoured`. The old `open_m`, `closed_m`,
  `stops_at` and `control.gripper.open_threshold_m` fail with the rewrite.
  The bridge closes on a goal nearer `closed` than `open`, which keeps
  every bundled behaviour and works in either direction and unit.
  `gripper_cmd.py` takes `open`, `closed` or a position, and `--arm`.
- **Cameras** have types: `configs/cameras/<type>.yaml` (or a file next
  to the robot's) says where a camera's color, intrinsics and depth
  topics are and the depth encoding. A bare name is the simulation
  bridge's own type, `openrua-sim`; a real robot lists each camera with
  its type and mount, or `[]`. machine.yaml has one entry per camera with
  its exact topics; preflight checks every declared camera; `cam_snap`
  takes a camera name and gives up after a timeout; `px2world` reads the
  optical frame from CameraInfo and the world frame from `frames.world`.
- **Simulation stays with simulators**: a simulated machine's
  machine.yaml has a `simulation:` section (clock, camera rendering,
  trajectory tolerance, base command policy and queue depth), and
  observations no field can say are `notes:` by manifest kind, written by
  the robot type, simulator, embodiment, benchmark scene or real
  instance. robosuite's observations moved out of the generic docs into
  its file; a test keeps simulator words out of the workspace template.
- **The agent checks declared facts** and writes differences to
  `machine-discrepancies.yaml` for a person to fold into the config.
- **probe** reads typed topic and action lists, takes the arm's joints
  from its trajectory controller and the gripper's from its controller,
  the planning group and hand from the SRDF, and drafts one camera per
  CameraInfo; what only the owner knows stays TODO and fails validation.
- **Tools**: `fjt_send` prints the largest gap between target and joint
  state after the result, and takes `--arm`; `ik_move --at tcp` refuses
  to guess a missing TCP offset. A machine without joints lists no
  `/joint_states`.
- **Real robots**: the agent gets the proxy address its sandbox can reach
  (it got the container name, which does not resolve on the host
  network), and startup no longer announces a booting simulator.
- **Panda defaults removed**: a robot names its own frames, simulator
  bodies and planning group; a missing one is an error, not a silent
  Panda name.
- Startup evidence regenerated for all four offered environments; the
  check now also asserts the declared gripper joint.

## 0.10.21

- Fix ROS discovery inside a container on Humble images (robosuite's own scenes, CaP-Bench, RoboCasa365). The Fast DDS initial-peers profile that stands in for `ROS_STATIC_PEERS` replaced the default multicast peer, so processes in one container never discovered each other: in the robot container, robot_state_publisher and move_group received no `/joint_states` or TF from the bridge, which left the TF arm chain unpublished and MoveIt's floating base at the origin; in the sandbox, two of the agent's own nodes could not talk. The profile now keeps the multicast peer next to the static ones. Jazzy images (LIBERO-PRO) were not affected.
- With MoveIt now following the bridge's world->panda_link0 transform, every robosuite scene plans in `world`: the simulator declares `planning.planning_frame: world`, as does RoboCasa365 for its PandaOmron. LIBERO-PRO's own override from 0.10.20 is no longer needed and is removed. On Humble scenes this changes what agents see: IK poses and FK results are in world coordinates instead of the arm base's, and TF lookups to the hand now succeed.
- The workspace perception docs no longer warn that the TF arm chain may be disconnected; the IK tool's comment points to `machine.yaml` for the planning frame.
- The startup check now also compares the hand's FK pose with its TF pose in the declared planning frame (within 5 mm), which needs a connected TF chain. With the old profile it fails on CaP-Bench waiting for TF from `world` to `panda_hand`; all four offered environments pass and their evidence is regenerated with both agents.

## 0.10.20

- A benchmark can now set the frame MoveIt plans in for the robots it loads (`scenes.robots.<type>.planning.planning_frame`), over the robot type's value. LIBERO-PRO sets `world`: in its world MoveIt reads IK poses and returns FK in world coordinates, while CaP-Bench and RoboCasa365 use the arm base. Previously every Panda declared `panda_link0` and the workspace docs said the planner always works in the arm base, so on LIBERO-PRO an IK request in arm-base coordinates aimed at the wrong point and often failed with -31.
- The workspace action docs no longer state a fixed planning frame; they point to `machine.yaml` `planning.planning_frame`. Checked live on LIBERO-PRO: the FK pose of the hand matches its TF pose in the declared frame to 0.1 mm. Startup evidence regenerated for all four offered environments.

## 0.10.19

- Style the terminal the way a coding agent's own terminal reads: the header names the session and the robot it is on (robot, simulator, benchmark scene, agent and model, task), your instructions carry a `❯`, agent text and each tool call carry a `⏺` coloured by phase (working, completed, failed), a tool call shows its command and the first three lines of its output folded under `⎿` with a count of the rest, and `/tools` opens or folds the whole output. Long lines are cut to the terminal width while folded and wrapped when opened.
- The status line shows an animated `Working…` with elapsed time, a green `✓ Ready`, a yellow `‖ Paused` with the next step, and yellow warnings for a disconnected agent, pending questions and an unconfirmed send; a failed, interrupted or uncertain turn is marked in colour under the instruction it belongs to. The shortcut line is dimmed and turns yellow while the service is unreachable.
- Sessions started from this version record the robot, simulator, benchmark, scene and task names in their retained state so the header can show them; older sessions show the header without them. Colours use the terminal's own 16-colour palette.

## 0.10.18

- Open TUI dialogs (`/files`, `/tools`, `/queue`, `/questions`, confirmations, text editing and file previews) below the transcript in the input's place, the way a coding agent's own menus appear, instead of in a framed overlay drawn over the conversation. Nothing is covered; Escape closes the open dialog and never doubles as interrupt; a menu that opens an editor returns when the editor closes.
- Text previews now scroll: a window of lines sized to the terminal, moved with the arrow, page, Home and End keys, with the position shown. Previously the whole file was drawn and cut at the overlay height, and the scroll keys did nothing on the main screen.
- Checked with the Node test suite against the real local service and in a pseudo-terminal with the actual TUI. Session, API and workspace reading are unchanged.

## 0.10.17

- Draw every TUI dialog (`/files`, `/tools`, `/queue`, `/questions`, confirmations, text editing and file previews) inside a bordered frame padded to its full width. Pi composites an overlay only over the columns it covers, so the conversation showed through on both sides of an unframed dialog and the two were hard to tell apart. The frame caps its own height so its bottom edge is always drawn.
- Checked with the Node test suite against the real local service and in a pseudo-terminal with the actual TUI. No session, API or workspace-reading behavior changed.

## 0.10.16

- Parse robot descriptions returned by native ROS topic echo by removing its trailing document separator before XML parsing. Previously, valid published URDFs could silently leave joint and frame fields empty in the draft.

- Preserve the explicitly selected ROS distribution in standalone robot profile drafts instead of always writing Jazzy. Read-only graph probing no longer depends on a running model proxy; host, static-peer and discovery-server connection settings still use the shared native network configuration.

## 0.10.15

- Stop robot profile probing when a topic, action or service query fails, preserving the native error or timeout output instead of printing an incomplete profile as a successful result. Missing optional robot descriptions still produce a draft with TODO fields.
- Clarify ROS domain and distribution selection when probing, and correct the real-robot example to distinguish interactive graph readiness from trial preflight checks.

## 0.10.14

- Keep tool invocation details alongside returned output in the TUI and browser. Completed calls no longer replace their command or arguments with the result; empty outputs and later status updates retain the original details. Reconnecting reconstructs both sections from recorded events.

## 0.10.13

- Show a temporary TUI connection loss as disconnected and retrying instead of leaving a stale working/ready status. Restore the current session status after reconnection without clearing drafts or local notices, duplicating tool output, or resending instructions.

## 0.10.12

- Show Claude Code tool results as completed or failed, with expandable native text output, instead of leaving finished calls labeled as started. Results remain tied to the original tool and turn; duplicate or unrelated results cannot complete a later call.
- Image payloads stay in native records rather than being printed as encoded data. Tool completion does not imply that the agent turn or robot task succeeded. Existing stored conversation events are not rewritten.

## 0.10.11

- Send the configured hand link in IK starter-tool requests, so hand and TCP targets refer to the documented frame instead of an implicit planning-group tip. Native Panda IK/FK checks reproduce a 45-degree hand orientation error with the implicit tip and zero error with the explicit hand link.
- This reviewed starter-tool fix updates newly seeded workspace templates. Existing workspaces and historical releases remain unchanged; no grasp strategy or agent workflow is added.

## 0.10.10

- Decode depth pixels using the native ROS image encoding, byte order, and row stride. The coordinate starter tool accepts REP 118 float meters and raw integer millimeters, rejects invalid layouts and depths, and preserves raw saved arrays with accurate unit documentation.
- This reviewed starter-tool and unit-documentation fix updates the template for newly seeded workspaces. Existing workspaces, recorded trials, and historical release templates remain unchanged. Camera topic/frame naming and aligned depth calibration are still required; no perception strategy or agent workflow is added.

## 0.10.9

- Real-driver trial preflight no longer requires the simulator clock or forbids unlisted gripper interfaces. Explicit empty camera lists omit camera checks and the generated camera entry in `machine.yaml`; declared joint and control interfaces remain checked.
- Clarify that interactive startup waits for graph visibility while trial preflight checks declared interfaces. Software ROS checks cover valid profiles and rejection of an incorrect joint name; the bundled benchmark checks and manifests remain unchanged.
- Save robot resource metadata as complete snapshots. Concurrent readers retain the previous record until replacement, and failed writes preserve it instead of leaving an empty or partial state file.

## 0.10.8

- Pass the selected ROS domain to real-driver launch commands on the host and in driver containers, matching the sandbox instead of inheriting a different host default. Externally managed drivers remain unchanged.
- Document native Fast DDS configuration for same-host containers with separate IPC namespaces, with a UDP profile and bounded software-graph lifecycle verification.

## 0.10.7

- Offer the Panda / robosuite / LIBERO-PRO combination in guided setup after checking its default scene, camera and joint streams, and trajectory interface in freshly built Jazzy images. Selection and image preparation use the existing configuration contracts; no task-specific control logic is added. The evidence covers environment startup, not model-task success or every benchmark task.

## 0.10.6

- Publish session connection files atomically so reconnecting clients cannot read incomplete JSON. Keep credentials private, refuse to overwrite another session owner, and clean up unsuccessful writes.
- Include the ROS readiness deadline fix from 0.10.5, whose release validation caught this connection-file race before publication.

## 0.10.5

- Enforce the real-backend ROS readiness deadline even when a probe command stalls. Stop the owned probe process group, preserve its diagnostic output, and cap polling sleeps to the remaining time. This does not stop an externally owned robot driver.

## 0.10.4

- Honor native multiple-choice questions in the keyboard TUI. Toggle choices with Space or Enter, optionally write a custom answer when the native agent allows it, and confirm before sending. Escape cancels without replying.
- Normalize Kimi question capabilities into the existing shared `multiple` field and an optional `allow_other` flag. Browser forms also support explicit custom answers. Native permission decisions remain single choice and are never answered automatically.

## 0.10.3

- Parse the documented `ik_move.py --at tcp` option correctly, keeping its value separate from pose coordinates and duration. Clarify that targets use the planning frame.
- Return a nonzero status from `fjt_send.py` for rejected, aborted or failed trajectories, preserving the native diagnostic instead of reporting shell success. IK forwards that exit status.
- Correct workspace documentation: benchmark completion ends trial resources, while a shared-chat reply leaves the session available for subsequent messages.
- These intentional starter-tool and documentation fixes change the workspace template hash for newly seeded workspaces. Existing workspaces and recorded trial hashes are untouched; use the original release or commit to reproduce an earlier template. No grasp strategy or agent workflow is added.

## 0.10.2

- Keep CLI status and event inspection available when a shared service has stopped or its endpoint is invalid. Read retained records without changing the queue or resources, warn that recorded execution state may be stale, and preserve JSON output and event cursors. Sending, resuming and ending still require the live service.

## 0.10.1

- Resolve manually entered agent manifests in keyboard setup before offering authentication choices. External API-capable plugins can be configured without borrowing another agent's key path; switching agents preserves unsaved model and authentication choices until the user explicitly saves.

- Preserve custom benchmark file paths during session startup instead of replacing them with an internal benchmark name.
- Open the default TUI for explicitly configured custom robot, simulator or benchmark profiles after validation and preparation, without forcing them into the tested bundled-environment menu. Saved custom defaults follow the same path; guided setup remains restricted to tested bundled combinations.

## 0.10.0

- Add an experimental external Kimi Code 2.1.1 plugin with a pinned installation and explicit Moonshot API profile. Shared chat uses the native local server and prompt-correlated transcript states, retaining OpenRUA's existing queue, permission prompts, exact-session resume and failure pause. Native container lifecycle checks use a synthetic local provider; subscription reuse and robot-task performance remain unvalidated. Kimi is not added to default setup options.

## 0.9.1

- Read a fresh session snapshot after message submission instead of reusing an earlier in-flight poll. Newly queued instructions now appear immediately in the queue menu, and a failed poll does not block the next refresh.
- Document a bounded real-provider file task through the experimental ZCode container plugin. Robot-task performance and subscription reuse remain unvalidated.

## 0.9.0

- Add keyboard authentication selection to setup. Native CLI login remains the default; API billing requires an explicit choice and a private key file path. Choices stay separate per agent, unsupported plugins do not offer API mode, and invalid selections cannot save defaults or start resources.
- Preserve explicit user authentication when composing benchmark configurations. Schema-generated native defaults no longer silently replace an API choice; authentication actually written in a benchmark retains its documented precedence.

- Add an experimental ZCode external plugin example with a pinned official source build, explicit BigModel API profile, and native shared-conversation adapter. Container lifecycle checks use a local model fixture; subscription reuse and robot-task performance remain unvalidated. The example is not a bundled setup option.

## 0.8.1

- Support explicit Claude Code API authentication through `config set --agent claude-code --auth api --api-key-file ...`. The native CLI reads a private session-local key through `apiKeyHelper`; subscription credentials are not mounted, and keys stay out of launch arguments. The same configuration works for shared chat, native terminal sessions, and benchmark trials. Native login remains the default.

## 0.8.0

- Add explicit Codex API authentication through `config set --auth api --api-key-file ...`, shared by interactive and benchmark startup. Native login remains the default; API profiles omit subscription credentials, and unsupported plugins reject the selection before updating defaults. Local checks do not spend tokens or validate quota.
- Let agent plugins prepare native profiles and inspect local login material for discovery and doctor. Custom layouts can share credential and refresh-lock directories without adding vendor-specific logic to the runner. Local checks do not authenticate online or switch billing sources.

## 0.7.1

- Retain the shared service endpoint when abnormal exit cannot confirm resource shutdown. This keeps `clean`, including `clean --all`, from deleting the session's files until the remaining resources have been inspected. Successful shutdown still removes the endpoint and preserves the conversation and workspace.
- Add a lifecycle regression covering two clients, queued execution, reconnection, shutdown, offline file access and explicit deletion, using real local HTTP, SQLite and a controlled child process. This verifies session behavior, not model or physical robot performance.
- Include the development-only ZCode conversation adapter and its documented validation limits. Kimi Code and ZCode are not yet selectable product integrations.

## 0.7.0

- Browse retained workspace files with `/files` after opening an ended or unavailable conversation through `/resume`. The same keyboard directory, text and image viewers now work without restarting the robot or agent.
- Use the recorded workspace path, including external directories. Missing files leave the transcript accessible. The temporary local viewer rejects mutation requests, preserves the existing file access restrictions, and closes when the history screen exits.
- Verify archived file browsing through a real terminal in the isolated distribution check, alongside Python and terminal integration tests. Image rendering still depends on terminal support; saved observations are not a live video feed.

## 0.6.0

- Make the Pi keyboard terminal the default for `openrua`, setup, history, and `chat --tui`. Remove the Textual frontend and dependency. Ordinary installation now includes the terminal runtime; `[pi]` and `[tui]` remain compatibility extras.
- Use Enter to send, Ctrl+J for newlines, slash commands for tools/files/queues, and keyboard selectors for configuration and confirmations. Ctrl+D on an empty input detaches. No mouse is required.
- Discover native Claude Code and Codex file-based logins and create absent OpenRUA profile aliases on launch, without copying OAuth credentials. Explicit profile paths, native environment overrides, and existing account directories are preserved. Doctor shows the effective credential source; cached files do not prove that a login is still valid.
- Avoid repeating a native error when it arrives as both a message and a final turn result.
- Keep browser/plain CLI startup independent of terminal runtime checks. Synchronize installation, tutorials, keyboard references and package validation with the new default.

## 0.5.0

- Browse saved workspace files from either terminal frontend without sending agent instructions. Pi uses its keyboard selector, scroll view and image component; the default TUI previews text and shows image metadata. Both reuse the existing read-only workspace API.
- Preserve the chat draft while browsing. Refresh changed files, report missing/oversized paths, and reject symlinks through the existing server checks. Image rendering depends on terminal support; offline history remains transcript-only.
- Check Docker daemon access and reject missing, empty or non-file credentials in readiness reports. Login-file presence is explicitly separate from authentication and quota. Existing images and shared proxy containers are not automatically replaced.
- Give README readers separate TUI and benchmark/demo entry points. Explain How it works through the paper’s workspace-as-harness, file-I/O perception and coding-based control.

## 0.4.3

This release includes the startup and diagnostic changes prepared for 0.4.2.
The 0.4.2 publishing gate caught a test that relied on a pre-existing Docker
network; no 0.4.2 package was published. Proxy integration tests now create
and clean up isolated networks and use unique container/image names, so they
exercise the same behavior on a clean CI runner and a development machine.


### Fixed

- Proxy startup verifies Docker network membership before returning a URL and
  preserves Docker's original errors on failed startup or attachment. An
  existing proxy is never replaced to repair another session's startup.
- Readiness checks inspect the standing proxy container separately from the
  image, so rebuilding an image cannot hide its older running policy. Warning
  repair guidance is now visible in text reports as well as JSON.
- Missing Docker labels use legacy aggregate metadata correctly instead of
  interpreting `<no value>` as an agent hash.

### Changed

- Guided setup adds Panda in the native robosuite Lift scene and in CaP-Bench,
  each backed by real startup, reset, camera, joint-state and no-op trajectory
  checks. The existing RoboCasa365 entry remains available.
- Environment validation accepts native scene names without a benchmark task
  instruction, and checks agent executables through their plugin manifests
  rather than a fixed list of CLI names.
- README and GitHub citation metadata link to the published OpenRUA paper.

### Scope

The new environment records cover their default scenes with existing agent
sandbox images. They do not establish model task success, every benchmark task,
physical deployment, or an entirely fresh machine installation. Shared-proxy
policy changes still require an explicit maintenance step after affected
sessions end; ordinary startup does not interrupt them.

## 0.4.1

### Fixed

- Robot, simulator and benchmark fields now filter linked, tested combinations
  instead of offering an arbitrary cross-product of registered names. Recorded
  RoboCasa365 startup, reset, ROS observations and no-op trajectory checks back
  the initial guided entry. Other profiles remain available through explicit CLI options.
- Invalid setup selections cannot overwrite saved defaults. Clearing a benchmark
  stays cleared during checks and subprocess launch. Mismatched benchmark and
  simulator selections fail before touching resources.

### Changed

- Prepare and start builds missing simulation, sandbox and proxy images for the
  selection, with progress, retained logs and retry. Existing images are reused.
  Save and check remains read-only with respect to images and running resources.
- The same preparation path serves Textual, Pi, and root CLI/browser launches.
  Low-level up, run and serve retain explicit preparation behavior.

## 0.4.0

### Added

- Optional Pi terminal, started with `openrua --tui pi` after installing
  `openrua[pi]`. The default Textual frontend remains available.
- Keyboard configuration and searchable `/resume` history, reusing the existing
  configuration, readiness checks, background launcher, and retained sessions.
- Pi slash commands for tool expansion, queue edits/withdrawal, agent questions,
  interruption, queue continuation, explicit end, and client detachment.
- Prepared frontend resources and licenses in wheels and source archives;
  the optional Node runtime is installed by pip, without a system Node requirement.
- Build manifest verification and clean-install pseudo-terminal checks alongside
  session integration tests and the existing Python/browser CI jobs.

### Scope

Pi uses the shared execution service and native agent plugins; it does not add
an agent loop. Linux x86_64 packaging, simulated keyboard input, and local
pseudo-terminal startup are verified. Real-terminal IME/SSH behavior and live
model/robot interaction still need user trials. Secret inputs and resolving
unknown execution results use existing clients. Agent/model hot switching
and persistent multi-panel layouts remain deferred.

## 0.3.0

### Added

- Unnamed product launches create unique session IDs automatically. History
  titles use the first user instruction, without an extra model call.
- `/resume` searches history from both the setup form and chat. `--resume [ID]`
  opens the selector or a specific conversation; live sessions reuse their
  existing execution owner and native conversation.
- Ended or unavailable conversations can be inspected read-only in the TUI.
  Selection never starts a robot, replays a task, or changes retained state.

### Changed

- Bare `openrua` now prepares a new session instead of implicitly reconnecting
  to a fixed default name. Use `/resume` or `--resume` to find earlier work.
- Explicit `--name` and existing named directories remain compatible. Native
  `run`, foreground `serve`, and administrative subcommands keep their defaults.
- README, installation, terminal guide, walkthrough, and CLI reference document
  the same new-session and history behavior.

### Scope

Automated checks cover legacy history, distinct generated IDs, live-owner
reconnection, stale endpoints, read-only transcripts, and local slash commands.
The web UI still represents one live session; select it with
`openrua --gui --resume ID`. Restarting ended execution owners, renaming history
entries, and migrating native agent conversations remain outside this release.

## 0.2.0

### Added

- `openrua` opens a setup form for a new shared session or reconnects to the
  named running session. `--gui` and `--cli` select browser or plain text chat.
- `--setup` edits the same validated `config.yaml` used by `config set` and
  direct file edits. The form reports missing preparation without automatically
  building images, logging in, or submitting robot tasks.
- Background startup reuses the existing `serve` command, waits for its API,
  retains startup logs, and prevents competing startup attempts.

### Changed

- TUI dependencies are included in the default installation. The `[tui]` extra
  remains accepted for compatibility. Administrative commands still import the
  interface lazily.
- Configuration updates preserve unrelated fields and use a lock and atomic
  replacement; `config set --bench null` clears the benchmark default.
- README, installation, shared-session walkthroughs, command reference, and
  architecture documentation describe the same startup and configuration paths.

### Upgrade and scope

Install with `pip install -U 'openrua==0.2.0'`. Existing native-terminal and
benchmark commands keep their behavior. Closing a shared client leaves its
service running; explicitly end it to stop resources. Retained names are not
reused automatically, and startup flags cannot reconfigure a running session.

Tests cover shared configuration, headless setup interaction, real child-process
startup and API readiness, duplicate startup, and retained failure output.
These additions do not establish physical-hardware reliability or execution-owner
crash recovery. In-session agent/model switching remains outside this release.

## 0.1.0

This release adds shared robot conversations and an optional terminal UI while
preserving the original native-agent terminal and benchmark commands.

### Added

- A local session service (`openrua serve`) with a persistent message queue,
  retained execution events, and native conversation adapters for Codex and
  Claude Code. Multiple clients can submit instructions to the same queue.
- Browser chat with streamed replies, tool output, pending agent questions,
  queue editing and withdrawal, interruption, and explicit queue resumption.
- Read-only browsing and previews of saved workspace images and files.
- An optional chat-first TUI (`openrua chat --tui`), using the same session API.
  Install it with `pip install 'openrua[tui]==0.1.0'`.
- Plain CLI chat and session-management commands for reconnection, status,
  queue operations, and reconciliation of unknown execution outcomes.
- Real-robot task startup without requiring a benchmark configuration.

### Changed

- Ending an interactive session retains its workspace and records. Deletion is
  a separate explicit operation. Closing a shared chat client leaves the
  execution service running.
- Interrupted, failed, or unknown execution pauses the queue and retains pending
  instructions. Restarting that queue requires an explicit action.
- Resource shutdown is handled by the process that owns the robot and native
  agent. Partial shutdown failures remain visible and retryable.
- The default Codex reasoning-effort setting is now high, matching the existing
  Claude Code manifest. Explicitly configure effort when reproducing older runs.

### Fixed

- Native terminal entry points retain the selected agent plugin and version.
- Resumed trials that exhaust their execution budget are no longer discarded
  as quota-limited runs.
- Workspace starter-tool indexing and its corresponding template fingerprint
  are consistent. Existing trial fingerprints remain unchanged.
- CLI reference generation is consistent across supported Python versions.

### Upgrade and validation

Install with `pip install -U 'openrua==0.1.0'`, or use the `tui` extra above.
Run `openrua doctor` for your chosen configuration and rebuild any images it
reports as stale. Follow `examples/shared-session.md` for a complete shared
session walkthrough.

Automated checks cover supported Python versions, the shared HTTP/SQLite
session lifecycle, browser interactions, and terminal interactions. A live Codex
simulation conversation has been exercised. Claude Code's conversation adapter
has protocol and error-path coverage, but successful live model turns have not
yet been validated. Physical hardware has not been validated.

Shared interfaces remain experimental within this regular release. The TUI
currently attaches to a separately started service; a single-command default
TUI, in-session agent/model switching, a TUI file panel, and a dedicated mobile
app are not implemented. Client reconnection does not provide execution-owner
crash recovery or a native-agent TUI handoff.

Older releases are recorded in [GitHub Releases](https://github.com/terminalworld/OpenRUA/releases).
