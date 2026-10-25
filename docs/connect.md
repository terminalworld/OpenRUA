---
summary: Connect a real ROS 2 robot with your coding agent doing the work, step by step
read_when:
  - You have a real robot with a ROS 2 driver and no OpenRUA config for it yet
  - You want to know what `openrua connect` does and what it checks
---

# Connect a real robot

Start in your robot project directory, or select one with `-C DIR`.
The first use asks whether you trust that directory. The agent works in this
project throughout connection and later operation.

```bash
mkdir my-robot-project
cd my-robot-project
openrua connect
```

OpenRUA creates missing `robot.yaml`, `driver/`, `cameras/` and `CONNECT.md`
in the project root. Existing files are retained. `--dir robot` places the
adaptation in a project subdirectory while keeping the whole project as the
agent's workspace. The setup path **Real robot → Connect a new robot →
Connect robot** uses the same project directory. Docker setup and native sign-in
are offered when needed.

Before opening the connecting terminal, OpenRUA builds its sandbox image if
missing, using your selected agent and ROS distro (`--distro jazzy` by default,
or `--distro humble`). Build output is shown as it runs; failure logs remain
under `~/.openrua/preparation/`. Retry the same command or menu entry after
fixing a build error; completed images are reused. No robot configuration is
needed for this initial preparation.

Before reusing an existing sandbox, preparation checks that its selected agent
CLI is available and satisfies the configured version. An image built only for
another agent is rebuilt through the same preparation flow. Connecting from the
setup menu uses the selected agent and model and saves its authentication
without requiring a known robot or replacing this project's environment choices.

After connecting, maintain the driver through the same conversation. Ask your
agent to apply changes or use `/robot` for status, logs, builds and restart.
The agent can call `python3 /openrua-runtime/robot.py --help`; its source folder
is available at `/adapter/`. Driver updates retain the agent environment when
possible; necessary tool changes hand over the conversation internally.
See [connection maintenance](sessions.md#maintain-the-robot-in-the-same-conversation).

To add a peripheral later, keep its Dockerfile and launch code in this same
project. Add its image directory to `machine.backend.build` in `robot.yaml`
and update `machine.backend.launch` (or its project script) to start it.
Use the explicit `{images[image-name]}` slot to pass a declared image to the
launch script, including the device arguments its native driver needs.
Update the declared camera topics and any project startup checks as needed.
Then restart through `/robot` or the agent's maintenance command. Restart
reads the edited project configuration and builds missing declared images
before replacing the running connection. A failed build leaves the running
driver in place. Explicit **Build** also rebuilds declared images when needed.
The launch script owns its child processes and releases their resources
when stopped. See [launch ownership](your-own-robot.md#bringing-it-up) for the lifecycle.

The robot's own terminal may use a different ROS distro from the connecting
terminal. The `robot` check verifies the selected agent in that actual sandbox
image and rebuilds it when needed before either test start. A bare Humble image
created for graph probing, for example, is not sufficient to run Codex. Lower
level startup also refuses a missing CLI and prints the matching build command.

When an existing image lacks the selected CLI, preparation adds the install
layer to that image, preserving its custom tools, users and runtime settings.
It checks a separate candidate before replacing the image tag. Failed builds
leave the working image in place. The same operation is available explicitly:

```sh
openrua build sandbox --base-image my-robot-terminal --tag my-robot-terminal --agent codex
```

Docker failures starting a check container are reported as runtime errors;
they do not trigger a rebuild. Timed-out checks remove only their own temporary
container. Inspect Docker access and storage before retrying.

If a build reports a missing `buildkit/.../cachemounts` directory, inspect the
Docker data location and builder state on that computer:

```sh
docker info --format 'Root={{.DockerRootDir}} Driver={{.Driver}}'
docker buildx ls
docker ps
```

Check that the data location is mounted and writable. If Docker is managed by
systemd and no running containers need to be preserved, reinitialize the daemon
with `sudo systemctl restart docker`, then retry
`openrua build sandbox --distro jazzy --agent codex`. Use your selected distro
and agent. OpenRUA reports this failure and retains its build log; it does not
restart Docker or change its storage directories automatically.

This opens a session whose terminal reaches what connecting a robot
needs: this computer's docker, its network (the robot's controller and
ROS 2 graph) and its devices (read-only, under `/host/dev`), working in
the coding project. The initial message names the adaptation directory. An
existing project continues from its maintained files. Your coding agent finds
out what the robot is, builds and starts its ROS 2 drivers and writes
the robot's config, as an engineer would with you beside the robot. It
asks you for what only a person can do (power, brakes, the vendor's web
interface, the emergency stop, cables, `sudo`). The session uses the selected
agent's native permission default; `/permissions` lists its supported modes.

## The folder

| File | Holds |
|---|---|
| `robot.yaml` | the robot's config ([your-own-robot.md](your-own-robot.md)): how its drivers start and stop, and what the machine is |
| `driver/` | the drivers' Dockerfiles and launch script; `backend.build` names the images |
| `cameras/` | one camera type per camera model |
| `CONNECT.md` | the steps, written for the agent |

`.openrua/connect/<session>/` in the project holds each check's output. The
folder is self-contained: another lab with the same robot uses it as it
is (`openrua --robot <folder>/robot.yaml`).

## Steps and their checks

The agent runs `python3 /openrua-connect/check.py <step>` in its
terminal; OpenRUA runs the check on this computer and prints the result.
The terminal's status line shows each step's last result.

| Step | Check |
|---|---|
| This computer | `host`: docker usable, disk space, the real-time limit, devices present; what fails comes with its fix |
| What the robot is | you confirm what the agent found (make, model, firmware, gripper, cameras) |
| Drivers | `drivers`: every image under `backend.build` builds |
| Together | `probe` reads the running graph into `robot.draft.yaml`; `config`: `robot.yaml` loads with no TODO left |
| The robot from its config | `robot`: missing images built, the robot started from `robot.yaml` twice and stopped in between, the live graph checked against what `robot.yaml` declares |
| First motion | `done` passes once every check above has; the session then moves to the robot's own terminal (no docker there) and the conversation goes on: with your word that the emergency stop is at hand, the gripper opens and closes once and one joint moves a few degrees |

After the move, `robot.yaml` is your default robot, so `openrua` starts
it, after its own startup checks (`backend.startup_checks`: the
controller answers, the devices are plugged in), which the agent writes
while connecting it. Only robots with a ROS 2 driver are in scope.
