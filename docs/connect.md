---
summary: Connect a real ROS 2 robot with your coding agent doing the work, step by step
read_when:
  - You have a real robot with a ROS 2 driver and no OpenRUA config for it yet
  - You want to know what `openrua connect` does and what it checks
---

# Connect a real robot

Start in your robot project directory, or select one with `--workspace DIR`.
The first use asks whether you trust that directory. Driver setup still has
its own robot folder; once connecting succeeds, the conversation moves to the
project you selected and saves that robot as its project default.

```bash
openrua connect my-arm            # or: openrua, then "Connect a real robot"
```

The setup menu's entry connects a robot named `my-robot`, in
`~/.openrua/robots/my-robot/`. When Docker is not ready or your agent is
not signed in yet, OpenRUA offers to set that up first
([install.md](install.md#docker)).

Before opening the connecting terminal, OpenRUA builds its sandbox image if
missing, using your selected agent and ROS distro (`--distro jazzy` by default,
or `--distro humble`). Build output is shown as it runs; failure logs remain
under `~/.openrua/preparation/`. Retry the same command or menu entry after
fixing a build error; completed images are reused. No robot configuration is
needed for this initial preparation.

Before reusing an existing sandbox, preparation checks that its selected agent
CLI is available and satisfies the configured version. An image built only for
another agent is rebuilt through the same preparation flow. Connecting from the
setup menu saves the selected agent, model and authentication without requiring
a known robot or replacing your existing robot/simulator defaults.

The robot's own terminal may use a different ROS distro from the connecting
terminal. The `robot` check verifies the selected agent in that actual sandbox
image and rebuilds it when needed before either test start. A bare Humble image
created for graph probing, for example, is not sufficient to run Codex. Lower
level startup also refuses a missing CLI and prints the matching build command.

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
the robot's folder, `~/.openrua/robots/my-arm/` (`--dir` names another;
an existing folder goes on from what is in it). Your coding agent finds
out what the robot is, builds and starts its ROS 2 drivers and writes
the robot's config, as an engineer would with you beside the robot. It
asks you for what only a person can do (power, brakes, the vendor's web
interface, the emergency stop, cables, `sudo`), and asks before each
command that changes something: the session starts with
`--permissions ask` (`/permissions` switches).

## The folder

| File | Holds |
|---|---|
| `robot.yaml` | the robot's config ([your-own-robot.md](your-own-robot.md)): how its drivers start and stop, and what the machine is |
| `driver/` | the drivers' Dockerfiles and launch script; `backend.build` names the images |
| `cameras/` | one camera type per camera model |
| `CONNECT.md` | the steps, written for the agent |

`.openrua-checks/` holds each check's output and is ignored by git. The
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
