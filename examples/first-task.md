---
summary: Hand the simulated Panda's coding agent a task, as a recorded run or in its own terminal, and watch what it does
read_when:
  - You have installed OpenRUA and want to see it work once
  - You want to know what the agent actually sees and types
---

# First task in simulation

This walkthrough gives a simulated Franka Panda's coding agent a task in two
ways: with `openrua exec`, where the agent works on its own and the run is
recorded, and in the agent's original terminal (`openrua up`, then
`openrua agent`). For OpenRUA's own TUI or browser, follow
[Your first shared robot session](shared-session.md) instead.

[Install](../docs/install.md) first; prepare the LIBERO-PRO simulator, the
sandbox for your selected agent and, for `exec`, the proxy
(`openrua build proxy`). Check them with
`openrua doctor panda --bench libero_pro`. Initial image builds take additional
time and download the simulator dependencies and assets.

## 1. One command

```bash
openrua exec panda --bench libero_pro
```

The robot container boots its scene and MoveIt (about a minute), the
sandbox is created with the workspace seeded, and the agent gets the
scene's task, "open the bottom drawer of the cabinet", the way `codex exec`
or `claude -p` take one: it works on its own until it is done or its wall
clock (`--wall-clock-min`) ends, and then the robot powers off. The
benchmark's own predicate judges success. The agent reaches only its model
service unless `--internet open` says otherwise. The run is recorded under
`runs/libero_pro/exec-<UTC time>/trials/libero_goal_task-0/seed0/`: the
transcript, `commands.sh`, `result.json`, `provenance.json` and the workspace
the agent left ([where it lands](../docs/running-experiments.md#where-it-lands)).

The rest of this page opens the agent's original terminal on the same scene,
in its two-terminal form, which keeps the robot up between agent visits.

## 1a. Bring the robot up

```bash
openrua up panda --sim robosuite --bench libero_pro
```

The command stays in the foreground:

```
[up] robot     panda: Franka Emika Panda, simulated by robosuite (ROS 2 jazzy)
[up] scene     libero_pro / libero_goal_task #0: "open the bottom drawer of the cabinet"
[up] agent     claude-code (claude-opus-5)
[up] sandbox   openrua-sandbox, on the robot's ROS 2 graph

     openrua agent --name openrua            # your coding agent, on the robot
     docker exec -it -u robot -w /workspace openrua-sandbox bash   # or you

     Ctrl-C here powers the robot off and keeps the workspace.
```

`--task-suite` and `--task-id` pick another scene from the benchmark;
`--init-state` another initial layout of the same scene. If no benchmark is
selected on the command line or in saved defaults, robosuite loads its native
`Lift` scene, a table and a cube.

## 1b. Hand the agent the task

In a second terminal:

```bash
openrua agent "open the bottom drawer of the cabinet"
```

This opens the agent `up` was configured with (Claude Code by default,
`--agent codex` for Codex) inside the sandbox, as user `robot` in
`/workspace`, with the sentence as its opening message. Nothing else is
injected: no tool list, no API, no planner. The agent reaches the
internet as it does on your machine; `openrua up --internet model-only`
limits it to its model service, through the proxy image `openrua build proxy`
makes ([what the agent can reach](../docs/install.md#what-the-agent-can-reach)).

## 2. What the agent sees

```
/workspace
  AGENTS.md            points any agent at README.md and machine.yaml
  README.md            start here: the map of the docs, a 30-second start
  machine.yaml         this robot's facts: joints, limits, frames, ports, cameras
  docs/
    10-machine.md      the robot and its ROS 2 graph
    20-perception.md   cameras, depth, pixel to world
    30-action.md       trajectories, the gripper, MoveIt, servoing
    40-patterns.md     how a task usually goes
  tools/
    perception/        cam_snap.py, px2world.py
    action/            fjt_send.py, gripper_cmd.py, ik_move.py, base_goto.py
```

`machine.yaml` is generated from the resolved robot configuration. The agent
can inspect the live graph to discover available interfaces. Recorded runs
(`exec`, `bench`) also run preflight checks before the agent starts; `up` does
not perform that trial-level validation. The tools are ordinary `rclpy` scripts the agent may read, copy or
ignore.

## 3. What a session looks like

The first commands of a real session (Codex, from a recorded run's
`commands.sh`):

```bash
sed -n '1,240p' README.md && sed -n '1,260p' machine.yaml
sed -n '1,260p' docs/10-machine.md && sed -n '1,320p' docs/20-perception.md
ros2 topic list && ros2 action list && ros2 service list && ros2 node list
timeout 12s ros2 topic echo /joint_states --once
python3 tools/perception/cam_snap.py agentview /workspace/agentview.png
python3 tools/perception/cam_snap.py robot0_eye_in_hand /workspace/robot0_eye_in_hand.png
```

From there it is ROS 2: a pixel-to-world lookup, an IK move or a
`FollowJointTrajectory` goal, a `GripperCommand`, another snapshot to
check. To watch from the outside while `up` runs, open a shell in the same
sandbox:

```bash
docker exec -it -u robot -w /workspace openrua-sandbox bash
ros2 topic echo /joint_states --once
```

## 4. Stop

An `exec` run stops by itself; its record stays under `runs/`.

`Ctrl-C` in the first terminal of `up` powers the robot off and removes both
containers. The workspace the agent left behind, snapshots and scripts
included, stays under `~/.openrua/sandboxes/openrua/workspace/workspace/`.

## Next

- [your-own-robot.md](../docs/your-own-robot.md): the same flow on your
  robot, from a profile drafted off its live graph.
- [running-experiments.md](../docs/running-experiments.md): what a recorded
  run holds, and `openrua bench`, which runs a benchmark's task set, one
  trial per task and seed, with the benchmark's own predicate deciding
  success.
