---
summary: Bring the simulated Panda up, hand your coding agent a task, watch what it does
read_when:
  - You have installed OpenRUA and want to see it work once
  - You want to know what the agent actually sees and types
---

# First task in simulation

Twenty minutes: a simulated Franka Panda with a live ROS 2 graph, your
coding agent on its terminal, one task. [Install](../docs/install.md)
first; `openrua doctor panda-sim` must be green.

## 1. One command

```bash
openrua run panda-sim "open the middle drawer of the cabinet"
```

The robot container boots its scene and MoveIt (about a minute), the
sandbox is created with the workspace seeded, and the agent opens on
it with the sentence as its opening message. When you leave the agent
the robot powers off. The rest of this page walks the same session in
its two-terminal form, which keeps the robot up between sessions.

## 1a. Bring the robot up

```bash
openrua up panda-sim
```

The command stays in the foreground:

```
[up] ready.
     robot     openrua-sim   (ROS 2 graph live; scene: libero_goal_task #0)
     terminal  openrua-sandbox
     task      open the middle drawer of the cabinet

     openrua agent --name openrua            # your coding agent, on the robot
     docker exec -it -u robot -w /workspace openrua-sandbox bash   # or you

     Ctrl-C here powers the robot off.
```

`--task-suite` and `--task-id` pick another scene from the benchmark;
`--init-state` another initial layout of the same scene.

## 1b. Hand the agent the task

In a second terminal:

```bash
openrua agent "open the middle drawer of the cabinet"
```

This opens the agent `up` was configured with (Claude Code by default,
`--agent codex` for Codex) inside the sandbox, as user `robot` in
`/workspace`, with the sentence as its opening message. Nothing else is
injected: no tool list, no API, no planner. The agent's only route to
the internet is the proxy to its model API.

## 2. What the agent sees

```
/workspace
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

`machine.yaml` is generated from the robot's profile, so it never
disagrees with the graph the agent is looking at; preflight checked
every claim in it before the agent started. The tools are ordinary
`rclpy` scripts the agent may read, copy or ignore.

## 3. What a session looks like

The first commands of a real session (Codex, from a trial's
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
check. To watch from the outside, open a shell in the same sandbox:

```bash
docker exec -it -u robot -w /workspace openrua-sandbox bash
ros2 topic echo /joint_states --once
```

## 4. Stop

`Ctrl-C` in the first terminal powers the robot off and removes both
containers. The workspace the agent left behind, snapshots and scripts
included, stays under `~/.openrua/workspaces/openrua/`.

## Next

- [your-own-robot.md](../docs/your-own-robot.md): the same flow on your
  robot, from a profile drafted off its live graph.
- [running-experiments.md](../docs/running-experiments.md): the scored
  version, `openrua bench`, with the benchmark's own predicate deciding
  success and every trial recorded.
