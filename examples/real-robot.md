---
summary: The first-task flow on a real ROS 2 arm, in a few commands
read_when:
  - You have a robot profile and want the shortest path to an agent on the hardware
---

# A real robot

The shortest path is `openrua connect` (or **Connect a real robot** in the
setup menu): your coding agent connects the robot with you and writes its
profile, drivers and startup checks, step by step
([Connect a real robot](../docs/connect.md)). The steps below are for a
profile you write yourself.

1. With the robot's ROS 2 stack running, draft a profile from its graph
   and finish the `TODO` lines (see `docs/your-own-robot.md`):

   ```bash
   openrua probe --host --ros-domain <id> --distro <humble-or-jazzy> > my-robot.yaml
   ```

2. Check it loads, its startup checks pass, and the images and login are in
   place:

   ```bash
   openrua doctor ./my-robot.yaml
   ```

3. Talk to the agent on the robot in OpenRUA's terminal:

   ```bash
   openrua --robot ./my-robot.yaml --ros-domain <id>
   ```

   `openrua config set --robot ./my-robot.yaml` makes it the default, so
   plain `openrua` starts it. To use the agent's own original terminal
   instead, run `openrua agent --robot ./my-robot.yaml --ros-domain <id>`:
   it starts the robot, and as the agent exits asks whether to turn it off.
   There Esc stops the agent but not the arm's motion, so keep the emergency
   stop at hand.

4. Or have the agent do the task on its own, with the run recorded:

   ```bash
   openrua exec ./my-robot.yaml "stack the red cube on the green one" --ros-domain <id>
   ```

   The agent reaches only its model service, through the image
   `openrua build proxy` makes; `--internet open` lifts that. The run lands
   under `.openrua/runs/<run id>/`: transcript, commands, the
   cameras' frames, `result.json` and `provenance.json`. Nothing resets the
   robot. At the end you are asked whether it did the task, with the agent's
   own last words beside the question, and the run records your answer
   (`verdict: person`); without a terminal to ask at, `success` stays `null`.
   The current directory is the project workspace, shared with interactive
   sessions; its files remain after shutdown. Use `-C DIR` to choose another
   project and `--trust-workspace` for unattended first use.

Interactive startup waits for ROS graph visibility. Recorded runs additionally
run preflight checks on the declared control interfaces, joint names, TF and
cameras. Neither check establishes safe physical operation; the robot and its
independent safety controls need validation on your hardware. See the
[robot guide](../docs/your-own-robot.md) for profile fields, driver ownership and
discovery troubleshooting.
