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

3. Bring it up with the task sentence the agent will be given, then open
   the agent in a second terminal:

   ```bash
   openrua up ./my-robot.yaml --ros-domain <id> --task "stack the red cube on the green one"
   openrua agent
   ```

   `openrua --robot ./my-robot.yaml --ros-domain <id>` opens OpenRUA's shared
   chat on the robot instead.

4. Or have the agent do the task on its own, with the run recorded:

   ```bash
   openrua exec ./my-robot.yaml "stack the red cube on the green one" --ros-domain <id>
   ```

   The agent reaches only its model service, through the image
   `openrua build proxy` makes; `--internet open` lifts that. The run lands
   under `runs/my-robot/<run id>/`: transcript, commands, workspace, the
   cameras' frames, `result.json` and `provenance.json`. Nothing resets the
   robot. At the end you are asked whether it did the task, with the agent's
   own last words beside the question, and the run records your answer
   (`verdict: person`); without a terminal to ask at, `success` stays `null`.

Interactive startup waits for ROS graph visibility. Recorded runs additionally
run preflight checks on the declared control interfaces, joint names, TF and
cameras. Neither check establishes safe physical operation; the robot and its
independent safety controls need validation on your hardware. See the
[robot guide](../docs/your-own-robot.md) for profile fields, driver ownership and
discovery troubleshooting.
