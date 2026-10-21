---
summary: The first-task flow on a real ROS 2 arm, in three commands
read_when:
  - You have a robot profile and want the shortest path to an agent on the hardware
---

# A real robot

To have your coding agent connect the robot with you, run `openrua connect`
([Connect a real robot](../docs/connect.md)); it writes the profile, its
drivers and its startup checks, step by step. The steps below are for a
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

Interactive startup waits for ROS graph visibility. Benchmark trials additionally
run preflight checks on the declared control interfaces, joint names, TF and
cameras. Neither check establishes safe physical operation; the robot and its
independent safety controls need validation on your hardware. See the
[robot guide](../docs/your-own-robot.md) for profile fields, driver ownership and
discovery troubleshooting.

`openrua bench --config <benchmark> --robot ./my-robot.yaml --run-id hw-1
--task-suite <suite> --task "..."` runs the same trial loop on hardware: no
reset, no automatic verdict (`success: null`, `verdict: not_applicable`),
everything else recorded.
