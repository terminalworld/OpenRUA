---
summary: The first-task flow on a real ROS 2 arm, in three commands
read_when:
  - You have a robot profile and want the shortest path to an agent on the hardware
---

# A real robot

1. With the robot's ROS 2 stack running, draft a profile from its graph
   and finish the `TODO` lines (see `docs/your-own-robot.md`):

   ```bash
   openrua probe --host > my-robot.yaml
   ```

2. Check it loads and the images and login are in place:

   ```bash
   openrua doctor ./my-robot.yaml
   ```

3. Bring it up with the task sentence the agent will be given, then open
   the agent in a second terminal:

   ```bash
   openrua up ./my-robot.yaml --ros-domain <id> --task "stack the red cube on the green one"
   openrua agent
   ```

Preflight runs the same checks as in simulation: the ports you listed
must be served, joint names must match the profile, TF and camera frames
must flow. If a check fails, `up` refuses and says which promise the
robot did not keep.

`openrua bench --config <benchmark> --robot ./my-robot.yaml --task "..."`
runs the same trial loop on hardware: no reset, no automatic verdict
(`success: null`, `verdict: not_applicable`), everything else recorded.
