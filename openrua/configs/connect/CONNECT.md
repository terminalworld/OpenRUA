# Connecting this robot to OpenRUA

This folder becomes the robot's adaptation: the files OpenRUA brings the
robot up from and describes it to an agent with. A person is at the
robot. They do what only a person can (power, brakes, the vendor's web
interface, the emergency stop, cables, `sudo` on this computer) and
approve each command that changes something here.

## What this terminal reaches

- The coding project at `/workspace`, also mounted at its host path for driver builds.
  `docker run -v` takes host paths, so a mount names that one.
- The host's docker: images built and containers run here are the
  host's. Drivers run in containers, on the host network, so the robot's
  controller and its ROS 2 graph are reachable from here.
- The host's devices, read-only, under `/host/dev` (serial adapters under
  `/host/dev/serial/by-id`, cameras as `/host/dev/video*`); a driver
  container is given the device it needs with `--device`.
- No `sudo` on the host. A change there (a group, a real-time limit, a
  network setting) is a command for the person to run, followed by a new
  login where the change needs one.

Reference: `/openrua-connect/robot-schema.json` is the schema of
`robot.yaml` (every key with its meaning); `/openrua-connect/examples/`
holds robot types OpenRUA ships, for the facts an arm's entry carries.

## The files

| File | Holds |
|---|---|
| `robot.yaml` | how the drivers start and stop (`backend`), and what the machine is: arms, joints, ports, frames, gripper, cameras, notes |
| `driver/` | the drivers' Dockerfiles and the launch script `backend.launch` runs |
| `cameras/` | one camera type per camera model; `robot.yaml` names them by path |

`robot.draft.yaml` (written by `check.py probe`) is the live graph read
into the same format: a starting point for `robot.yaml`'s machine facts.

## Steps

Each step is done when its check passes: `python3 /openrua-connect/check.py <step>`.
OpenRUA runs the check on the host and prints the result; the full output
is in the project's `.openrua/connect/<session>/<step>.log`.

1. **This computer** (`host`): docker, disk space, the real-time limit,
   the devices present. What fails comes with the command that fixes it.
2. **What the robot is**: the arm's make, model and controller or
   firmware version, the gripper, the cameras and their serial numbers.
   The firmware version decides which driver release talks to it. The
   person confirms the list before drivers are chosen.
3. **Drivers** (`drivers`): a ROS 2 driver for each part, the vendor's or
   the community's, in a version that matches what step 2 found, built as
   an image from `driver/` and named under `backend.build`. Each part,
   started on its own and only reading, shows its data (joint states,
   images, the gripper's state) before parts are combined.
   Build names resolve to content-addressed tags. Pass the resolved image
   into a host launch script explicitly, for example
   `launch: python3 driver/up.py --image {images[driver]}` with
   `build: {driver: driver}`; let the script accept `--image` rather than
   hard-code the build name. If `backend.image` names a build, OpenRUA
   resolves it to the same tag.
4. **Together** (`probe`, then `config`): one command (`backend.launch`)
   starts every driver and stops them all on SIGINT, releasing the arm
   first if its driver needs that; names that collide between drivers are
   namespaced apart. Set the driver's ROS distro, discovery settings and
   sandbox dependencies in `backend` first. With the robot up, `probe`
   reads that connection using its sandbox image into `robot.draft.yaml`.
   Other robot facts and camera files may still be incomplete. A successful
   probe means a draft was saved, not that the intended robot was verified.
   Review the observed model and interfaces before `robot.yaml` takes its facts from there,
   plus what the graph cannot say (which end of the gripper's travel is
   open, what each camera is and where it is mounted, notes on how the
   robot behaves), and `backend.startup_checks`: what has to be true
   before the robot can start (its controller answers, each device is
   plugged in), each a command run on this computer with what a person
   does when it fails. OpenRUA runs them before every start. `config`
   passes when it loads with no TODO left.
5. **The robot from its config** (`robot`): OpenRUA builds what is
   missing, starts the robot from `robot.yaml` twice, stopping it in
   between, and checks the live graph against what `robot.yaml`
   declares. Starting it twice is what shows that stopping leaves the
   robot ready to start again. The robot moves during this check only if
   the launch command moves it.
6. **First motion** (`done`, then in the robot's own terminal): `done`
   passes when every check above has passed; the session then moves to
   the robot's own terminal (no docker there) and the conversation goes
   on. With the person's word that the emergency stop is at hand, the
   gripper opens and closes once and one joint moves a few degrees.

Only robots with a ROS 2 driver are in scope.
