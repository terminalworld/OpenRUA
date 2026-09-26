# tools/: minimal utilities

Six small Python tools for camera capture, geometric measurement, and
motion requests. Plain source files: read them, run them, adapt them,
or write your own programs using the native ROS interfaces (see docs).

| Tool | Usage | Operation |
|---|---|---|
| `perception/cam_snap.py` | `python3 tools/perception/cam_snap.py <camera_or_topic> [out.png]` | save a camera frame (color → PNG; depth → visualization PNG + numerical `.npy` array) |
| `perception/px2world.py` | `python3 tools/perception/px2world.py <camera> <u> <v>` | convert a selected depth pixel to world coordinates using camera intrinsics and TF |
| `action/fjt_send.py` | `python3 tools/action/fjt_send.py <p1,...,pN> <seconds>` | send a joint trajectory and wait for its result; joint names and port come from `machine.yaml` |
| `action/gripper_cmd.py` | `python3 tools/action/gripper_cmd.py <width_m>` | command a gripper position and print the result; `width_m` is the per-finger position |
| `action/ik_move.py` | `python3 tools/action/ik_move.py <x> <y> <z> <qx> <qy> <qz> <qw> [seconds]` | request inverse kinematics, then execute the joint trajectory; see `docs/30-action.md` for planning-frame conventions |
| `action/base_goto.py` | `python3 tools/action/base_goto.py <x> <y> [yaw_rad]` | drive a mobile base toward a world pose using odometry feedback; requires base and odometry ports in `machine.yaml` |
