# Workspace: start here

You are on the onboard computer of a robot arm workstation. This
workspace ships structured startup docs and a few generic tools.

## Map

| Read | For |
|---|---|
| [machine.yaml](machine.yaml) | THIS machine's facts: robot model, ports, joints, limits, frames |
| [docs/10-machine.md](docs/10-machine.md) | how to read machine.yaml; software stack, runtime environment, timing |
| [docs/20-perception.md](docs/20-perception.md) | how to sense: joints, poses, force, cameras |
| [docs/30-action.md](docs/30-action.md) | how to move: trajectories, servo, gripper, mobile base, planning |
| [docs/40-patterns.md](docs/40-patterns.md) | reliable working patterns and troubleshooting |
| [tools/README.md](tools/README.md) | ready-made utilities (perception/ and action/) |

## 30-second start

```bash
ros2 topic list                                    # what the machine exposes
ros2 topic echo /joint_states --once               # where the arm is
python3 tools/perception/cam_snap.py agentview     # what the scene looks like
```

Docs are numbered in reading order; each capability section follows the
same shape: what it is → port → `ros2` CLI → `rclpy` → tool shortcut →
practical notes.
