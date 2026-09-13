---
summary: Recorded trials rendered as videos, one per task type
read_when:
  - You want to see what a trial looks like before running one
---

# Demos

Each entry is one trial from the reported runs, replayed with the
cameras recorded and rendered by `openrua demo`: the commands the agent
typed on the left, the robot's cameras on the right, on the trial's own
clock. The command file each was replayed from is linked.

| Benchmark | Task | Agent / model | Clip | Commands |
|:---|:---|:---|:---|:---|
| placeholder | placeholder | placeholder | [gif](media/placeholder.gif) · [mp4](media/placeholder.mp4) | [commands.sh](media/placeholder-commands.sh) |

To make one of your own:

```bash
openrua bench --config libero_pro --run-id demo --task-suite libero_goal_task \
            --task-ids 3 --seeds 0 --operator script \
            --script runs/libero_pro/main/trials/libero_goal_task-3/seed0/commands.sh --record
openrua demo runs/libero_pro/demo/trials/libero_goal_task-3/seed0 --gif --speed 8 --pace 3 --from-motion 5
```

See [running-experiments.md](running-experiments.md#making-a-demo-video).
