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
| LIBERO-10 (`libero_pro`, `libero_10_task` 4, seed 0) | put the yellow and white mug on the left plate and put the white mug on the right plate | Codex, GPT-6 Astra | [mp4](https://github.com/terminalworld/OpenRUA/releases/download/v0.0.7/libero10-mugs.mp4) (35 s) | [commands.sh](media/libero10-mugs-commands.sh) |
| LIBERO-10 (`libero_pro`, `libero_10_swap` 8, seed 1) | put both moka pots on the stove | Codex, GPT-6 Astra | [mp4](https://github.com/terminalworld/OpenRUA/releases/download/v0.0.7/libero10-moka-pots.mp4) (41 s) | [commands.sh](media/libero10-moka-pots-commands.sh) |
| LIBERO-10 (`libero_pro`, `libero_10_task` 2, seed 0) | turn on the stove and put the pan on it | Codex, GPT-6 Astra | [mp4](https://github.com/terminalworld/OpenRUA/releases/download/v0.0.7/libero10-stove-pan.mp4) (29 s) | [commands.sh](media/libero10-stove-pan-commands.sh) |
| RoboCasa365 (`robocasa365`, `PrepareCoffee`, seed 2) | pick the mug from the cabinet, place it under the coffee machine dispenser, and press the start button | Claude Code, Claude Opus 5 | [mp4](https://github.com/terminalworld/OpenRUA/releases/download/v0.0.7/robocasa365-coffee.mp4) (69 s) | [commands.sh](media/robocasa365-coffee-commands.sh) |

Every clip plays the robot at 8x and the terminal at 3x, starting 5 s
before the first motion; frames were recorded at 1280x960
(`--record-size`) and rendered at 1920x1080. The clips are assets of
[release v0.0.7](https://github.com/terminalworld/OpenRUA/releases/tag/v0.0.7).

To make one of your own:

```bash
openrua bench --config libero_pro --run-id demo --task-suite libero_goal_task \
            --task-ids 3 --seeds 0 --operator script \
            --script runs/libero_pro/main/trials/libero_goal_task-3/seed0/commands.sh --record
openrua demo runs/libero_pro/demo/trials/libero_goal_task-3/seed0 --speed 8 --pace 3 --from-motion 5
```

See [running-experiments.md](running-experiments.md#making-a-demo-video).
